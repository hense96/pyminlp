# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


import heapq
from enum import Enum

from pyomo.opt.results.results_ import SolverResults
from pyomo.opt.results.solution import Solution, SolutionStatus
from pyomo.opt.results.solver import TerminationCondition, SolverStatus
import pyomo.core.kernel
from pyomo.environ import *

from pyminlp.conshdlr import *
from pyminlp.subprob import Instance
from pyminlp.bnb import BranchAndBound
from pyminlp.bnb import UserInputStatus
from pyminlp.bnb import BnBResult
from pyminlp.stats import Stats


class Coordinator:
    """
    This class coordinates all user related operations.
    It offers interface functions for the solving process, calls and
    coordinates the user plugins and processes the data that the user
    passes via the solver interface.

    Use the factory method _Coordinator.create(...) for generating a
    new instance.

    Private class attributes:
      1.: User related data
        _py_model     The user given pyomo model (ConcreteObject) to
                          solve.
        _result       The Pyomo SolverResult object providing data
                          about the result of the solving process.
        _relaxation_solver
                      The user given relaxation solver object.
        _plugin_list  The list of known constraint handlers, i.e.
                          user defined IPyMINLPConsHandler service
                          objects.
        _hdlrs_ident  The priorised heapq of used constraint handlers
                          ordered by identify priority.
                          Elements on this list have the format:
                              (prio, conshandler) with type
                              (int,  ConsHandler)
        _hdlrs_enf    The priorised heapq of used constraint handlers
                          ordered by enforce priority.
                          Elements on this list have the format:
                              (prio, conshandler) with type
                              (int,  ConsHandler)
        _options      The options object.
      2.: References
        _bnb_tree     The branch and bound algorithm object
                          (BranchAndBound object)
        _solver       The solver interface object.
      3.: Current data
        _cur_instance The Instance object that the solver is currently
                          working on.
        _hdlr_obj     The dictionary of constraint handler objects that
                          do only change when a new _cur_instance is
                          registered. Key of the dict is the
                          conshandler name, value the object.
        _curdata      Stack of _Curdata objects, each object storing
                          needed current data one user function call.

    Public class attributes:
      4.: Status data
        stage        The stage of the solving process.
    """

    @classmethod
    def create(cls, solver):
        """Factory method to generate a new Coordinator object.
        Parameters as described above.
        """
        coord = Coordinator()
        coord._solver = solver
        # Set up plugin structure.
        coord._plugin_list = ExtensionPoint(IPyMINLPConsHandler)
        return coord

    def __init__(self):
        """Constructor setting attributes to None."""
        # User related data.
        self._py_model = None
        self._result = None
        self._relaxation_solver = None
        self._plugin_list = None
        self._hdlrs_ident = []
        self._hdlrs_enf = []
        self._options = None
        self._gap_epsilon = None
        self._cons_epsilon = None
        self._verbosity = None
        self._time_limit = None
        # References.
        self._bnb_tree = None
        self._solver = None
        # Current data.
        self._cur_instance = None
        self._hdlr_obj = {}
        self._curdata = []
        # Status data.
        self._stage = SolvingStage.SETUP

    # Interface functions for branch and bound.

    def set_instance(self, instance):
        """Sets a new current instance and creates new constraint
        handler objects to handle this instance. All other operations
        will be performed on this registered instance.
        :param instance: The currently relevant Instance object."""
        assert type(instance) is Instance
        self._cur_instance = instance
        for (_, hdlr) in self._hdlrs_ident:
            self._hdlr_obj[hdlr.name()] = hdlr.generate()

    def identify(self):
        """Calls the identify methods of the constraint handlers for
        all unclassified constraints in the registered current instance.

        The identify methods of the constraint handlers are called. The
        order is given by the identify priority order (smaller priority
        first). All indices of unclassified constraints are passed
        using the sets parameter. Before calling a plugin function, a
        current data object is appended to the _curdata stack.

        Unaccepted user operations: Branching, bounds tightening,
        adding constraints, declaring infeasibility.

        :return: A UserInputStatus.
        """
        assert self._cur_instance is not None
        instance = self._cur_instance
        # Iterate over all constraint handlers in the specified order.
        for (_, hdlr) in self._hdlrs_ident:
            # Generate unclassified constraint sets.
            sets = {}
            model = instance.model()
            for constype in hdlr.constypes():
                sets[constype] = instance.unclassified(constype)
            # Set up current data.
            curdata = _CurrentData.create(hdlr)
            # Call identify method.
            self._curdata.append(curdata)
            self._get_hdlr(hdlr).identify(sets, model, self._solver)
            self._curdata.pop()

            Stats.identify(self, curdata)

            # Handle user input.
            if curdata.ncuts > 0 or curdata.ntighten > 0 or curdata.branched\
                    or curdata.infeasible:
                raise UserInputError('Unallowed interface operation during '
                                     'identify process.')
        if instance.nunclassified() == 0:
            return UserInputStatus.OK
        else:
            raise UserInputError('After calling all constraint handlers, '
                                 'there are still unclassified constraints.')

    def prepare(self):
        """Calls the prepare methods of the constraint handlers for
        all of their constraints in the registered current instance.

        The prepare methods of the constraint handlers are called. The
        order is given by the enforce priority order (smaller priority
        first). All indices of constraints belonging to the handler are
        passed using the sets parameter. Before calling a plugin
        function, a current data object is appended to the _curdata
        stack.

        Unaccepted user operations: None.

        :return: A UserInputStatus.
        """
        assert self._cur_instance is not None
        instance = self._cur_instance
        # Iterate over all constraint handlers in the specified order.
        for (_, hdlr) in self._hdlrs_enf:
            # Generate constraint handler sets.
            sets = instance.conshandler_sets(hdlr)
            model = instance.model()
            # Set up current data.
            curdata = _CurrentData.create(hdlr)
            # Call prepare method.
            self._curdata.append(curdata)
            self._get_hdlr(hdlr).prepare(sets, model, self._solver)
            self._curdata.pop()

            Stats.prepare(self, curdata)

            # Handle user input.
            if curdata.infeasible:
                return UserInputStatus.INFEASIBLE
            elif curdata.branched:
                return UserInputStatus.BRANCHED
            elif curdata.nmatched > 0:
                raise UserInputError('Unallowed interface operation during '
                                     'prepare process.')
        return UserInputStatus.OK

    def solve_relaxation(self):
        """Calls the relaxation solving method of the registered
        instance and passes the user defined relaxation solver.

        :return: A UserInputStatus.
        """
        assert self._cur_instance is not None
        instance = self._cur_instance
        instance.solve_relaxation(self._relaxation_solver, self._cons_epsilon)
        return UserInputStatus.OK

    def enforce(self):
        """Calls the enforce methods of the constraint handlers for
        all violated constraints in the registered current instance.

        The prepare methods of the constraint handlers that are not
        part of the relaxation are called. The order is given by the
        enforce priority order (smaller priority first). All indices of
        constraints belonging to the handler are passed using the sets
        object. Before calling a plugin function, a current data
        object is appended to the _curdata stack.

        Unaccepted user operations: None.

        :return: A UserInputStatus.
        """
        assert self._cur_instance is not None
        instance = self._cur_instance
        # Iterate over all constraint handlers in the specified order.
        for (_, hdlr) in self._hdlrs_enf:
            if not hdlr.relax() and instance.nviolated(hdlr) > 0:
                # Generate constraint handler sets.
                sets = instance.violated_sets(hdlr)
                # Only possible to do inside if resolve after every
                # conshandler that performs changes.
                model = instance.relax_model()
                # Set up current data if not already existing.
                curdata = _CurrentData.create(hdlr)
                # Call enforce method.
                self._curdata.append(curdata)
                self._get_hdlr(hdlr).enforce(sets, model, self._solver)
                self._curdata.append(curdata)

                Stats.separate(self, curdata)

                # Handle user input.
                if curdata.infeasible:
                    return UserInputStatus.INFEASIBLE
                elif curdata.branched:
                    self._bnb_tree.register_child_instances(curdata.children)
                    return UserInputStatus.BRANCHED
                elif curdata.ncuts > 0 or curdata.ntighten > 0:
                    return UserInputStatus.RESOLVE

        raise UserInputError('Enforcement failed. No constraint handler '
                             'called an interface function.')

    def time_limit_reached(self):
        return Stats.get_solver_time() > self._time_limit

    # Interface functions for plugin functions.

    def add_constraints(self, constype, conss, params={}):
        """Function to add constraints to the current instance object.
        See solver module for precise explanation.
        """
        # Read relevant current data.
        instance = self._cur_instance
        curdata = self._curdata.pop()
        self._curdata.append(curdata)
        # Perform operation on instance.
        instance.add_constraints(constype, conss, params)
        # Update current data.
        curdata.add_cuts(len(conss))
        # Follow up operations.
        res = self.identify()
        if res != UserInputStatus.OK:
            raise UserInputError('identify failed for constraints added in '
                                 'prepare method.')

    def branch(self, var, point):
        """Function to perform branching on the current instance object.
        See solver module for precise explanation.
        """
        # Read relevant current data.
        instance = self._cur_instance
        curdata = self._curdata.pop()
        self._curdata.append(curdata)
        # Perform operation.
        left = Instance.derive_instance(instance)
        left.change_varbounds(var, upper_bound=point)
        right = Instance.derive_instance(instance)
        right.change_varbounds(var, lower_bound=point)
        # Update current data.
        curdata.add_child(left)
        curdata.add_child(right)
        curdata.set_branched()

    def change_bounds(self, var, lower=None, upper=None):
        """Function to change a variable's bounds on the current
        instance object. See solver module for precise explanation.
        """
        # Read relevant current data.
        instance = self._cur_instance
        curdata = self._curdata.pop()
        self._curdata.append(curdata)
        # Perform operation.
        instance.change_varbounds(var, lower_bound=lower, upper_bound=upper)
        # Update current data.
        curdata.tighten()

    def declare_infeasible(self):
        """Function to declare that the instance is actually infeasible.
        """
        # Read relevant current data.
        curdata = self._curdata.pop()
        self._curdata.append(curdata)
        # Update current data.
        curdata.set_infeasible()

    def match(self, constraint):
        """Function to assign a constraint to the current conshandler.
        See solver module for precise explanation.
        """
        # Read relevant current data.
        instance = self._cur_instance
        curdata = self._curdata.pop()
        self._curdata.append(curdata)
        # Perform operation.
        instance.register_conshandler(constraint, curdata.handler)
        # Update current data.
        curdata.add_match()

    # Interface functions for solver setup.

    def add_conshandler(self, name, constypes, id_prio, enf_prio, relax):
        """Function to register that a conshandler is used. See solver
        module for precise explanation.
        """
        # Check if handler is already included.
        for (_, hdlr) in self._hdlrs_ident:
            if hdlr.name() == name:
                raise ValueError('Constraint handler {} is already included.'
                                 ''.format(name))
        # New plugin structure.
        for plugin in self._plugin_list:
            if name == plugin.name:
                self._check_plugin_sanity(plugin)
                hdlr = ConsHandlerManager.create(name, constypes, id_prio,
                                                 enf_prio, relax, plugin)
                # Old plugin structure.
                heapq.heappush(self._hdlrs_ident,
                               (hdlr.identify_prio(), hdlr))
                heapq.heappush(self._hdlrs_enf,
                               (hdlr.enforce_prio(), hdlr))
                return
        raise ValueError('Constraint handler {} is not known.'
                         ''.format(name))

    def register_relaxation_solver(self, relaxation_solver):
        """Registers a relaxation solver."""
        self._relaxation_solver = relaxation_solver

    def set_gap_epsilon(self, epsilon):
        """Registers the gap epsilon value for the solving process.
        :param epsilon: When lower and upper bound of the optimal
        objective function value of a (sub-)problem differ in less than
        the gap epsilon, the (sub-)problem is considered solved.
        """
        self._gap_epsilon = epsilon

    def set_cons_epsilon(self, epsilon):
        """Registers the constraint epsilon value for the solving
        process.
        :param epsilon: The tolerance applied to constraint bounds
        when determining whether they are met or not.
        """
        self._cons_epsilon = epsilon

    def set_verbosity(self, verbosity):
        """Sets the verbosity, i.e. the how much information regarding
        the solving process the solver provides."""
        self._verbosity = verbosity

    def set_time_limit(self, time_limit):
        self._time_limit = time_limit

    def solve(self, py_model):
        """Initiates the solving process. Calls the branch and bound
        algorithm.
        :param py_model: Model to solve.
        """
        Stats.initialise(verbosity=self._verbosity)
        self._py_model = py_model
        # Create instance.
        instance = Instance.create_instance(py_model.clone())
        # Create bnb_tree.
        self._bnb_tree = BranchAndBound.create(self)
        self._bnb_tree.register_root_instance(instance)
        # Check sanity of user setup.
        self._check_setup_sanity(instance)
        # Start solving process.
        self._stage = SolvingStage.SOLVING
        res = self._bnb_tree.execute(self._gap_epsilon)
        self._stage = SolvingStage.DONE
        self._postprocess(res)
        self._stage = SolvingStage.SETUP
        return self._result

    # Information for solver interface.

    @property
    def stage(self):
        return self._stage

    # Miscellaneous functions.

    def _check_plugin_sanity(self, plugin):
        """Checks whether the given user plugin fulfills syntactical
        requirements.
        :param plugin: A service object for IPyMINLPConsHandler.
        """
        # Check if plugin has all required functions.
        for method in ['identify', 'prepare', 'enforce']:
            if not hasattr(plugin, method) \
                   or not callable(getattr(plugin, method)):
                raise UserInputError('The constraint handler {} does not '
                                     'implement a {} method.'
                                     ''.format(plugin.name, method))

    def _check_setup_sanity(self, instance):
        """Checks whether the minimum solver set up requirements for
        solving an instance are fulfilled.

        :param instance: The instance object belonging to the root node.
        """
        # Check if constraint handlers cover all constraint types.
        constypes_inst = set(instance.constypes())
        constypes_hdlrs = set([])
        for (_, hdlr) in self._hdlrs_ident:
            cur_types = hdlr.constypes()
            for type in cur_types:
                constypes_hdlrs.add(type)
        if constypes_inst != constypes_hdlrs:
            diff = constypes_inst.difference(constypes_hdlrs)
            if len(diff) > 0:
                raise ValueError('No constraint handler covers the following '
                                 'constraint types: {}.'.format(diff))
        # Check if a relaxation solver is registered.
        if self._relaxation_solver is None:
            raise ValueError('No relaxation solver is set.')

    def _get_hdlr(self, conshandler):
        """Returns the current handler object of the given constraint
        handler manager.
        """
        assert conshandler.name() in self._hdlr_obj.keys()
        return self._hdlr_obj[conshandler.name()]

    def _postprocess(self, result):
        """Create a SolverResult object, save it as _result and write
        solution data onto the user given Pyomo object (_py_model).
        :param result: The BnBResult indicating the result of the
        branch and bound process.
        """
        assert self._stage == SolvingStage.DONE
        assert type(result) is BnBResult

        # Copy solution to model.
        optinst = self._bnb_tree.best_instance()
        if optinst is not None:
            optinst.write_solution(self._py_model)

        # Create result object.
        self._result = SolverResults()
        soln = Solution()

        # Set some result meta data.
        self._result.solver.name = ('PyMINLP')
        self._result.solver.wallclock_time = Stats.get_solver_time()

        # Set solver status.
        if result == BnBResult.INFEASIBLE:
            self._result.solver.status = SolverStatus.ok
            self._result.solver.termination_condition \
                = TerminationCondition.infeasible
            soln.status = SolutionStatus.infeasible
        elif result == BnBResult.OPTIMAL:
            self._result.solver.status = SolverStatus.ok
            self._result.solver.termination_condition \
                = TerminationCondition.optimal
            soln.status = SolutionStatus.optimal
        elif result == BnBResult.TIMEOUT:
            self._result.solver.status = SolverStatus.ok
            self._result.solver.termination_condition \
                = TerminationCondition.maxTimeLimit
            soln.status = SolutionStatus.stoppedByLimit

        if result == BnBResult.OPTIMAL or result == BnBResult.TIMEOUT:
            # Set solution data.
            self._result.problem.sense = pyomo.core.kernel.minimize
            self._result.problem.lower_bound = self._bnb_tree.lower_bound()
            self._result.problem.upper_bound = self._bnb_tree.upper_bound()
            soln.gap = self._bnb_tree.upper_bound() \
                       - self._bnb_tree.lower_bound()

            if optinst is not None:
                # Set objective data.
                obj = self._py_model.component_objects(Objective)
                for o in obj:
                    obj_name = o.name
                    obj_val = value(o)
                    break
                soln.objective[obj_name] = {'Value': obj_val}

                # Set variable data.
                #for vartype in self._py_model.component_objects(Var):
                #    for v in vartype:
                #        name = vartype[v].name
                #        val = value(vartype[v])
                #        soln.variable[name] = {'Value': val}

        # Set problem instance data.
        self._result.problem.name = self._py_model.name
        self._result.problem.number_of_constraints = \
            self._py_model.nconstraints()
        # self._result.problem.number_of_nonzeros = None
        self._result.problem.number_of_variables = self._py_model.nvariables()
        # self._result.problem.number_of_binary_variables = None
        # self._result.problem.number_of_integer_variables = None
        # self._result.problem.number_of_continuous_variables = None
        self._result.problem.number_of_objectives = \
            self._py_model.nobjectives()

        # Set branch and bound data.
        nsubprob = 'Number of created subproblems'
        self._result.solver.statistics.branch_and_bound[nsubprob] = \
            self._bnb_tree.nopennodes() + self._bnb_tree.nconsiderednodes()
        nsubprob = 'Number of considered subproblems'
        self._result.solver.statistics.branch_and_bound[nsubprob] = \
            self._bnb_tree.nconsiderednodes()

        self._result.solution.insert(soln)


class SolvingStage(Enum):
    """Enumeration for the stage of the solving process."""
    SETUP = 1
    SOLVING = 2
    DONE = 3


class SolvingStageError(Exception):
    pass


class UserInputError(Exception):
    pass


class _CurrentData:
    """
    This class is used to register user given data that are provided
    within a single plugin function call. It saves all solver status
    data that are needed to evaluate the interface function calls
    performed by the user as well as the meta information about what
    decisions the user makes.

    Use the factory method _CurrentData.create(...) for generating a new
    instance.

    Public class attributes:
      1) Solver status data
        handler     The current conshandler (ConsHandlerManager).
      2) User given data
        nmatched    The number of constraints assigned to a constraint
                        handler by the user.
        ncuts       The number of cuts added by the user.
        ntighten    The number of tighten_bounds calls by the user.
        branched    Flag indicating if the instance is branched by the
                        user.
        children    Children instances created by the user.
        infeasible  Flag indicating if the instance is declared
                        infeasible by the user.
    """

    @classmethod
    def create(cls, handler):
        """Factory method to generate a new _Solution object. Parameters
        as described above.
        """
        data = _CurrentData()
        data._handler = handler
        return data

    def __init__(self):
        """Constructor setting attributes to default value."""
        # Solver status data.
        self._handler = None
        # User given data.
        self._nmatched = 0
        self._ncuts = 0
        self._ntighten = 0
        self._branched = False
        self._children = []
        self._infeasible = False

    @property
    def handler(self):
        return self._handler

    @property
    def nmatched(self):
        return self._nmatched

    def add_match(self):
        """Increments the number of constraints assigned to a
        conshandler.
        """
        self._nmatched += 1

    @property
    def ncuts(self):
        return self._ncuts

    def add_cuts(self, amount):
        """Increases the number of added cuts by the given amount.
        :param amount: An int.
        """
        assert type(amount) is int
        assert amount >= 0
        self._ncuts += amount

    @property
    def ntighten(self):
        return self._ntighten

    def tighten(self):
        """Increments the number of tighten_bounds calls."""
        self._ntighten += 1

    @property
    def branched(self):
        return self._branched

    def set_branched(self):
        """Sets the branched flag to True."""
        self._branched = True

    @property
    def children(self):
        return self._children

    def add_child(self, child):
        """Adds a child instance to the list of childs.
        :param child: An Instance object.
        """
        assert type(child) is Instance
        self._children.append(child)

    @property
    def infeasible(self):
        return self._infeasible

    def set_infeasible(self):
        """Sets the infeasible flag to True."""
        self._infeasible = True
