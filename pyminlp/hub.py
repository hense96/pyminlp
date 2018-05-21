# This module coordinates all user related operations.


import heapq

from pyminlp.subprob import Instance
from pyminlp.bnb import BranchAndBound
from pyminlp.bnb import UserInputStatus


class Coordinator:
    """
    This class coordinates all user related operations.
    It offers interface functions for the solving process, calls and
    coordinates the user plugins and processes the data that the user
    passes via the solver interface.

    Use the factory method _Coordinator.create(...) for generating a
    new instance.

    Private class attributes:
      1.: User given data
        _py_model     The user given pyomo model (ConcreteObject) to
                          solve.
        _relaxation_solver
                      The user given relaxation solver object.
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
      3.: Current data
        _cur_instance The Instance object that the solver is currently
                          working on.
        _hdlr_obj     The dictionary of constraint handler objects that
                          do only change when a new _cur_instance is
                          registered. Key of the dict is the
                          conshandler name, value the object.
        _curdata      Stack of _Curdata objects, each object storing
                          needed current data one user function call.
    """

    @classmethod
    def create(cls, relaxation_solver=None, options=None):
        """Factory method to generate a new Coordinator object.
        Parameters as described above.
        """
        coord = Coordinator()
        coord._relaxation_solver = relaxation_solver
        coord._options = options
        return coord

    def __init__(self):
        """Constructor setting attributes to None."""
        # User data.
        self._py_model = None
        self._relaxation_solver = None
        self._hdlrs_ident = []
        self._hdlrs_enf = []
        self._options = None
        self._epsilon = None
        # References.
        self._bnb_tree = None
        # Current data.
        self._cur_instance = None
        self._hdlr_obj = {}
        self._curdata = []

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
            self._get_hdlr(hdlr).identify(sets, model)
            self._curdata.pop()

            # Handle user input.
            # TODO more precisely.
            if curdata.ncuts > 0 or curdata.ntighten > 0 or curdata.branched\
                    or curdata.infeasible:
                raise Exception('Unallowed interface operation during '
                                'identify process.')
        if instance.nunclassified() == 0:
            return UserInputStatus.OK
        else:
            raise Exception('After calling all constraint handlers, '
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

        Unaccepted user operations: Branching.

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
            self._get_hdlr(hdlr).prepare(sets, model)
            self._curdata.pop()

            # Handle user input.
            # TODO Exception more precisely.
            # TODO handle other cases, e.g. direct branching.
            if curdata.infeasible:
                return UserInputStatus.INFEASIBLE
            elif curdata.branched:
                raise Exception('Unallowed interface operation during '
                                'prepare process.')
        return UserInputStatus.OK

    def solve_relaxation(self):
        """Calls the relaxation solving method of the registered
        instance and passes the user defined relaxation solver.

        :return: A UserInputStatus.
        """
        assert self._cur_instance is not None
        instance = self._cur_instance
        instance.solve_relaxation(self._relaxation_solver, self._epsilon)
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
                self._get_hdlr(hdlr).enforce(sets, model)
                self._curdata.append(curdata)

                # Handle user input.
                if curdata.infeasible:
                    return UserInputStatus.INFEASIBLE
                elif curdata.branched:
                    self._bnb_tree.register_child_instances(curdata.children)
                    return UserInputStatus.BRANCHED
                elif curdata.ncuts > 0 or curdata.ntighten > 0:
                    return UserInputStatus.RESOLVE

        raise Exception('enforce failed.')

    # Miscellaneous functions.

    def _get_hdlr(self, conshandler):
        """Returns the current handler object of the given constraint
        handler manager.
        """
        assert conshandler.name() in self._hdlr_obj.keys()
        return self._hdlr_obj[conshandler.name()]

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
            raise Exception('identify failed for constraints added in '
                            'prepare method.')

    def branch(self, var=None, point=None):
        """Function to perform branching on the current instance object.
        See solver module for precise explanation.
        """
        # Read relevant current data.
        instance = self._cur_instance
        curdata = self._curdata.pop()
        self._curdata.append(curdata)
        # TODO maybe allow more sophisticated branching expressions.
        # Perform operation.
        if var is not None and point is not None:
            left = Instance.derive_instance(instance)
            left.change_varbounds(var, upper_bound=point)
            right = Instance.derive_instance(instance)
            right.change_varbounds(var, lower_bound=point)
            # Update current data.
            curdata.add_child(left)
            curdata.add_child(right)
        curdata.set_branched()

    def change_bound(self, var, lower=None, upper=None):
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

    def add_conshandler(self, conshandler):
        """Function to register that a conshandler is used.
        :param conshandler: A ConsHandler object.
        """
        heapq.heappush(self._hdlrs_ident,
                       (conshandler.identify_prio(), conshandler))
        heapq.heappush(self._hdlrs_enf,
                       (conshandler.enforce_prio(), conshandler))

    def register_relaxation_solver(self, relaxation_solver):
        """Registers a relaxation solver."""
        self._relaxation_solver = relaxation_solver

    def set_epsilon(self, epsilon):
        """Registers the epsilon for the solving process.
        The epsilon is the tolerance applied to constraint bounds when
        determining whether they are met or not.
        """
        self._epsilon = epsilon

    def solve(self, py_model):
        """Initiates the solving process. Calls the branch and bound
        algorithm.
        :param py_model: Model to solve.
        """
        self._py_model = py_model
        # Create instance.
        instance = Instance.create_instance(py_model.clone())
        # Create bnb_tree.
        self._bnb_tree = BranchAndBound.create(self)
        self._bnb_tree.register_root_instance(instance)
        # Start solving process.
        self._bnb_tree.execute()
        print('Done')


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
