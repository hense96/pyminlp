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
        _py_model    The user given pyomo model (ConcreteObject) to
                         solve.
        _relaxation_solver
                     The user given relaxation solver object.
        _hdlrs_ident The priorised heapq of used constraint handlers
                         ordered by identify priority.
                         Elements on this list have the format:
                             (prio, conshandler) with type
                             (int,  ConsHandler)
        _hdlrs_enf   The priorised heapq of used constraint handlers
                         ordered by enforce priority.
                         Elements on this list have the format:
                             (prio, conshandler) with type
                             (int,  ConsHandler)
        _options     The options object.
      2.: References
        _bnb_tree    The branch and bound algorithm object
                         (BranchAndBound object)
      3.: Current data
        _curdata     Dict of _Curdata objects storing all necessary
                         current data before calling functions from
                         other modules. Key of the dictionary is the
                         method name (as string) maintains the object.
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
        self._curdata = {'identify': None,
                         'prepare': None,
                         'enforce': None}
        self._stage = None

    # Interface functions for branch and bound.

    def identify(self, instance):
        """Calls the identify methods of the constraint handlers for
        all unclassified constraints.

        Exact behaviour: The identify methods of the constraint
        handlers are called. The order is given by the identify
        priority order (smaller priority first). All indices of
        unclassified constraints are passed in the sets object.
        Before calling a plugin function, the current data are stored
        in the respective slot.
        TODO maybe change following.
        After every plugin function call, the current data are deleted.

        :param instance: The Instance object.
        :return: A UserInputStatus.
        """
        assert type(instance) is Instance
        # Early stopping if there is nothing to do.
        if instance.nunclassified() == 0:
            return UserInputStatus.OK
        self._stage = 'identify'
        stage = self._stage
        # Iterate over all constraint handlers in the specified order.
        for (_, hdlr) in self._hdlrs_ident:
            # Generate unclassified constraint sets.
            sets = {}
            model = instance.model()
            for constype in hdlr.constypes():
                sets[constype] = instance.unclassified(constype)
            # Set up current data.
            self._curdata[stage] = _CurrentData.create(instance, hdlr)
            # Call identify method.
            self._curdata[stage].handler_instance.identify(sets, model)

            # Handle user input.
            data = self._curdata[stage]
            # TODO more precisely.
            if data.ncuts > 0 or data.ntighten > 0 or data.branched:
                raise Exception('Unallowed interface operation during '
                                'identify process.')
            # Reset current data.
            self._curdata[stage] = None
        if instance.nunclassified() == 0:
            return UserInputStatus.OK
        else:
            raise Exception('After calling all constraint handlers, '
                            'there are still unclassified constraints.')

    def prepare(self, instance):
        """Calls the prepare methods of the constraint handlers for
        all unclassified constraints.

        Exact behaviour: The prepare methods of the constraint
        handlers are called. The order is given by the enforce
        priority order (smaller priority first). All indices of
        constraints belonging to the handler are passed in the sets
        object.
        Before calling a plugin function, the current data are stored
        in the respective slot.
        TODO maybe change following.
        After every plugin function call, the current data are deleted.

        :param instance: The Instance object.
        :return: A UserInputStatus.
        """
        assert type(instance) is Instance
        self._stage = 'prepare'
        stage = self._stage
        # Iterate over all constraint handlers in the specified order.
        for (_, hdlr) in self._hdlrs_enf:
            # Generate constraint handler sets.
            sets = instance.conshandler_sets(hdlr)
            model = instance.model()
            # Set up current data.
            self._curdata[stage] = _CurrentData.create(instance, hdlr)
            # Call prepare method.
            self._curdata[stage].handler_instance.prepare(sets, model)

            # Handle user input.
            data = self._curdata[stage]
            # TODO more precisely.
            # TODO handle other cases, e.g. direct branching or
            # TODO infeasibility.
            if data.branched:
                raise Exception('Unallowed interface operation during '
                                'prepare process.')
            # Reset current data.
            self._curdata[stage] = None
        return UserInputStatus.OK

    def solve_relaxation(self, instance):
        """Calls the relaxation solving method of the instance and
        passes the user defined relaxation solver.

        :param instance: The Instance object.
        :return: A UserInputStatus.
        """
        assert type(instance) is Instance
        instance.solve_relaxation(self._relaxation_solver, self._epsilon)
        return UserInputStatus.OK

    def enforce(self, instance):
        """Calls the enforce methods of the constraint handlers for
        all unclassified constraints.

        Exact behaviour: The prepare methods of the constraint
        handlers that are not patr of the relaxation are called. The
        order is given by the enforce priority order (smaller priority
        first). All indices of constraints belonging to the handler are
        passed in the sets object.
        Before calling a plugin function, the current data are stored
        in the respective slot. If branching happens, infeasibility
        is declared or the constraint handler passes, the current data
        are deleted afterwards. If only cuts are added and bound
        tightening is performed, only the user data are reset and the
        solver data are maintained for the upcoming enforcement call.

        :param instance: The Instance object.
        :return: A UserInputStatus.
        """
        assert type(instance) is Instance
        self._stage = 'enforce'
        stage = self._stage
        # Iterate over all constraint handlers in the specified order.
        for (_, hdlr) in self._hdlrs_enf:
            if not hdlr.relax() and instance.nviolated(hdlr) > 0:
                # Generate constraint handler sets.
                sets = instance.violated_sets(hdlr)
                # Only possible to do inside if resolve after every
                # conshandler that performs changes.
                model = instance.relax_model()
                # Set up current data if not already existing.
                if self._curdata[stage] is None \
                or hdlr.name() != self._curdata[stage].handler.name():
                    self._curdata[stage] = _CurrentData.create(instance, hdlr)
                # Call enforce method.
                self._curdata[stage].handler_instance.enforce(sets, model)

                # Handle user input.
                data = self._curdata[stage]
                if data.branched:
                    self._bnb_tree.register_children_instances(data.children)
                    self._curdata[stage] = None
                    return UserInputStatus.BRANCHED
                elif data.ncuts > 0 or data.ntighten > 0:
                    self._curdata[stage].reset_userdata()
                    return UserInputStatus.RESOLVE
                # Reset current data.
                self._curdata[stage] = None

        raise Exception('enforce failed.')

    # Interface functions for solver module, i.e. user interface.

    def add_conshandler(self, conshandler):
        """Function to register that a conshandler is used.
        :param conshandler: A ConsHandler object.
        """
        heapq.heappush(self._hdlrs_ident,
                       (conshandler.identify_prio(), conshandler))
        heapq.heappush(self._hdlrs_enf,
                       (conshandler.enforce_prio(), conshandler))

    def add_constraints(self, constype, conss, params={}):
        """Function to add constraints to the current instance object.
        See solver module for precise explanation.
        """
        curdata = self._curdata[self._stage]
        curdata.instance.add_constraints(constype, conss, params)
        curdata.add_cuts(len(conss))

    def branch(self, var=None, point=None):
        """Function to perform branching on the current instance object.
        See solver module for precise explanation.
        """
        curdata = self._curdata[self._stage]
        # TODO maybe allow more sophisticated branching expressions.
        curdata._branched = True
        if var is not None and point is not None:
            left = Instance.derive_instance(curdata.instance)
            left.change_varbounds(var, upper_bound=point)
            curdata.add_child(left)
            right = Instance.derive_instance(curdata.instance)
            right.change_varbounds(var, lower_bound=point)
            curdata.add_child(right)

    def change_bound(self, var, lower=None, upper=None):
        """Function to change a variable's bounds on the current
        instance object. See solver module for precise explanation.
        """
        curdata = self._curdata[self._stage]
        curdata.instance.change_varbounds(var, lower_bound=lower,
                                               upper_bound=upper)
        curdata.tighten()

    def match(self, constraint):
        """Function to assign a constraint to the current conshandler.
        See solver module for precise explanation.
        """
        curdata = self._curdata[self._stage]
        curdata.instance.register_conshandler(constraint, curdata.handler)
        curdata.add_match()

    def register_relaxation_solver(self, relaxation_solver):
        """Registers a relaxation solver."""
        self._relaxation_solver = relaxation_solver

    def set_epsilon(self, epsilon):
        """Registers the epsilon for the solving process.
        The epsilon is the tolerance applied to constraint bounds when
        determining whether they are met or not."""
        self._epsilon = epsilon

    def solve(self, py_model):
        """Initiates the solving process. Calls the branch and bound
        algorithm.
        :param py_model: Model to solve.
        """
        self._py_model = py_model
        # Create instance.
        instance = Instance.create_instance(py_model.clone())
        # Assign constraint handlers.
        self.identify(instance)
        # Create bnb_tree.
        self._bnb_tree = BranchAndBound.create(self)
        self._bnb_tree.register_root_instance(instance)
        # Start solving process.
        self._bnb_tree.execute()
        print('Done')


class _CurrentData:
    """
    This class maintains two kinds of data.
    Firstly, it maintains the relevant current data of the algorithm
    that the coordinator executes. These data represent the status of
    the solver.
    Secondly, an object of this class saves all relevant data about
    which interface functions the user has called during a single
    plugin function call.

    Use the factory method _CurrentData.create(...) for generating a new
    instance.

    Public class attributes:
      1) Solver status data
        instance    The current instance to work on.
        handler     The current conshandler (ConsHandlerManager).
        handler_obj The current constraint handler object (ConsHandler).
      2) User given data
        nmatched    The number of constraints assigned to a constraint
                        handler by the user.
        ncuts       The number of cuts added by the user.
        ntighten    The number of tighten_bounds calls by the user.
        branched    Flag if the instance is branched by the user.
        children    Children instances created by the user.
    """

    @classmethod
    def create(cls, instance, handler):
        """Factory method to generate a new _Solution object. Parameters
        as described above.
        """
        data = _CurrentData()
        data._instance = instance
        data._handler = handler
        data._handler_obj = handler.generate()
        return data

    def __init__(self):
        """Constructor setting attributes to default value."""
        # Solver status data.
        self._instance = None
        self._handler = None
        self._handler_obj = None
        # User given data.
        self._nmatched = 0
        self._ncuts = 0
        self._ntighten = 0
        self._branched = False
        self._children = []

    def reset_userdata(self):
        """Resets all user given data to default value."""
        self._nmatched = 0
        self._ncuts = 0
        self._ntighten = 0
        self._branched = False
        self._children = []

    @property
    def instance(self):
        return self._instance

    @property
    def handler(self):
        return self._handler

    @property
    def handler_instance(self):
        return self._handler_obj

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

    def branch(self):
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
