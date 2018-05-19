# This module coordinates all user related operations.



import heapq

from pyminlp.subprob import Instance
from pyminlp.bnb import BranchAndBound
from pyminlp.conshdlr import ConsHandler


class Coordinator:
    """
    This class coordinates all user related operations.
    It offers interface functions for the solving process, calls and
    coordinates the user plugins and processes the data that the user
    passes via the solver interface.

    Use the factory method _Coordinator.create(...) for generating a
    new instance.

    Private class attributes:
      1.: User data
        _py_model   The user given pyomo model (ConcreteObject) to
                        solve.
        _relaxation_solver
                    The user given relaxation solver object.
        _conshandlers
                    The priorised heapq of used constraint handlers.
                    Elements on this list have the format:
                        (prio, conshandler) with type
                        (int,  ConsHandler)
        _options    The options object.
      2.: References
        _bnb_tree   The branch and bound algorithm object
                        (BranchAndBound object)
      3.: Current data
        TODO encapsulate
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
        self._conshandlers = []
        self._options = None
        # References.
        self._bnb_tree = None
        # Current data.
        self._cur_instance = None
        self._cur_handler = None
        self._cuts_added = None
        self._branched = None
        self._children = None

    # Interface functions for branch and bound.

    def add_underestimators(self, instance):
        assert type(instance) is Instance
        # TODO maybe implement.
        self._reset_curdata()

    def enforce(self, instance):
        """Calls the enforce methods of the constraint handlers and
        handles the user input, i.e. branching decisions and cutting
        plane adding.
        :param instance: The Instance object to perform enforcement on.
        """
        assert type(instance) is Instance
        # TODO unbounded case.
        self._cur_instance = instance
        model = instance.relax_model()
        # Get all violated constraints.
        nviolated = {}
        for (_, conshdlr) in self._conshandlers:
            nviolated[conshdlr.name()] = instance.nviolated(conshdlr)
        # Call the enforce methods of the relevant constraint handlers.
        for (_, conshdlr) in self._conshandlers:
            if not conshdlr.relax() and nviolated[conshdlr.name()] > 0:
                # 1) Call user plugins.
                self._cur_handler = conshdlr
                cons_sets = instance.violated_sets(conshdlr)
                conshdlr.enforce(cons_sets, model.clone())
                # 2) Handle user input.
                if self._branched:
                    self._bnb_tree.branching(self._children)
                    break
        # If there are new constraints, classify them.
        # TODO what happens if branching and cutting? Also cut
        # TODO classification on children? Maybe, but does not look
        # TODO clean.
        if self._cuts_added > 0:
            self.identify(self._cur_instance)
        self._reset_curdata()

    def identify(self, instance):
        """Calls the identify methods of the constraint handlers for
        all unclassified constraints.
        :param instance: The Instance object.
        """
        assert type(instance) is Instance
        self._cur_instance = instance
        for (_, hdlr) in self._conshandlers:
            self._cur_handler = hdlr
            sets = {}
            model = self._cur_instance.model()
            for constype in hdlr.constypes():
                sets[constype] = self._cur_instance.unclassified(constype)
            self._cur_handler = hdlr
            hdlr.identify(sets, model)
        self._reset_curdata()

    def solve_relaxation(self, instance):
        """Calls the relaxation solving method of the instance and
        passes the user defined relaxation solver."""
        assert type(instance) is Instance
        self._cur_instance = instance
        instance.solve_relaxation(self._relaxation_solver)
        self._bnb_tree.relaxation_solved()
        self._reset_curdata()

    # Miscellaneous functions.

    def _reset_curdata(self):
        # TODO encapsulate
        self._cur_instance = None
        self._cur_handler = None
        self._cuts_added = 0
        self._branched = False
        self._children = []

    # Interface functions for solver module, i.e. user interface.

    def add_conshandler(self, conshandler):
        """Function to register that a conshandler is used.
        :param conshandler: A ConsHandler object.
        """
        assert type(conshandler) is ConsHandler
        heapq.heappush(self._conshandlers, (conshandler.prio(), conshandler))

    def add_constraints(self, constype, conss, params={}):
        """Function to add constraints to the current instance object.
        See solver module for precise explanation.
        """
        self._cur_instance.add_constraints(constype, conss, params)
        self._cuts_added += len(conss)

    def branch(self, var=None, point=None):
        """Function to perform branching on the current instance object.
        See solver module for precise explanation.
        """
        # TODO maybe allow more sophisticated branching expressions.
        self._branched = True
        if var is not None and point is not None:
            left = Instance.derive_instance(self._cur_instance)
            left.change_varbounds(var, upper_bound=point)
            right = Instance.derive_instance(self._cur_instance)
            right.change_varbounds(var, lower_bound=point)
            self._children = [left, right]

    def change_bound(self, var, lower=None, upper=None):
        """Function to change a variable's bounds on the current
        instance object. See solver module for precise explanation.
        """
        self._cur_instance.change_varbounds(var, lower_bound=lower,
                                                 upper_bound=upper)

    def match(self, constraint):
        """Function to assign a constraint to the current conshandler.
        See solver module for precise explanation.
        """
        self._cur_instance.register_conshandler(constraint, self._cur_handler)

    def register_relaxation_solver(self, relaxation_solver):
        """Registers a relaxation solver."""
        self._relaxation_solver = relaxation_solver

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
        self._bnb_tree.register_instance(instance)
        # Start solving process.
        self._bnb_tree.execute()
        print('Done')
