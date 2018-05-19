# This module


import heapq

from pyminlp.subprob import Instance
from pyminlp.bnb import BranchAndBound


class Coordinator:

    @classmethod
    def create(cls, relaxation_solver=None, options=None):
        coord = Coordinator()
        coord._relaxation_solver = relaxation_solver
        coord._options = options
        return coord

    def __init__(self):
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
        # TODO maybe implement.
        self._reset_curdata()

    def enforce(self, instance):
        # TODO unbounded case.
        self._cur_instance = instance
        model = instance.relax_model()
        nviolated = {}
        for (_, conshdlr) in self._conshandlers:
            nviolated[conshdlr.name()] = instance.nviolated(conshdlr)
        for (_, conshdlr) in self._conshandlers:
            if not conshdlr.relax() and nviolated[conshdlr.name()] > 0:
                # Call user plugins.
                self._cur_handler = conshdlr
                cons_sets = instance.violated_sets(conshdlr)
                conshdlr.enforce(cons_sets, model.clone())
                # Handle user input.
                if self._branched:
                    self._bnb_tree.branching(self._children)
                    break
        if self._cuts_added > 0:
            self.identify(self._cur_instance)
        self._reset_curdata()

    def identify(self, instance):
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
        self._cur_instance = instance
        instance.solve_relaxation(self._relaxation_solver)
        self._bnb_tree.relaxation_solved()
        self._reset_curdata()

    # Miscellaneous functions.

    def _reset_curdata(self):
        self._cur_instance = None
        self._cur_handler = None
        self._cuts_added = 0
        self._branched = False
        self._children = []

    # Interface functions for solver module, i.e. user interface.

    def add_conshandler(self, conshandler):
        heapq.heappush(self._conshandlers, (conshandler.prio(), conshandler))

    def add_constraints(self, constype, conss, params={}):
        self._cur_instance.add_constraints(constype, conss, params)
        self._cuts_added += len(conss)

    def branch(self, var=None, point=None):
        # branching_expr is either some sort of constraint expression or
        # branching point. Concrete representation may be chosen later.
        # This is concrete enough for the moment.
        # The children will have to be passed somewhere.
        self._branched = True
        if var is not None and point is not None:
            left = Instance.derive_instance(self._cur_instance)
            left.change_varbounds(var, upper_bound=point)
            right = Instance.derive_instance(self._cur_instance)
            right.change_varbounds(var, lower_bound=point)
            self._children = [left, right]

    def change_bound(self, var, lower=None, upper=None):
        # Var is a name.
        # Offer this function either for a concrete variable or for a
        # whole bounds object. This has to be decided at a later point
        # in time.
        self._cur_instance.change_varbounds(var, lower_bound=lower,
                                                 upper_bound=upper)

    def match(self, cons):
        # Cons is a name.
        # Registers that a cons belongs to a constraint handler.
        self._cur_instance.register_conshandler(cons, self._cur_handler)

    def register_relaxation_solver(self, relaxation_solver):
        self._relaxation_solver = relaxation_solver

    def solve(self, py_model):
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
