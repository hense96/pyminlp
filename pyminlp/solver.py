# This module is the interface module for the user.
# It checks the input format and passes the information to the hub.


from pyminlp.conshdlr import ConsHandlerManager
from pyminlp.hub import Coordinator


class PyMINLP:

    def __init__(self):
        self._coordinator = Coordinator.create()
        # TODO implement this as a plugin.
        self._known_hdlrs = []
        self._known_hdlrs.append(ConsHandlerManager.create('linear'))
        self._known_hdlrs.append(ConsHandlerManager.create('quadconv'))
        self._known_hdlrs.append(ConsHandlerManager.create('quadnonc'))
        for hdlr in self._known_hdlrs:
            # TODO find a better way for that.
            hdlr.solver = self
        self._used_hdlrs = []

    # Functions for the set up.

    def set_relaxation_solver(self, relaxation_solver):
        # TODO maybe change so that only name required.
        self._coordinator.register_relaxation_solver(relaxation_solver)

    def set_epsilon(self, gap_epsilon=None, constraint_epsilon=None):
        self._coordinator.set_epsilon(gap_epsilon, constraint_epsilon)

    def set_verbosity(self, verbosity):
        self._coordinator.set_verbosity(verbosity)

    def use_constraint_handler(self, name, types, identify_prio,
                               enforce_prio=None, relax=False):
        for hdlr in self._known_hdlrs:
            if hdlr.name() == name:
                hdlr.set_relax(relax)
                hdlr.add_constypes(types)
                if enforce_prio is None:
                    hdlr.set_prio(identify_prio, identify_prio)
                else:
                    hdlr.set_prio(identify_prio, enforce_prio)
                self._coordinator.add_conshandler(hdlr)
                break

    # Functions for solving.

    def solve(self, py_model):
        self._coordinator.solve(py_model)

    # Functions for implementing plugins.

    def add_constraints(self, constype, constraints, params={}):
        # TODO do some input format checking (not logically, format).
        self._coordinator.add_constraints(constype, constraints, params)

    def branch(self, var, point):
        # TODO do some input format checking (not logically, format).
        self._coordinator.branch(var, point)

    def change_bounds(self, var, lower=None, upper=None):
        # TODO do some input format checking (not logically, format).
        self._coordinator.change_bounds(var, lower, upper)

    def declare_infeasible(self):
        self._coordinator.declare_infeasible()

    def match(self, cons):
        # TODO do some input format checking (not logically, format).
        self._coordinator.match(cons)
