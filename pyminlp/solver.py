# This is a dummy file for the interface. It provides a hopefully
# complete list of the needed interface functions for problem
# specification, option setting and plugins. The functions may be
# moved to a different location later.


import heapq

from pyminlp.plugins.quad import *
from pyminlp.hub import Coordinator


class PyMINLP:

    def __init__(self):
        self._coordinator = Coordinator.create()
        # TODO implement this as a plugin.
        self._known_hdlrs = []
        self._known_hdlrs.append(LinearHandler())
        self._known_hdlrs.append(QuadConvHandler())
        self._known_hdlrs.append(QuadNoncHandler())
        for hdlr in self._known_hdlrs:
            hdlr.solver = self
        self._used_hdlrs = []

    # Set up

    def use_constraint_handler(self, name, types, prio, relax=False):
        for hdlr in self._known_hdlrs:
            if hdlr.name() == name:
                hdlr.set_relax(relax)
                hdlr.add_constypes(types)
                hdlr.set_prio(prio)
                self._coordinator.add_conshandler(hdlr)
                break

    # Settings

    def set_relaxation_solver(self, relaxation_solver):
        # TODO maybe change so that only name required.
        self._coordinator.register_relaxation_solver(relaxation_solver)

    def set_epsilon(self, epsilon):
        pass

    # Solve
    def solve(self, py_model):
        self._coordinator.solve(py_model)

    # Plugin function library

    def add_constraints(self, constype, constraints, params={}):
        # TODO do some input format checking (not logically, format).
        self._coordinator.add_constraints(constype, constraints, params)

    def branch(self, var, point):
        # TODO do some input format checking (not logically, format).
        self._coordinator.branch(var, point)

    def change_bound(self, var, lower=None, upper=None):
        # TODO do some input format checking (not logically, format).
        self._coordinator.change_varbounds(var, lower, upper)

    def match(self, cons):
        # TODO do some input format checking (not logically, format).
        self._coordinator.match(cons)
