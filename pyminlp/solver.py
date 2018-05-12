# This is a dummy file for the interface. It provides a hopefully
# complete list of the needed interface functions for problem
# specification, option setting and plugins. The functions may be
# moved to a different location later.


import heapq

from pyminlp.plugins.quad import *
from pyminlp.subprob import Instance


class PyMINLP:

    def __init__(self):
        # TODO implement this as a plugin.
        self._known_hdlrs = []
        self._known_hdlrs.append(LinearHandler())
        self._known_hdlrs.append(QuadConvHandler())
        self._known_hdlrs.append(QuadNoncHandler())
        for hdlr in self._known_hdlrs:
            hdlr.solver = self
        self._used_hdlrs = []
        self._cur_instance = None
        self._cur_handler = None

    # Set up
    def use_constraint_handler(self, name, types, prio, relax=False):
        for hdlr in self._known_hdlrs:
            if hdlr.name() == name:
                hdlr.set_relax(relax)
                hdlr.add_constypes(types)
                hdlr.set_prio(prio)
                heapq.heappush(self._used_hdlrs, (hdlr.prio(), hdlr))
                break

    # Settings
    def set_relaxation_solver(self, name):
        # Maybe return relaxation solver for settings
        pass

    def set_epsilon(self, epsilon):
        pass

    # Solve
    def solve(self, py_model):
        """This is a dummy function that is called by the interface to
        simulate all functionality the solver already has."""
        self._cur_instance = Instance.create_instance(py_model)
        instance = self._cur_instance
        # TODO first breakpoint, show internal instance
        # Call identify function of all used constraint handlers.
        for (_, hdlr) in self._used_hdlrs:
            set = {}
            model = instance.model()
            for reptype in hdlr.constypes():
                set[reptype] = instance.unclassified(reptype)
            self._cur_handler = hdlr
            hdlr.identify(set, model)
        # TODO second breakpont, show internal instance
        # Rest is called by example interface. But only use solver as
        # interface!

    def register_instance(self, instance=None, constraint_handler=None):
        if instance is not None:
            self._cur_instance = instance
        if constraint_handler is not None:
            self._cur_handler = constraint_handler


    # Plugin function library
    def add_constraints(self, constype, conss, params={}):
        """Note that the index set of the multiple conss may be accessed
        through the conss object
        :param name: string or direct reference to indicate
                         model.Quadcons
        :param conss: the constraint object, may contain multiple conss
        :param params: dictionary of parameters, key is reference to
                       model.A for example and parameters itself content
        """
        self._cur_instance.add_constraints(constype, conss, params)
        for (_, hdlr) in self._used_hdlrs:
            set = {}
            model = self._cur_instance.model()
            for reptype in hdlr.constypes():
                set[reptype] = self._cur_instance.unclassified(reptype)
            self._cur_handler = hdlr
            hdlr.identify(set, model)


    def branch(self, var, point):
        # branching_expr is either some sort of constraint expression or
        # branching point. Concrete representation may be chosen later.
        # This is concrete enough for the moment.
        # The children will have to be passed somewhere.
        inst = self._cur_instance
        left = Instance.derive_instance(inst)
        self._cur_instance = left
        self.change_bound(var, upper=point)
        right = Instance.derive_instance(inst)
        self._cur_instance = right
        self.change_bound(var, lower=point)
        self._cur_instance = inst
        return left, right

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
