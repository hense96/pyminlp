# This is a dummy file for the interface. It provides a hopefully
# complete list of the needed interface functions for problem
# specification, option setting and plugins. The functions may be
# moved to a different location later.


from pyminlp.plugins.quad import *


class PyMINLP:

    def __init__(self):
        # TODO make a plugin out of it
        self._known_hdlrs = []
        self._known_hdlrs.append(QuadConvHandler())
        self._known_hdlrs.append(QuadNoncHandler())
        self._used_hdlrs = []

    # Set up
    def use_constraint_handler(self, name, types, relax=False):
        for hdlr in self._known_hdlrs:
            if hdlr.name() == name:
                hdlr.set_relax(relax)
                hdlr.set_types(types)
                self._used_hdlrs.append(hdlr)
                break

    # Settings
    def set_relaxation_solver(self, name):
        # Maybe return relaxation solver for settings
        pass

    def set_epsilon(self, epsilon):
        pass

    # Plugin function library
    def add_constraints(self, constype, conss, params=None):
        """Note that the index set of the multiple conss may be accessed
        through the conss object
        :param name: string or direct reference to indicate
                         model.Quadcons
        :param conss: the constraint object, may contain multiple conss
        :param params: dictionary of parameters, key is reference to
                       model.A for example and parameters itself content
        """
        pass


    def branch(self, branching_expr):
        # branching_expr is either some sort of constraint expression or
        # branching point. Concrete representation may be chosen later.
        # This is concrete enough for the moment.
        # The children will have to be passed somewhere.
        pass

    def change_bound(self, var, lower, upper):
        # Offer this function either for a concrete variable or for a
        # whole bounds object. This has to be decided at a later point
        # in time.
        pass

    def match(self, cons):
        # Registers that a cons belongs to a constraint handler.
        pass
