# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


class ConsHandler:
    """This class is an interface for consraint handlers for different
    constraint types.
    """

    def __init__(self):
        self.solver = None

    @classmethod
    def create(cls):
        # TODO make proper plugin structure. Maybe factory pattern.
        """Creates an object of this constraint handler."""
        raise NotImplementedError('Please register a factory method for '
                                  'constraint handler {}!'.format(cls.name()))

    @classmethod
    def name(cls):
        """Returns a string as a unique name for the constraint handler.
        """
        raise NotImplementedError('Please assign a name to this constraint '
                                  'handler!')

    def identify(self, sets, model):
        """Checks whether a constraint belongs to this constraint
        handler.
        """
        name = self.name()
        raise NotImplementedError('identify method of {} handler not '
                                  'implemented!'.format(name))

    def prepare(self, sets, model):
        """Prepares a model before being solved by the relaxation
        solver.
        """
        name = self.name()
        raise NotImplementedError('prepare method of {} handler not '
                                  'implemented!'.format(name))

    def enforce(self, sets, model):
        """Implements a strategy for enforcing violated constraints
        after the relaxation of the model was solved."""
        name = self.name()
        raise NotImplementedError('enforce method of {} handler not '
                                  'implemented!'.format(name))


class ConsHandlerManager:
    """This class is an interface for consraint handlers for different
    constraint types.
    """

    @classmethod
    def create(cls, conshdlr_name):
        # TODO make proper plugin structure. Maybe factory pattern.
        manager = ConsHandlerManager()
        manager._name = conshdlr_name
        if conshdlr_name == LinearHandler.name():
            manager._generator = LinearHandler.create
        elif conshdlr_name == QuadConvHandler.name():
            manager._generator = QuadConvHandler.create
        elif conshdlr_name == QuadNoncHandler.name():
            manager._generator = QuadNoncHandler.create
        if manager._generator is None:
            raise ValueError('Can not find constraint handler '
                             '{}.'.format(conshdlr_name))
        return manager

    def __init__(self):
        self._name = None
        self._generator = None
        self._relax = False
        self._constypes = []
        self._id_prio = None
        self._enf_prio = None
        # TODO find a better way for that.
        self.solver = None

    def __lt__(self, other):
        return self.name() < other.name()

    def name(self):
        return self._name

    def relax(self):
        return self._relax

    def set_relax(self, relax):
        self._relax = relax

    def constypes(self):
        return self._constypes

    def add_constypes(self, constypes):
        self._constypes.extend(constypes)

    def identify_prio(self):
        return self._id_prio

    def enforce_prio(self):
        return self._enf_prio

    def set_prio(self, identify_prio=None, enforce_prio=None):
        if identify_prio is not None:
            self._id_prio = identify_prio
        if enforce_prio is not None:
            self._enf_prio = enforce_prio

    def generate(self):
        obj = self._generator()
        obj.solver = self.solver
        return obj

from pyminlp.plugins.quad import *
