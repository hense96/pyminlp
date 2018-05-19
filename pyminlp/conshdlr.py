


class ConsHandler:
    """This class is an interface for consraint handlers for different
    constraint types.
    TODO make this a singleton
    """

    def __init__(self):
        # Flag indicating whether constraints of this type should be
        # passed to the relaxation solver.
        self._relax = False
        self._constypes = []
        self._prio = None
        self.solver = None

    def __lt__(self, other):
        return self.name() < other.name()

    def relax(self):
        return self._relax

    def set_relax(self, relax):
        self._relax = relax

    def constypes(self):
        return self._constypes

    def add_constypes(self, constypes):
        self._constypes.extend(constypes)

    def prio(self):
        return self._prio

    def set_prio(self, prio):
        self._prio = prio


    def name(cls):
        """Returns a string as a unique name for the constraint handler.
        """
        raise NotImplementedError('Please assign a name to this constraint'
                                  'handler!')

    def identify(self, sets, model):
        """Checks whether a constraint belongs to this constraint
        handler.
        """
        name = self.name()
        raise NotImplementedError('Identify method of {} handler not '
                                  'implemented!'.format(name))

    def separate(self, sets, model):
        """Implements a strategy to add underestimators for the given
        constraints."""
        name = self.name()
        raise NotImplementedError('Separate method of {} handler not '
                                  'implemented!'.format(name))

    def branch(self, sets, model):
        """Implements the branching strategy."""
        name = self.name()
        raise NotImplementedError('Branch method of {} handler '
                                  'not implemented!'.format(name))

    def tighten(self, sets, model):
        """Implements a strategy for inferring tighter variable bounds
        from the constraints."""
        name = self.name()
        raise NotImplementedError('Tighten method of {} handler not '
                                  'implemented!'.format(name))

    def enforce(self, sets, model):
        """Implements a strategy for enforcing violated constraints."""
        name = self.name()
        raise NotImplementedError('Enforce method of {} handler not '
                                  'implemented!'.format(name))
