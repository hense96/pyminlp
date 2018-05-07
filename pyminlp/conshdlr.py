


class ConsHandler:
    """This class is an interface for consraint handlers for different
    constraint types.
    TODO make this a singleton
    """

    def __init__(self):
        # Flag indicating whether constraints of this type should be
        # passed to the relaxation solver.
        self._relax = False
        self._types = []

    def get_relax(self):
        return self._relax

    def set_relax(self, relax):
        self._relax = relax

    def get_reptypes(self):
        return self._types

    def set_reptypes(self, reptypes):
        self._types.extend(reptypes)


    def name(cls):
        """Returns a string as a unique name for the constraint handler.
        """
        raise NotImplementedError('Please assign a name to this constraint'
                                  'handler!')

    def identify(self, set, model):
        """Checks whether a constraint belongs to this constraint
        handler.
        """
        name = self.name()
        raise NotImplementedError('Identify method of {} handler not '
                                  'implemented!'.format(name))

    def separate(self, set, model):
        """Implements a strategy to add underestimators for the given
        constraints."""
        name = self.name()
        raise NotImplementedError('Separate method of {} handler not '
                                  'implemented!'.format(name))

    def branch(self, set, model):
        """Implements the branching strategy."""
        name = self.name()
        raise NotImplementedError('Branch method of {} handler '
                                  'not implemented!'.format(name))

    def tighten(self, set, model):
        """Implements a strategy for inferring tighter variable bounds
        from the constraints."""
        name = self.name()
        raise NotImplementedError('Tighten method of {} handler not '
                                  'implemented!'.format(name))
