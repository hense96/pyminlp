
from pyutilib.component.core import *


class IPyMINLPConsHandler(Interface):
    """This is the interface for constraint handlers.

    It leverages the Pyutilib Component Architecture (PCA).
    # TODO add reference to PCA paper.

    For implementing a constraint handler, be aware of the following
    steps.

    1. Import PCA package:

        from pyutilib.component.core import *

    2.: Class declaration:

        class MyConstraintHandler(SingletonPlugin):
            implements(IPyMINLPConsHandler)

    3.: Implement identify, prepare and enforce method.

    4.: Integrate handler into the solving process:

        solver = PyMINLP()
        [...]
        solver.use_constraint_handler(name='MyConstraintHandler', ...)

    """

    def identify(self, sets, model, solver):
        """Checks whether a constraint belongs to this constraint
        handler.

        # TODO precise description.
        """

    def prepare(self, sets, model, solver):
        """Prepares a model before being solved by the relaxation
        solver.

        # TODO precise description.
        """

    def enforce(self, sets, model, solver):
        """Implements a strategy for enforcing violated constraints
        after the relaxation of the model was solved.

        # TODO precise description.
        """


class ConsHandlerManager:
    """This class manages meta-data of constraint handlers.

    Use the factory method ConsHandlerManager.create(...) for
    generating a instance.

    Private class attributes:
        _name        The name of the constraint handler.
        _constypes   The list of constraint types the handler is
                         responsible for.
        _id_prio     The identify priority of the handler.
        _enf_prio    The enforcement priority of the handler.
        _relax       A flag indicating if the constraints of the
                         handler are part of the relaxation.
        _plugin      The constraint handler plugin object. It should
                         have all methods defined in the
                         IPyMINLPConsHandler interface.
    """

    @classmethod
    def create(cls, name, constypes, id_prio, enf_prio, relax, plugin):
        """Factory method to create a _Node object. Provide the
        underlying instance. Provide a parent node, if existing."""
        manager = ConsHandlerManager()
        manager._name = name
        manager._constypes = constypes
        manager._id_prio = id_prio
        manager._enf_prio = enf_prio
        manager._relax = relax
        manager._plugin = plugin
        return manager

    def __init__(self):
        self._name = None
        self._constypes = []
        self._id_prio = None
        self._enf_prio = None
        self._relax = False
        self._plugin = None

    def __lt__(self, other):
        return self.name() < other.name()

    def name(self):
        return self._name

    def relax(self):
        return self._relax

    def constypes(self):
        return self._constypes

    def identify_prio(self):
        return self._id_prio

    def enforce_prio(self):
        return self._enf_prio

    def generate(self):
        """Returns the plugin object of the constraint handler
        containing the callable interface functions.
        :return: A service for the IPyMINLPConsHandler interface.
        """
        return self._plugin
