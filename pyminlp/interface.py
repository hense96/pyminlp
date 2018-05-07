"""This is the module that registers the solver to Pyomo and
distributes the data given by Pyomo. It should be implemented
according to the implementation given in the Pyomo code basis.

This is still to do. So far, the module only functions as a class that
processes the instance data directly passed from an application file.
"""


# TODO this should be done on a lower level.
import pyminlp.subprob.Instance


class PyMINLP:
    """Dummy implementation of the solver."""

    def __init__(self):
        pass

    def solve(self, pyomo_model):
        # TODO this should be done on a lower level.
        inst = Instance.create_instance(pyomo_model)
        pass
