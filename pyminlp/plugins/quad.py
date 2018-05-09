
import numpy as np

from pyomo.environ import *

from pyminlp.conshdlr import ConsHandler
from pyminlp.solver import *


class LinearHandler(ConsHandler):


    def name(cls):
        return 'linear'

    def identify(self, sets, model):
        # Iterate over all constraint representation types.
        for constype in sets:
            set = sets[constype]

            # Cuts are always linear.
            if constype == 'Cuts':
                for index in set:
                    cons = model.Cuts[set[index]]
                    self.solver.match(cons.name)

            # For each quadratic constraint, check if the quadratic part
            # is zero.
            if constype == 'Quadcons1' or constype == 'Quadcons2':
                for index in set:
                    cons = model.component(constype)[index]
                    A = _get_numpy_rep(model, model.A, index, 2)
                    if _is_zero(A):
                        self.solver.match(cons.name)


class QuadConvHandler(ConsHandler):


    def name(cls):
        return 'quadconv'

    def identify(self, sets, model):
        # Iterate over all constraint representation types.
        for constype in sets:
            set = sets[constype]

            # For each constraint, check if A matrix is positive
            # semi-definite.
            for index in set:
                cons = model.component(constype)[index]
                A = _get_numpy_rep(model, model.A, index, 2)
                if constype == 'Quadcons1' and _is_psd(A) \
                    or constype == 'Quadcons2' and _is_psd(-A):
                    self.solver.match(cons.name)

    def separate(self, sets, model, add_model):
        # Iterate over all constraint representation types.
        for constype in sets:
            set = sets[constype]

            # For each violated constraint, compute a cutting plane.
            for index in set:
                cons = model.component(constype)[index]
                A = _get_numpy_rep(model, model.A, index, 2)
                # ...
                # compute a vector 'a' for linear constraint for each
                # cutting plane
                a = {}
                # ...

            # Generate new parameters and add constraints using
            # the add_model object and the interface function.
            add_model.a_new = Param(set, model.V, initialize=a, default=0)
            add_model.Cuts_new = Constraint(set, rule=linear_rule)
            self.solver.add_constraints(constype='Cuts',
                                        constraints=add_model.Cuts_new,
                                        params={'a':add_model.a_new})


class QuadNoncHandler(ConsHandler):


    def name(cls):
        return 'quadnonc'

    def identify(self, sets, model):
        # Match any constraint.
        for constype in sets:
            set = sets[constype]
            for index in set:
                cons = model.component(constype)[index]
                self.solver.match(cons.name)

    def branch(self, sets, model):
        # Do computations to find branching variable and point.
        left_child, right_child = self.solver.branch('X[x1]', 1.0)
        print('Branching done')


# Miscellaneous functions.
def _get_numpy_rep(model, param, index, dim):
    if dim == 0:
        res = param[index]
    elif dim == 1:
        res = np.zeros(model.nvariables())
        i = 0
        for v in model.X:
            res[i] = param[index, v]
            i += 1
    elif dim == 2:
        res = np.zeros((model.nvariables(), model.nvariables()))
        i = 0
        for v1 in model.X:
            j = 0
            for v2 in model.X:
                res[i, j] = param[index, v1, v2]
                j += 1
            i += 1
    return res

def _is_zero(A):
    return np.count_nonzero(A) == 0

def _is_psd(A):
    return np.all(np.linalg.eigvals(A) > 0)

def linear_rule(model, i):
    #return sum(model.b[k] * model.X[k] for k in model.V)
    # Dummy version.
    return model.X['x1'] <= model.X['x2']
