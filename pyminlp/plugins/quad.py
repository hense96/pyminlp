
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
            if constype == 'Cut':
                for index in set:
                    cons = model.Cut[index]
                    self.solver.match(cons.name)

            # For each quadratic constraint, check if the quadratic part
            # is zero.
            if constype == 'Quadcons1' or constype == 'Quadcons2':
                for index in set:
                    cons = model.component(constype)[index]
                    A = _get_numpy_rep(model, model.A, 2, index)
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
                A = _get_numpy_rep(model, model.A, 2, index)
                if constype == 'Quadcons1' and _is_psd(A) \
                    or constype == 'Quadcons2' and _is_psd(-A):
                    self.solver.match(cons.name)

    def separate(self, sets, model):
        # Iterate over all constraint representation types.
        for constype in sets.keys():
            index_set = sets[constype]

            # For each violated constraint, compute a cutting plane.
            for index in index_set:

                # Read relevant parameters.
                x = _get_numpy_rep(model, model.X, 1)
                if constype == 'Quadcons1':
                    A = _get_numpy_rep(model, model.A, 2, index)
                    b = _get_numpy_rep(model, model.b, 1, index)
                    c = model.c[index] - model.Quadcons1[index].upper
                elif constype == 'Quadcons2':
                    A = - _get_numpy_rep(model, model.A, 2, index)
                    b = - _get_numpy_rep(model, model.b, 1, index)
                    c = - (model.c[index] + model.Quadcons2[index].upper)

                # Compute cutting plane parameters.
                an = b + np.multiply(2.0, np.matmul(np.transpose(x), A))
                cn = c - np.matmul(np.transpose(x), np.matmul(A, x))

                an = _get_pyomo_rep(an, 1, model.V)

                # Create Pyomo parameters.
                model.an = Param(model.V, initialize=an)
                model.cn = Param(initialize=cn)

                model.Cut = Constraint(rule=linear_rule)

                # Use interface function to add constraint.
                self.solver.add_constraints(constype='Cut',
                                            constraints=model.Cut,
                                            params={'an': model.an,
                                                    'cn': model.cn})

    def enforce(self, sets, model):
        self.separate(sets, model)


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
        self.solver.branch('X[x1]', 1.0)

    def enforce(self, sets, model):
        self.branch(sets, model)


# Miscellaneous functions.

def _get_numpy_rep(model, param, dim, index=None):
    if dim == 0:
        if index is not None:
            res = value(param[index])
        else:
            res = value(param)
    elif dim == 1:
        res = np.zeros(model.nvariables())
        i = 0
        for v in model.X:
            if index is not None:
                res[i] = value(param[index, v])
            else:
                res[i] = value(param[v])
            i += 1
    elif dim == 2:
        res = np.zeros((model.nvariables(), model.nvariables()))
        i = 0
        for v1 in model.X:
            j = 0
            for v2 in model.X:
                if index is not None:
                    res[i, j] = value(param[index, v1, v2])
                else:
                    res[i, j] = value(param[v1, v2])
                j += 1
            i += 1
    return res


def _get_pyomo_rep(vector, dim, index_set):
    new_vector = {}
    if dim == 1:
        i = 0
        for v in index_set:
            new_vector[v] = vector[i]
            i += 1
    return new_vector


def _is_zero(A):
    return np.count_nonzero(A) == 0


def _is_psd(A):
    return np.all(np.linalg.eigvals(A) >= 0)


def linear_rule(model):
    body = sum(model.an[k] * model.X[k] for k in model.V) + model.cn
    return body <= 0
