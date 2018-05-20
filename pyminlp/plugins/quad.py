
import numpy as np

from pyomo.environ import *

from pyminlp.conshdlr import ConsHandler


class LinearHandler(ConsHandler):
    """Handler for all linear constraints."""

    @classmethod
    def create(cls):
        return LinearHandler()

    @classmethod
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

    def prepare(self, sets, model):
        pass

    def enforce(self, sets, model):
        pass


class QuadConvHandler(ConsHandler):
    """Handler for all convex quadratic constraints."""

    @classmethod
    def create(cls):
        return QuadConvHandler()

    @classmethod
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
                if (constype == 'Quadcons1' and _is_psd(A)) \
                   or (constype == 'Quadcons2' and _is_psd(-A)):
                    self.solver.match(cons.name)

    def prepare(self, sets, model):
        pass

    def enforce(self, sets, model):
        self.separate(sets, model)

    # Own functions.

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
                    A = _get_symmetric_part(A)
                    b = _get_numpy_rep(model, model.b, 1, index)
                    c = model.c[index] - model.cu[index]
                elif constype == 'Quadcons2':
                    A = - _get_numpy_rep(model, model.A, 2, index)
                    A = _get_symmetric_part(A)
                    b = - _get_numpy_rep(model, model.b, 1, index)
                    c = - (model.c[index] - model.cl[index])

                # Compute cutting plane parameters.
                an = b + np.multiply(2.0, np.matmul(np.transpose(x), A))
                cn = c - np.matmul(np.transpose(x), np.matmul(A, x))

                an = _get_pyomo_rep(an, 1, model.V)

                # Create Pyomo parameters.
                model.a_new = Param(model.V, initialize=an)
                model.c_new = Param(initialize=cn)

                model.Cut_new = Constraint(rule=linear_rule)

                # Use interface function to add constraint.
                self.solver.add_constraints(constype='Cut',
                                            constraints=model.Cut_new,
                                            params={'an': model.a_new,
                                                    'cn': model.c_new})


class QuadNoncHandler(ConsHandler):
    """Handler for all non-convex quadratic constraints."""

    @classmethod
    def create(cls):
        return QuadNoncHandler()

    @classmethod
    def name(cls):
        return 'quadnonc'

    def identify(self, sets, model):
        # Match any constraint.
        for constype in sets:
            set = sets[constype]
            for index in set:
                cons = model.component(constype)[index]
                self.solver.match(cons.name)

    def prepare(self, sets, model):
        pass

    def enforce(self, sets, model):
        self.mccormick(sets, model)
        self.branch(sets, model)

    def branch(self, sets, model):
        # Do computations to find branching variable and point.
        linvar = None
        quadvar = None
        for constype in sets.keys():
            index_set = sets[constype]

            # For each violated constraint, compute a cutting plane.
            for index in index_set:
                for v in model.X:
                    if not _is_fixed(model.X[v]):
                        linvar = v
                        if _appears_in_A(model, v, model.A, index):
                            quadvar = v
                            break

        if quadvar is not None:
            var = quadvar
        elif linvar is not None:
            var = linvar
        else:
            # TODO declare infeasibility.
            pass

        xb = model.X[var]
        xl = model.X[var].lb
        xu = model.X[var].ub
        thresh = 0.8

        if xl is not None and xu is not None:
            if (xb - xl) / (xu - xl) <= thresh:
                xb = xl + thresh * (xu - xl)
            elif (xu - xb) / (xu - xl) <= thresh:
                xb = xl + (1.0 - thresh) * (xu - xl)

        self.solver.branch(model.X[var].name, xb)


    def mccormick(self, sets, model):
        # Iterate over all constraint representation types.
        for constype in sets.keys():
            index_set = sets[constype]

            # For each violated constraint, compute a cutting plane.
            for index in index_set:

                # Read relevant parameters.
                x = _get_numpy_rep(model, model.X, 1)
                xl, xu = _get_bounds(model)
                # TODO read solution and varbound parameters.
                if constype == 'Quadcons1':
                    A = _get_numpy_rep(model, model.A, 2, index)
                    A = _get_symmetric_part(A)
                    b = _get_numpy_rep(model, model.b, 1, index)
                    c = model.c[index] - model.cu[index]
                elif constype == 'Quadcons2':
                    A = - _get_numpy_rep(model, model.A, 2, index)
                    A = _get_symmetric_part(A)
                    b = - _get_numpy_rep(model, model.b, 1, index)
                    c = - (model.c[index] - model.cl[index])

                # Do computation.
                bn, cn = self._compute_mccormick_params(A, b, c, x, xl, xu)

                bn = _get_pyomo_rep(bn, 1, model.V)

                # Add constraint.
                model.a_new = Param(model.V, initialize=bn)
                model.c_new = Param(initialize=cn)

                model.Cut_new = Constraint(rule=linear_rule)

                # Use interface function to add constraint.
                self.solver.add_constraints(constype='Cut',
                                            constraints=model.Cut_new,
                                            params={'an': model.a_new,
                                                    'cn': model.c_new})

    def _compute_mccormick_params(self, A, b, c, x, xl, xu):
        # Initialise values.
        n = len(A)
        A = _get_upper_triangular(A)
        bn = b
        cn = c
        for j in range(n):
            # Sum up underestimators for every quadratic term according
            # to Vigerske.
            if A[j, j] > 0:
                bn[j] += 2.0 * A[j, j] * x[j]
                cn += - x[j] * A[j, j] * x[j]
            elif A[j, j] < 0:
                bn[j] += A[j, j] * (xl[j] + xu[j])
                cn += - A[j, j] * xl[j] * xu[j]
            # Sum up mccormick underestimators for every bilinear term
            # according to Vigerske.
            for k in range(j + 1, n):
                if A[j, k] > 0:
                    if (xu[j] - xl[j]) * x[k] - (xu[k] - xl[k]) * x[j] \
                            <= xu[j] * xu[k] - xl[j] * xl[k]:
                        bn[j] += A[j, k] * xl[k]
                        bn[k] += A[j, k] * xl[j]
                        cn += - A[j, k] * xl[j] * xl[k]
                    else:
                        bn[j] += A[j, k] * xu[k]
                        bn[k] += A[j, k] * xu[j]
                        cn += - A[j, k] * xu[j] * xu[k]
                elif A[j, k] < 0:
                    if (xu[j] - xl[j]) * x[k] - (xu[k] - xl[k]) * x[j] \
                            <= xu[j] * xl[k] - xl[j] * xu[k]:
                        bn[j] += A[j, k] * xl[k]
                        bn[k] += A[j, k] * xu[j]
                        cn += - A[j, k] * xu[j] * xl[k]
                    else:
                        bn[j] += A[j, k] * xu[k]
                        bn[k] += A[j, k] * xl[j]
                        cn += - A[j, k] * xl[j] * xu[k]
        return bn, cn


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


def _get_bounds(model):
    lb = np.zeros(len(model.X))
    ub = np.zeros(len(model.X))
    i = 0
    for v in model.X:
        lb[i] = model.X[v].lb
        ub[i] = model.X[v].ub
    return lb, ub


def _get_pyomo_rep(vector, dim, index_set):
    new_vector = {}
    if dim == 1:
        i = 0
        for v in index_set:
            new_vector[v] = vector[i]
            i += 1
    return new_vector


def _get_symmetric_part(A):
    return 0.5 * (A + np.transpose(A))


def _get_upper_triangular(A):
        A = np.multiply(2.0, _get_symmetric_part(A))
        for i in range(len(A)):
            for j in range(0, i):
                A[i, j] = 0.0
            A[i, i] *= 0.5
        return A


def _is_zero(A):
    return np.count_nonzero(A) == 0


def _is_psd(A):
    eigvals = np.linalg.eigvals(A)
    return np.all(eigvals >= 0) and np.any(eigvals > 0)


def _is_fixed(var):
    if var.lb is None or var.ub is None:
        return False
    else:
        return var.ub - var.lb <= 0.0001


def _appears_in_A(model, varindex, A, consindex):
    for v in model.X:
        if model.A[consindex, varindex, v] != 0 \
            or model.A[consindex, v, varindex] != 0:
            return True
    return False


def linear_rule(model):
    body = sum(model.a_new[k] * model.X[k] for k in model.V) + model.c_new
    return body <= 0
