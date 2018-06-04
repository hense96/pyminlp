# This is a plugin for solving generic QCP.
# It distinguishes between linear constraints, convex quadratic
# and nonconvex quadratic constraints.
#
# The underlying QCP formulation is given in pyminlp/dev/qcp.py.


import numpy as np
import random

from pyomo.environ import *

from pyutilib.component.core import *

from pyminlp.conshdlr import IPyMINLPConsHandler


class LinearHandler(SingletonPlugin):
    implements(IPyMINLPConsHandler)
    """Handler for all linear constraints."""

    @classmethod
    def create(cls):
        return LinearHandler()

    @classmethod
    def name(cls):
        return 'linear'

    def identify(self, sets, model, solver):
        # Iterate over all constraint representation types.
        for constype in sets:
            set = sets[constype]

            # Cuts are always linear.
            if constype == 'Cut':
                for index in set:
                    cons = model.Cut[index]
                    solver.match(cons.name)

            # For each quadratic constraint, check if the quadratic part
            # is zero.
            if constype == 'Quadcons1' or constype == 'Quadcons2':
                for index in set:
                    cons = model.component(constype)[index]
                    A = _get_numpy_rep(model, model.A, 2, index)
                    if _is_zero(A):
                        solver.match(cons.name)

    def prepare(self, sets, model, solver):
        pass

    def enforce(self, sets, model, solver):
        pass


class QuadConvHandler(SingletonPlugin):
    implements(IPyMINLPConsHandler)
    """Handler for all convex quadratic constraints."""

    @classmethod
    def create(cls):
        return QuadConvHandler()

    @classmethod
    def name(cls):
        return 'quadconv'

    def identify(self, sets, model, solver):
        """Identifies convexity using the notion of positive
        semidefiniteness."""
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
                    solver.match(cons.name)

    def prepare(self, sets, model, solver):
        pass

    def enforce(self, sets, model, solver):
        """Firstly calls a separation method that adds cuts,
        then a tighten method that tightens bounds."""
        self.separate(sets, model, solver)
        #self.tighten(model, solver)

    # Own functions.

    def separate(self, sets, model, solver):
        """Separation method for quadratic constraints using a linear
        underestimation of the constraints at the solution point."""
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
                solver.add_constraints(constype='Cut',
                                       constraints=model.Cut_new,
                                       params={'an': model.a_new,
                                               'cn': model.c_new})

                model.del_component('a_new')
                model.del_component('c_new')
                model.del_component('Cut_new')

    def tighten(self, model, solver):
        """This is dummy function to test the tighten_bounds interface
        function. It only works for the minlplib_st_e30 instance, where
        the actual varbounds of X[x11], X[x12], X[x13] are 0 and 3."""
        index_set = ['x11', 'x12', 'x13']
        for index in index_set:
            lb = model.X[index].lb
            if lb <= -1:
                solver.change_bounds(model.X[index].name, lower=(lb + 1))
            elif lb < 0:
                solver.change_bounds(model.X[index].name, lower=0)
            else:
                ub = model.X[index].ub
                if ub >= 4:
                    solver.change_bounds(model.X[index].name,
                                             lower=(ub - 1))
                elif ub > 3:
                    solver.change_bounds(model.X[index].name, lower=3)


class QuadNoncHandler(SingletonPlugin):
    implements(IPyMINLPConsHandler)
    """Handler for all non-convex quadratic constraints."""

    def __init__(self):
        self._count_enforce = 0

    @classmethod
    def create(cls):
        return QuadNoncHandler()

    @classmethod
    def name(cls):
        return 'quadnonc'

    def identify(self, sets, model, solver):
        """All leftover constraints are considered nonconvex."""
        # Match any constraint.
        for constype in sets:
            set = sets[constype]
            for index in set:
                cons = model.component(constype)[index]
                solver.match(cons.name)

    def prepare(self, sets, model, solver):
        """Call the tighten method."""
        #self.tighten(model, solver)

    def enforce(self, sets, model, solver):
        """Add McCormick underestimators, try to solve again,
        then branch."""
        self.mccormick(sets, model, solver)
        if self._count_enforce > -1:
            self.branch(sets, model, solver)
        self._count_enforce += 1

    def branch(self, sets, model, solver):
        """Branching strategy. If no branching variable found,
        declare infeasibility."""
        # Do computations to find branching variable and point.
        linvar = []
        quadvar = []
        for constype in sets.keys():
            index_set = sets[constype]

            # For each violated constraint, compute a cutting plane.
            for index in index_set:
                for v in model.X:
                    if not _is_fixed(model.X[v]):
                        linvar.append(v)
                        if _appears_in_A(model, v, model.A, index):
                            quadvar.append(v)
                            break

        if len(quadvar) > 0:
            var = random.choice(quadvar)
        elif len(linvar) > 0:
            var = random.choice(linvar)
        else:
            solver.declare_infeasible()
            return

        xb = value(model.X[var])
        xl = model.X[var].lb
        xu = model.X[var].ub
        thresh = 0.8

        if xl is not None and xu is not None:
            if (xb - xl) / (xu - xl) <= thresh:
                xb = xl + thresh * (xu - xl)
            elif (xu - xb) / (xu - xl) <= thresh:
                xb = xl + (1.0 - thresh) * (xu - xl)

        solver.branch(model.X[var].name, xb)

    def mccormick(self, sets, model, solver):
        """Create McCormick underestimators."""
        # Iterate over all constraint representation types.
        for constype in sets.keys():
            index_set = sets[constype]

            # For each violated constraint, compute a cutting plane.
            for index in index_set:

                # Read relevant parameters.
                x = _get_numpy_rep(model, model.X, 1)
                xl, xu = _get_bounds(model)
                if constype == 'Quadcons1':
                    A = _get_numpy_rep(model, model.A, 2, index)
                    A = _get_symmetric_part(A)
                    b = _get_numpy_rep(model, model.b, 1, index)
                    c = model.c[index] - model.cu[index]
                elif constype == 'Quadcons2':
                    A = _get_numpy_rep(model, model.A, 2, index)
                    A = - _get_symmetric_part(A)
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
                solver.add_constraints(constype='Cut',
                                            constraints=model.Cut_new,
                                            params={'an': model.a_new,
                                                    'cn': model.c_new})

                model.del_component('a_new')
                model.del_component('c_new')
                model.del_component('Cut_new')

    def tighten(self, model, solver):
        """This is dummy function to test the tighten_bounds interface
        function. It only works for the minlplib_st_e30 instance, where
        the actual varbounds of X[x11], X[x12], X[x13] are 0 and 3."""
        index_set = ['x11', 'x12', 'x13']
        for index in index_set:
            lb = model.X[index].lb
            if lb <= -1:
                solver.change_bounds(model.X[index].name, lower=(lb + 1))
            elif lb < 0:
                solver.change_bounds(model.X[index].name, lower=0)
            else:
                ub = model.X[index].ub
                if ub >= 4:
                    solver.change_bounds(model.X[index].name,
                                         lower=(ub - 1))
                elif ub > 3:
                    solver.change_bounds(model.X[index].name, lower=3)

    def _compute_mccormick_params(self, A, b, c, x, xl, xu):
        """Parameter computation for McCormick underestimators."""
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
