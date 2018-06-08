# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________

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


_varlist = None


class MIQCQPLinear(SingletonPlugin):
    implements(IPyMINLPConsHandler)
    """
    Handler for all linear constraints, i.e.:
        - Quadcons1/Quadcons2 without quadratic part
        - Cut.
    """

    # Plugin methods.

    def identify(self, sets, model, solver):
        # Set varlist to clarify order of variable access.
        # TODO find a better solution for that.
        global _varlist
        if _varlist is None:
            _varlist = []
            for v in model.X:
                _varlist.append(v)

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
        self.tighten(sets, model, solver)

    def enforce(self, sets, model, solver):
        pass

    # Own methods.

    def tighten(self, sets, model, solver):
        """A simple bound tightening method trying to derive tighter
        bounds for every variable based on an individual consideration
        of the linear constraints.
        """
        eps = 0.0001
        xl, xu = _get_bounds(model)

        for constype in sets.keys():
            for index in sets[constype]:

                # Read parameters.
                if constype == 'Quadcons1':
                    b = _get_numpy_rep(model, model.b, 1, index)
                    cu = model.cu[index]
                elif constype == 'Quadcons2':
                    b = - _get_numpy_rep(model, model.b, 1, index)
                    cu = - model.cl[index]
                elif constype == 'Cut':
                    b = _get_numpy_rep(model, model.an, 1, index)
                    cu = value(model.cn[index])

                n = len(xl)
                xi = 0
                # Iterate over all variables.
                for x in _varlist:
                    if b[xi] == 0.0:
                        xi += 1
                        continue
                    elif b[xi] > 0.0:
                        comp_lb = False
                    else:
                        comp_lb = True

                    sum = 0
                    # Compute lowest possible body value considering
                    # all other variables.
                    for other_xi in range(n):
                        if other_xi != xi:
                            if b[other_xi] > 0.0:
                                sum += b[other_xi] * xl[other_xi]
                            elif b[other_xi] < 0.0:
                                sum += b[other_xi] * xu[other_xi]

                    # Check if the new bounds are tighter.
                    if comp_lb:
                        xl_new = (cu - sum) / b[xi]
                        if xl_new > xl[xi] + eps:
                            solver.change_bounds(model.X[x], lower=xl_new)
                            xl[xi] = xl_new
                    else:
                        xu_new = (cu - sum) / b[xi]
                        if xu_new < xu[xi] - eps:
                            solver.change_bounds(model.X[x], upper=xu_new)
                            xu[xi] = xu_new

                    xi += 1

        # Check if infeasibility has been derived.
        for v in _varlist:
            if model.X[v].lb > model.X[v].ub + eps:
                solver.declare_infeasible()
                break


class MIQCQPConvex(SingletonPlugin):
    implements(IPyMINLPConsHandler)
    """Handler for all convex quadratic constraints."""

    # Plugin functions.

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
        self.separate(sets, model, solver)

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
                    c = - model.cu[index]
                elif constype == 'Quadcons2':
                    A = - _get_numpy_rep(model, model.A, 2, index)
                    A = _get_symmetric_part(A)
                    b = - _get_numpy_rep(model, model.b, 1, index)
                    c = model.cl[index]

                # Compute cutting plane parameters.
                an = b + np.multiply(2.0, np.matmul(np.transpose(x), A))
                cn = (c - np.matmul(np.transpose(x), np.matmul(A, x)))

                an = _get_pyomo_rep(an, 1, _varlist)

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


class MIQCQPQuadratic(SingletonPlugin):
    implements(IPyMINLPConsHandler)
    """Handler for generic quadratic constraints."""

    _enf_node = None

    # Plugin methods.

    def identify(self, sets, model, solver):
        """All leftover constraints are covered by this handler."""
        # Match any constraint.
        for constype in sets:
            set = sets[constype]
            for index in set:
                cons = model.component(constype)[index]
                solver.match(cons.name)

    def prepare(self, sets, model, solver):
        pass

    def enforce(self, sets, model, solver):
        """Add McCormick underestimators, try to solve again,
        then branch."""
        count = self.mccormick(sets, model, solver)
        if count == 0 or model.name == self._enf_node:
            self.branch(sets, model, solver)
        else:
            self._enf_node = model.name

    # Own functions.

    def mccormick(self, sets, model, solver):
        """Create McCormick underestimators."""
        count = 0
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
                    b = _get_numpy_rep(model, model.b, 1, index)
                    c = - model.cu[index]
                elif constype == 'Quadcons2':
                    A = _get_numpy_rep(model, model.A, 2, index)
                    b = - _get_numpy_rep(model, model.b, 1, index)
                    c = model.cl[index]

                # Do computation.
                bn, cn = self._compute_mccormick_params(A, b, c, x, xl, xu)

                # Catch case all parameters are zero.
                if np.all(bn) == 0.0:
                    continue

                bn = _get_pyomo_rep(bn, 1, _varlist)
                cn = -cn

                # Add constraint.
                model.a_new = Param(model.V, initialize=bn)
                model.c_new = Param(initialize=cn)

                model.Cut_new = Constraint(rule=linear_rule)

                # Use interface function to add constraint.
                solver.add_constraints(constype='Cut',
                                       constraints=model.Cut_new,
                                       params={'an': model.a_new,
                                               'cn': model.c_new})

                count += 1

                model.del_component('a_new')
                model.del_component('c_new')
                model.del_component('Cut_new')

        return count

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
                for v in _varlist:
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
        for v in _varlist:
            if index is not None:
                res[i] = value(param[index, v])
            else:
                res[i] = value(param[v])
            i += 1
    elif dim == 2:
        res = np.zeros((model.nvariables(), model.nvariables()))
        i = 0
        for v1 in _varlist:
            j = 0
            for v2 in _varlist:
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
    for v in _varlist:
        lb[i] = model.X[v].lb
        ub[i] = model.X[v].ub
        i = i + 1
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
    for v in _varlist:
        if model.A[consindex, varindex, v] != 0 \
            or model.A[consindex, v, varindex] != 0:
            return True
    return False


def linear_rule(model):
    body = sum(model.a_new[k] * model.X[k] for k in model.V) + model.c_new
    return body <= 0.0


def reset_varlist():
    global _varlist
    _varlist = None
