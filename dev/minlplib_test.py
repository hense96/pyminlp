# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


import numpy as np
from pathlib import Path

from pyminlp.solver import PyMINLP
from pyminlp.plugins.miqcqp import *

import sys
sys.path.append('../osil_parser')
from osilParser import OsilParser
from variable import Variable
from quadraticConstraint import QuadraticConstraint
from objectiveFunction import ObjectiveFunction


def main():
    testset = []

    dir_base = './instances/minlplib_testset/osil/'
    dir_names = ['QP', 'QCQP', 'MIQCQP', 'IQCQP']

    for dir_name in dir_names:
        dir_str = '{}{}'.format(dir_base, dir_name)
        pathlist = Path(dir_str).glob('**/*.osil')
        for path in pathlist:
            testset.append(str(path))

    for filename in testset:
        solve_instance(filename)


def solve_instance(filename):

    print('Read file {}.'.format(filename))

    problem = read_problem(filename)
    is_min = problem.objectiveFunctions[0].maxOrMin == 'min'
    quad_obj = prepare(problem)
    instance = create_pyomo_instance(problem)

    print('Solve instance {}.'.format(filename))

    solver = set_up_solver()
    res = solver.solve(instance)
    #print(res)

    print('Instance {} done.'.format(filename))


def read_problem(filename):
    parser = OsilParser(filename)
    return parser.parseProblem()


def prepare(problem):
    # Turn quadratic objective function into constraint.
    quad_obj = False
    if problem.objectiveFunctions[0].quadratic is not None:
        quad_obj = True
        objvar = Variable('C', -float('inf'), float('inf'), '_objvar')
        problem.variables.append(objvar)
        for cons in problem.constraints:
            if cons.linear is not None:
                cons.linear = np.append(cons.linear, 0.0)

        lin = problem.objectiveFunctions[0].linear
        quad = problem.objectiveFunctions[0].quadratic
        if problem.objectiveFunctions[0].constant is None:
            cons = 0.0
        else:
            cons = problem.objectiveFunctions[0].constant
        if problem.objectiveFunctions[0].maxOrMin != 'min':
            # Make min problem.
            if lin is not None:
                for i in range(len(lin)):
                    if lin[i] != 0.0:
                        lin[i] = - lin[i]
            for i in range(len(quad.data)):
                if quad.data[i] != 0.0:
                    quad.data[i] = - quad.data[i]
        if lin is not None:
            lin.append(-1.0)
        objcons = QuadraticConstraint('_objcons', -float('inf'), -cons,
                                      lin, quad)
        problem.constraints.append(objcons)

        obj_lin = np.zeros(len(lin))
        obj_lin[len(obj_lin) - 1] = 1.0
        obj = ObjectiveFunction(
            problem.objectiveFunctions[0].weight, '_obj', 'min', obj_lin)
        problem.objectiveFunctions[0] = obj
    # Set variable bounds to -1,000,000 and 1,000,000 if unknown.
    # Do not choose instances with to large variable values.
    bound_count = 0
    for var in problem.variables:
        if np.isinf(var.lb):
            var.lb = -1000000
            bound_count += 1
        if np.isinf(var.ub):
            var.ub = 1000000
            bound_count += 1
    print('Randomly set variable bounds: {}'.format(bound_count))
    return quad_obj


def create_pyomo_instance(problem):
    # Create an abstract model for a QCQP with linear objective
    model = ConcreteModel()

    # Sets
    model.C = Set(initialize=_create_name_set(problem.constraints))
    model.V = Set(initialize=_create_name_set(problem.variables))

    # Constraint parameters
    model.A = Param(model.C, model.V, model.V, default=0,
                    initialize=_create_A(problem))
    model.b = Param(model.C, model.V, default=0,
                    initialize=_create_b(problem))

    # Constraint bounds
    cl, cu = _create_bounds(problem.constraints)
    model.cl = Param(model.C, initialize=cl)
    model.cu = Param(model.C, initialize=cu)

    # Variable bounds
    xl, xu = _create_bounds(problem.variables)
    model.xl = Param(model.V, initialize=xl)
    model.xu = Param(model.V, initialize=xu)

    # Bounds
    def var_bounds_rule(model, k):
        return model.xl[k], model.xu[k]

    # Variables
    model.X = Var(model.V, bounds=var_bounds_rule, domain=Reals)

    for v in problem.variables:
        if v.varType == 'B':
            model.X[v.name].domain = Binary
        elif v.varType == 'I':
            model.X[v.name].domain = Integers

    # Constraints
    def cons_rule_1(model, i):
        quad = sum(sum(model.A[i, k1, k2] * model.X[k1] * model.X[k2]
                       for k1 in model.V) for k2 in model.V)
        lin = sum(model.b[i, k] * model.X[k] for k in model.V)
        return quad + lin <= model.cu[i]

    model.Quadcons1 = Constraint(model.C, rule=cons_rule_1)

    def cons_rule_2(model, i):
        quad = sum(sum(model.A[i, k1, k2] * model.X[k1] * model.X[k2]
                       for k1 in model.V) for k2 in model.V)
        lin = sum(model.b[i, k] * model.X[k] for k in model.V)
        return (-1) * (quad + lin) <= (-1) * model.cl[i]

    model.Quadcons2 = Constraint(model.C, rule=cons_rule_2)

    # Handle objective function.
    obj_min = problem.objectiveFunctions[0].maxOrMin == 'min'
    obj_lin = problem.objectiveFunctions[0].quadratic is None

    if not obj_lin:
        raise('Found quadratic objective function in parsed problem.')

    # Objective function parameters
    model.z = Param(model.V, default=0,
                    initialize=_create_z(problem, obj_min))

    # Objective
    def obj_rule(model):
        return sum(model.z[k] * model.X[k] for k in model.V)

    model.Obj = Objective(rule=obj_rule, sense=minimize)

    return model


def set_up_solver():
    solver = PyMINLP()

    solver.use_constraint_handler(name='MIQCQPLinear',
                                  constypes=['Quadcons1', 'Quadcons2', 'Cut'],
                                  identify_prio=1,
                                  enforce_prio=1,
                                  relaxation=True)
    solver.use_constraint_handler(name='MIQCQPConvex',
                                  constypes=['Quadcons1', 'Quadcons2'],
                                  identify_prio=2,
                                  enforce_prio=2,
                                  relaxation=False)
    solver.use_constraint_handler(name='MIQCQPQuadratic',
                                  constypes=['Quadcons1', 'Quadcons2'],
                                  identify_prio=3,
                                  enforce_prio=3,
                                  relaxation=False)

    relax_solver = SolverFactory('cbc')
    solver.set_relaxation_solver(relax_solver)
    solver.set_epsilon(gap_epsilon=0.00001, constraint_epsilon=0.00001)
    solver.set_verbosity(2)
    solver.set_time_limit(15)

    reset_varlist()

    return solver


def _create_name_set(list):
    s = []
    for obj in list:
        s.append(obj.name)
    return s


def _create_A(problem):
    A = {}
    for c in range(len(problem.constraints)):
        cons = problem.constraints[c]
        if cons.quadratic is None:
            continue
        for i in range(len(cons.quadratic.data)):
            v1 = problem.variables[cons.quadratic.row[i]].name
            v2 = problem.variables[cons.quadratic.col[i]].name
            corr_factor = 1.0
            if cons.quadratic.row[i] != cons.quadratic.col[i]:
                corr_factor = 0.5
            A[cons.name, v1, v2] = corr_factor * cons.quadratic.data[i]
    return A


def _create_b(problem):
    b = {}
    for c in range(len(problem.constraints)):
        cons = problem.constraints[c]
        if cons.linear is None:
            continue
        for v in range(len(problem.variables)):
            var = problem.variables[v]
            if problem.constraints[c].linear[v] != 0.0:
                b[cons.name, var.name] = problem.constraints[c].linear[v]
    return b


def _create_c(problem):
    return {}


def _create_bounds(list):
    l = {}
    u = {}
    for obj in list:
        l[obj.name] = obj.lb
        u[obj.name] = obj.ub
    return l, u


def _create_z(problem, is_min):
    z = {}
    coef = problem.objectiveFunctions[0].linear
    for v in range(len(problem.variables)):
        if coef[v] == 0.0:
            continue
        var = problem.variables[v]
        if is_min:
            z[var.name] = coef[v]
        else:
            z[var.name] = - coef[v]
    return z
