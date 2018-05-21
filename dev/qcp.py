from pyomo.environ import *
from pyminlp.solver import PyMINLP
from pyminlp.subprob import *
from pyminlp.plugins.quad import *


def foo(filename):

    # Create model instance first.
    model = createInstance(filename)

    # Set up solver.
    solver = PyMINLP()

    solver.use_constraint_handler(name='linear',
                                  types=['Quadcons1', 'Quadcons2', 'Cut'],
                                  identify_prio=1, relax=True)
    solver.use_constraint_handler(name='quadconv',
                                  types=['Quadcons1', 'Quadcons2'],
                                  identify_prio=2, relax=False)
    solver.use_constraint_handler(name='quadnonc',
                                  types=['Quadcons1', 'Quadcons2'],
                                  identify_prio=3, relax=False)

    relax_solver = SolverFactory('glpk')
    solver.set_relaxation_solver(relax_solver)
    solver.set_epsilon(0.0001)

    solver.solve(model)



def plugin_simulation(model, solver):
    # Plugin simulation.
    hdlrs = solver._used_hdlrs
    for (_, hdlr) in hdlrs:
        if hdlr.name() == 'quadconv':
            conv_hdlr = hdlr
        elif hdlr.name() == 'quadnonc':
            nonc_hdlr = hdlr

    # Cutting plane generation
    violated_conss = {'Quadcons1':['e1', 'e2']}
    conv_hdlr.separate(violated_conss, model, model.clone())
    conv_hdlr.separate(violated_conss, model, model.clone())

    # Branching
    nonc_hdlr.branch(violated_conss, model)


    print('Done')


def createInstance( filename ):
    """Find the model specification in this function. It creates an
    instance of this model using the provided data file and returns this
    instance.
    """

    # Abstract model
    model = AbstractModel()

    # Sets
    model.C = Set()
    model.V = Set()

    # Constraint parameters
    model.A = Param(model.C, model.V, model.V, default=0)
    model.b = Param(model.C, model.V, default=0)
    model.c = Param(model.C, default=0)

    # Constraint bounds
    model.cl = Param(model.C)
    model.cu = Param(model.C)

    # Variable bounds
    model.xl = Param(model.V)
    model.xu = Param(model.V)

    # Objective function parameters
    model.z = Param(model.V, default=0)

    # Bounds
    def var_bounds_rule(model, k):
        return model.xl[k], model.xu[k]

    # Variables
    model.X = Var(model.V, bounds=var_bounds_rule, domain=Reals)

    # Objective
    def obj_rule(model):
        return sum(model.z[k] * model.X[k] for k in model.V)
    model.Obj = Objective(rule=obj_rule, sense=minimize)

    # Constraints
    def cons_rule_1(model, i):
        quad = sum(sum(model.A[i, k1, k2] * model.X[k1] * model.X[k2]
                       for k1 in model.V) for k2 in model.V)
        lin = sum(model.b[i, k] * model.X[k] for k in model.V) + model.c[i]
        return quad + lin <= model.cu[i]
    model.Quadcons1 = Constraint(model.C, rule=cons_rule_1)

    def cons_rule_2(model, i):
        quad = sum(sum(model.A[i, k1, k2] * model.X[k1] * model.X[k2]
                       for k1 in model.V) for k2 in model.V)
        lin = sum(model.b[i, k] * model.X[k] for k in model.V) + model.c[i]
        return -(quad + lin) <= -model.cl[i]
    model.Quadcons2 = Constraint(model.C, rule=cons_rule_2)

    instance = model.create_instance(filename)

    return instance
