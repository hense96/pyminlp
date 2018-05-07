from pyomo.environ import *
from pyminlp.solver import PyMINLP
from pyminlp.subprob import *
from pyminlp.plugins.quad import *


def foo(filename):

    # create model instance first
    model = createInstance(filename)

    # display read instance
    #model.pprint()

    # set up solver
    #solver = PyMINLP()
    #solver.use_constraint_handler(name='quadconv', types=['Quadcons'], relax=False)
    #solver.use_constraint_handler(name='quadnonc', types=['Quadcons'], relax=False)

    ch1 = QuadConvHandler()
    ch2 = QuadNoncHandler()

    int_inst = Instance.create_instance(model)

    count = 0
    for ct in model.component_objects(ctype=Constraint):
        for c in ct:
            py_cons = ct[c]
            if count == 0:
                int_inst.register_constype(py_cons, ch1)
                count = 1
            else:
                int_inst.register_constype(py_cons, ch2)
                count = 0

    clone = model.clone()
    new_cons = clone.Quadcons
    par_A = clone.A
    par_b = clone.b
    par_c = clone.c

    clone.k = Param(initialize=3)
    k = clone.k

    xyz = par_A['e1',:,:]

    int_inst.add_constraints('Cons', new_cons,
                             {'cn':par_c, 'An':par_A, 'bn':par_b, 'kn':k})

    int_inst.change_varbounds(model.X['x1'], lower_bound=-5.0, upper_bound=5.0)


    print('Done')


def createInstance( filename ):
    """Find the model specification in this function. It creates an
    instance of this model using the provided data file and returns this
    instance.
    """

    # Create an abstract model for a QCQP with linear objective
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
    def cons_rule(model, i):
        quad = sum(sum(model.A[i, k1, k2] * model.X[k1] * model.X[k2]
                       for k1 in model.V) for k2 in model.V)
        lin = sum(model.b[i, k] * model.X[k] for k in model.V) + model.c[i]
        return model.cl[i], quad + lin, model.cu[i]
    model.Quadcons = Constraint(model.C, rule=cons_rule)

    instance = model.create_instance(filename)

    return instance
