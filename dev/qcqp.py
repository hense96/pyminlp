from pyomo.environ import *



def solveQCQP(filename):

    # create model instance first
    instance = createInstance(filename)

    # display read instance
    #instance.pprint()

    # solve instance
    #opt = PyMINLP()
    #results = opt.solve(instance)


def createInstance(filename):
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
    model.b = Param(model.C, model.V, dfault=0)
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
        quad = sum(sum(model.A[i, k1, k2] * model.X[k1] * model.X[k2] for k1 in
                       model.V) for k2 in model.V)
        lin = sum(model.b[i, k] * model.X[k] for k in model.V) + model.c[i]
        return model.cl[i], quad + lin, model.cu[i]
    model.Cons = Constraint(model.C, rule=cons_rule)

    instance = model.create_instance(filename)

    return instance
