



import unittest

from pyomo.environ import *

from pyminlp.subprob import Instance
from pyminlp.conshdlr import ConsHandler


class DiseaseEstimation(unittest.TestCase):


    def setUp(self):
        model = AbstractModel()

        model.S_SI = Set(ordered=True)

        model.P_REP_CASES = Param(model.S_SI)
        model.P_POP = Param()

        model.I = Var(model.S_SI, bounds=(0, model.P_POP), initialize=1)
        model.S = Var(model.S_SI, bounds=(0, model.P_POP), initialize=300)
        model.beta = Var(bounds=(0.05, 70))
        model.alpha = Var(bounds=(0.5, 1.5))
        model.eps_I = Var(model.S_SI, initialize=0.0)

        def _objective(model):
            return sum((model.eps_I[i]) ** 2 for i in model.S_SI)

        model.objective = Objective(rule=_objective, sense=minimize)

        def _InfDynamics(model, i):
            if i != 1:
                return model.I[i] == (model.beta * model.S[i - 1] * model.I[
                    i - 1] ** model.alpha) / model.P_POP
            return Constraint.Skip

        model.InfDynamics = Constraint(model.S_SI, rule=_InfDynamics)

        def _SusDynamics(model, i):
            if i != 1:
                return model.S[i] == model.S[i - 1] - model.I[i]
            return Constraint.Skip

        model.SusDynamics = Constraint(model.S_SI, rule=_SusDynamics)

        def _Data(model, i):
            return model.P_REP_CASES[i] == model.I[i] + model.eps_I[i]

        model.Data = Constraint(model.S_SI, rule=_Data)

        instance = model.create_instance(
                         'instances/pyomo_disease_estimation.dat')

        self.instance = instance
        self.dummy1 = ConsHandlerDummy1()
        self.dummy2 = ConsHandlerDummy2()

    def parameter_consistency_test(self):
        # Initialisation test.
        int_inst = Instance.create_instance(self.instance)
        # Count constraints.
        n_conss = 3 * len(self.instance.S_SI.value) - 2
        self.assertEqual(int_inst._model.nconstraints(), n_conss)
        self.assertEqual(len(int_inst._consmap), n_conss)
        self.assertEqual(len(int_inst._classif), 0)
        self.assertEqual(int_inst._n_unclassif, n_conss)
        self.assertEqual(len(int_inst.get_unclassified()), n_conss)
        self.assertEqual(len(int_inst.get_unclassified('InfDynamics')),
                        len(self.instance.S_SI.value) - 1)
        self.assertEqual(len(int_inst.get_constypes()), 3)
        # Check _Cons object content.
        constypes = set([])
        for c in int_inst._consmap.keys():
            cons = int_inst._consmap[c]
            self.assertTrue(cons.pyrep is not None)
            self.assertTrue(cons.name is not None)
            self.assertTrue(cons.constype is not None)
            self.assertTrue(cons.index is not None)
            self.assertTrue(cons.conshdlr is None)
            self.assertEqual(cons.name, c)
            model_cons = int_inst._model.component(cons.constype)
            self.assertEqual(model_cons[cons.index], cons.pyrep)
            self.assertEqual('{}[{}]'.format(cons.constype, cons.index),
                             cons.name)
            constypes.add(cons.constype)
        self.assertEqual(constypes,
                        set(int_inst.get_constypes()))

        # Register constraint handler.
        typelist = int_inst.get_constypes()
        removed_type = 'InfDynamics'
        typelist.remove(removed_type)
        for constype in typelist:
            cons = int_inst.model().component(constype)
            indices = int_inst.get_unclassified(constype)
            for index in indices:
                int_inst.register_conshandler(cons[index].name, self.dummy2)
            for index in indices:
                int_inst.register_conshandler(cons[index].name, self.dummy1)
        self.assertEqual(int_inst._n_unclassif, len(self.instance.S_SI.value)
                         - 1)
        self.assertEqual(len(int_inst.get_unclassified()),
                        len(self.instance.S_SI.value) - 1)
        self.assertEqual(len(int_inst.get_unclassified(removed_type)),
                        len(self.instance.S_SI.value) - 1)
        for constype in typelist:
            self.assertEqual(len(int_inst.get_unclassified(constype)), 0)
        for index in int_inst.get_unclassified(removed_type):
            cons = int_inst.model().component(removed_type)
            int_inst.register_conshandler(cons[index].name, self.dummy2)
        self.assertEqual(int_inst._n_unclassif, 0)
        self.assertEqual(len(int_inst.get_unclassified()), 0)

        # Add constraints.
        add_model = ConcreteModel()
        add_model.S = Set(initialize=[1, 2, 3])
        add_model.Par = Param(add_model.S, initialize={1:1, 2:2, 3:3})
        add_model.I = Var(self.instance.S_SI, bounds=(0, self.instance.P_POP),
                          initialize=1)

        def cut_rule(model, j):
            return model.I[3] <= add_model.Par[j]

        add_model.Cut = Constraint(add_model.S, rule=cut_rule)

        for i in range(5):
            int_inst.add_constraints('Cut', add_model.Cut,
                                     {'Par':add_model.Par})

        # Check constraint numbers.
        n_conss += 15
        # TODO nconstraints() does not get updated.
        #self.assertEqual(int_inst._model.nconstraints(), n_conss)
        count = 0
        for i in int_inst._model.component_objects(Constraint):
            for j in int_inst._model.component(i):
                count += 1
        self.assertEqual(count, n_conss)
        self.assertEqual(len(int_inst._consmap), n_conss)
        self.assertEqual(int_inst._n_unclassif, 15)
        self.assertEqual(len(int_inst.get_unclassified()), 15)
        self.assertEqual(len(int_inst.get_unclassified('Cut')), 15)
        self.assertEqual(len(int_inst.get_constypes()), 4)

        for c in int_inst._consmap.keys():
            cons = int_inst._consmap[c]
            self.assertTrue(cons.pyrep is not None)
            self.assertTrue(cons.name is not None)
            self.assertTrue(cons.constype is not None)
            self.assertTrue(cons.index is not None)
            self.assertEqual(cons.name, c)
            model_cons = int_inst._model.component(cons.constype)
            self.assertEqual(model_cons[cons.index], cons.pyrep)
            self.assertEqual('{}[{}]'.format(cons.constype, cons.index),
                             cons.name)
            constypes.add(cons.constype)

        cons = int_inst.model().component('Cut')
        for index in int_inst.get_unclassified('Cut'):
            int_inst.register_conshandler(cons[index].name, self.dummy2)
        self.assertEqual(int_inst._n_unclassif, 0)

        self.assertEqual(len(int_inst._classif.keys()), 2)
        count = 0
        consset = set([])
        for key in int_inst._classif.keys():
            list = int_inst._classif[key]
            count += len(list)
            consset = consset.union(set(list))
        self.assertEqual(count, len(consset))
        self.assertEqual(count, len(int_inst._consmap.keys()))

        # Check parameters
        self.assertEqual(len(int_inst._model.component('Par')), 15)


class QCQP(unittest.TestCase):


    def setUp(self):
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

        instance = model.create_instance('instances/minlplib_ex_9_2_8.dat')

        self.instance = instance


class CrazyInstance(unittest.TestCase):


    def setUp(self):
        pass


class ConsHandlerDummy1(ConsHandler):
    def name(self):
        return 'dummy1'

class ConsHandlerDummy2(ConsHandler):
    def name(self):
        return 'dummy2'

if __name__ == '__main__':
    unittest.main()
