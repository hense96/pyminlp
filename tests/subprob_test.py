



import unittest

from pyomo.environ import *

from pyminlp.subprob import Instance
from pyminlp.conshdlr import ConsHandler


class DiseaseEstimation(unittest.TestCase):


    def setUp(self):
        """Generates the disease estimation model from the Pyomo book
        and saves the resulting constructed ConcreteModel. Additionally
        creates two dummy constraint handler.
        """
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


    def data_consistency_test(self):
        """ This test focuses on the consistency of the attributes of
        an Instance object and the consistency of the data providing
        functions executing different changes.
        """
        # Initialisation test.
        int_inst = Instance.create_instance(self.instance)
        # Count constraints.
        n_conss = 3 * len(self.instance.S_SI.value) - 2
        self.assertEqual(int_inst._model.nconstraints(), n_conss)
        self.assertEqual(len(int_inst._consmap), n_conss)
        self.assertEqual(len(int_inst._classif), 0)
        self.assertEqual(int_inst.nunclassified(), n_conss)
        self.assertEqual(len(int_inst.unclassified()), n_conss)
        self.assertEqual(len(int_inst.unclassified('InfDynamics')),
                         len(self.instance.S_SI.value) - 1)
        self.assertEqual(len(int_inst.constypes()), 3)
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
                         set(int_inst.constypes()))

        # Register constraint handler.
        typelist = int_inst.constypes()
        removed_type = 'InfDynamics'
        typelist.remove(removed_type)
        for constype in typelist:
            cons = int_inst.model().component(constype)
            indices = int_inst.unclassified(constype)
            for index in indices:
                int_inst.register_conshandler(cons[index].name, self.dummy1)
        self.assertEqual(int_inst.nunclassified(), len(self.instance.S_SI.value)
                         - 1)
        self.assertEqual(len(int_inst.unclassified()),
                         len(self.instance.S_SI.value) - 1)
        self.assertEqual(len(int_inst.unclassified(removed_type)),
                         len(self.instance.S_SI.value) - 1)
        for constype in typelist:
            self.assertEqual(len(int_inst.unclassified(constype)), 0)
        for index in int_inst.unclassified(removed_type):
            cons = int_inst.model().component(removed_type)
            int_inst.register_conshandler(cons[index].name, self.dummy2)
        self.assertEqual(int_inst.nunclassified(), 0)
        self.assertEqual(len(int_inst.unclassified()), 0)

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
        self.assertEqual(int_inst.nunclassified(), 15)
        self.assertEqual(len(int_inst.unclassified()), 15)
        self.assertEqual(len(int_inst.unclassified('Cut')), 15)
        self.assertEqual(len(int_inst.constypes()), 4)

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
        for index in int_inst.unclassified('Cut'):
            int_inst.register_conshandler(cons[index].name, self.dummy2)
        self.assertEqual(int_inst.nunclassified(), 0)

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


    def copy_depth_test(self):
        """This test focuses on the interdependence of Instance objects
        that are derived from each other. It checks if the attributes
        change globally or locally when changed locally. Some attributes
        have local and some have global character.
        """
        int_inst = Instance.create_instance(self.instance)

        # Attribute manipulation via constraint handler.
        constypes = int_inst.constypes()
        for constype in constypes:
            cons = int_inst.model().component(constype)
            indices = int_inst.unclassified(constype)
            for index in indices:
                int_inst.register_conshandler(cons[index].name, self.dummy2)
        count = int_inst.nunclassified()

        int_inst_l = Instance.derive_instance(int_inst)
        self.assertEqual(count, int_inst_l.nunclassified())

        int_inst_r = Instance.derive_instance(int_inst)
        self.assertEqual(count, int_inst_r.nunclassified())

        self.assertEqual(int_inst.nunclassified(), count)
        self.assertEqual(int_inst_r.nunclassified(), count)

        count_o = 0
        count_r = 0
        for key in int_inst._consmap.keys():
            hdlr = int_inst._consmap[key].conshdlr
            self.assertTrue(hdlr is None or hdlr.name() == 'dummy2')
            if hdlr is not None:
                count_o += 1
        for key in int_inst_r._consmap.keys():
            hdlr = int_inst_r._consmap[key].conshdlr
            self.assertTrue(hdlr is None or hdlr.name() == 'dummy2')
            if hdlr is not None:
                count_r += 1
        self.assertEqual(int_inst.nconstraints() - count_o, count)
        self.assertEqual(int_inst.nconstraints() - count_r, count)

        self.assertEqual(len(int_inst._classif.keys()), 1)
        self.assertEqual(len(int_inst_r._classif.keys()), 1)
        self.assertEqual(len(int_inst._classif['dummy2']),
                         int_inst.nconstraints())
        self.assertEqual(len(int_inst_r._classif['dummy2']),
                         int_inst_r.nconstraints())

        # Model manipulation.
        int_inst.model().InfDynamics.deactivate()
        int_inst.model().preprocess()

        int_inst_l._model.SusDynamics.deactivate()
        int_inst_l.model().preprocess()

        self.assertTrue(int_inst_l.model().InfDynamics._active)
        self.assertTrue(int_inst_r.model().InfDynamics._active)
        self.assertFalse(int_inst.model().InfDynamics._active)
        self.assertTrue(int_inst.model().SusDynamics._active)
        self.assertTrue(int_inst_r.model().SusDynamics._active)
        self.assertFalse(int_inst_l.model().SusDynamics._active)

        int_inst.model().InfDynamics.activate()
        int_inst.model().preprocess()

        int_inst_l.model().SusDynamics.activate()
        int_inst_l.model().preprocess()

        # Adding constraints
        n_cons = int_inst.nconstraints()
        self.assertEqual(n_cons, self.instance.nconstraints())
        self.assertEqual(n_cons, int_inst_r.nconstraints())

        add_model = ConcreteModel()
        add_model.S = Set(initialize=[1, 2, 3])
        add_model.Par = Param(add_model.S, initialize={1: 1, 2: 2, 3: 3})
        add_model.I = Var(self.instance.S_SI, bounds=(0, self.instance.P_POP),
                          initialize=1)

        def cut_rule(model, j):
            return model.I[3] <= add_model.Par[j]

        add_model.Cut = Constraint(add_model.S, rule=cut_rule)

        for i in range(5):
            int_inst_l.add_constraints('Cut', add_model.Cut,
                                     {'Par': add_model.Par})

        self.assertEqual(int_inst_l.nconstraints(), n_cons + 15)
        self.assertEqual(int_inst.nconstraints(), n_cons)
        self.assertEqual(int_inst_r.nconstraints(), n_cons)
        self.assertEqual(len(int_inst_l.constypes()), 4)
        self.assertEqual(len(int_inst.constypes()), 3)
        self.assertEqual(len(int_inst_r.constypes()), 3)

        for i in range(3):
            int_inst_r.add_constraints('Cut', add_model.Cut,
                                     {'Par': add_model.Par})

        self.assertEqual(int_inst_r.nconstraints(), n_cons + 9)
        self.assertEqual(int_inst_l.nconstraints(), n_cons + 15)
        self.assertEqual(int_inst.nconstraints(), n_cons)

        # Check attributes again
        self.assertEqual(len(int_inst_r._consmap.keys()), n_cons + 9)
        self.assertEqual(len(int_inst_l._consmap.keys()), n_cons + 15)
        self.assertEqual(len(int_inst._consmap.keys()), n_cons)

        count = len(int_inst_l._classif['dummy2'])

        for index in int_inst_r.unclassified('Cut'):
            cons = int_inst_r.model().Cut[index]
            int_inst_r.register_conshandler(cons.name, self.dummy2)

        self.assertEqual(count, len(int_inst_l._classif['dummy2']))
        self.assertEqual(count, int_inst_l.nconstraints()
                                - int_inst_l.nunclassified())

        for i in range(1):
            int_inst.add_constraints('Cut', add_model.Cut,
                                     {'Par': add_model.Par})

        self.assertEqual(int_inst_r.nconstraints(), n_cons + 9)
        self.assertEqual(int_inst_l.nconstraints(), n_cons + 15)
        self.assertEqual(int_inst.nconstraints(), n_cons + 3)

        # Check attributes again
        self.assertEqual(len(int_inst_r._consmap.keys()), n_cons + 9)
        self.assertEqual(len(int_inst_l._consmap.keys()), n_cons + 15)
        self.assertEqual(len(int_inst._consmap.keys()), n_cons + 3)

        for index in int_inst.unclassified('Cut'):
            cons = int_inst.model().Cut[index]
            int_inst.register_conshandler(cons.name, self.dummy2)

        self.assertEqual(count, len(int_inst_l._classif['dummy2']))
        self.assertEqual(count, int_inst_l.nconstraints()
                                - int_inst_l.nunclassified())


    def plugin_function_test(self):
        """This test focuses on the correct implementation and the
        reaction to bad uses of the functions that are relevant for the
        plugins, i.e. add_constraints(...) and change_varbounds(...).
        """
        pass


class QCQP(unittest.TestCase):


    def setUp(self):
        """Generates the disease estimation model from the Pyomo book
        and saves the resulting constructed ConcreteModel. Additionally
        creates two dummy constraint handler.
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


    def plugin_function_test(self):
        """This test focuses on the correct implementation and the
        reaction to bad uses of the functions that are relevant for the
        plugins, i.e. add_constraints(...) and change_varbounds(...).
        """
        pass


class ConsHandlerDummy1(ConsHandler):
    def name(self):
        return 'dummy1'

class ConsHandlerDummy2(ConsHandler):
    def name(self):
        return 'dummy2'

if __name__ == '__main__':
    unittest.main()
