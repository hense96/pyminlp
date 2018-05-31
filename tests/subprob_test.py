

import unittest

from pyomo.environ import *
from pyomo.opt import TerminationCondition
import numpy as np

from pyminlp.subprob import Instance
from pyminlp.conshdlr import ConsHandlerManager


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

        Tested attributes:
            _consmap
            _classif

        Tested functions:
            nconstraints()
            nunclassified()
            unclassified(constype)
            constypes()
            register_conshandler(constraint, conshandler)
            add_constraints(constype, constraints, parameters)
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
        add_model.Par = Param(add_model.S, initialize={1: 1, 2: 2, 3: 3})
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

        Tested attributes:
            _model
            _consmap
            _classif

        Tested functions:
            Instance.derive_instance(instance)
            nconstraints()
            nunclassified()
            register_conshandler(constraint, conshandler)
            add_constraints(constype, constraints, parameters)
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
        int_inst_base = Instance.create_instance(self.instance)

        # Check change_varbounds(...) function using different types of
        # variables.
        int_inst = int_inst_base._clone()
        simple = self.instance.alpha
        indexed = self.instance.I
        int_inst.change_varbounds(simple.name, 0, 1)
        for v in indexed:
            int_inst.change_varbounds(indexed[v].name, -1.0, 1.0)
        self.assertEqual(int_inst.model().alpha.lb, 0.0)
        self.assertEqual(int_inst.model().alpha.ub, 1.0)
        for v in int_inst.model().I:
            self.assertEqual(int_inst.model().I[v].lb, -1.0)
            self.assertEqual(int_inst.model().I[v].ub, 1.0)

        # Check false input for change_varbounds(...) function.
        self.assertRaises(KeyError, int_inst.change_varbounds, ('novar'))
        # The varbounds itself are arbitrary values. This is a Pyomo
        # specification though, so I will not restrict it.

        # Add different kinds of constraints and parameters.
        int_inst = int_inst_base._clone()
        add_model = ConcreteModel()
        add_model.sset1 = Set(initialize=['a', 'b', 'c'])
        add_model.sset2 = Set(initialize=['d', 'e', 'f'])
        add_model.cset1 = Set(add_model.sset1, add_model.sset1,
                              initialize={('a', 'a'): 'a', ('b', 'b'): 'b'})
        add_model.cset2 = self.instance.S_SI * self.instance.S_SI
        add_model.v = Var(add_model.sset2)
        int_inst.model().v = Var(add_model.sset2)

        add_model.scons = Constraint(expr=add_model.v['d']
                                          - add_model.v['e'] <= 0)

        def simple_rule(model, i):
            return add_model.v['d'] - add_model.v['e'] <= 0

        def complex_rule(model, i, j):
            return add_model.v['d'] - add_model.v['e'] <= 0

        add_model.sindcons = Constraint(add_model.sset1, rule=simple_rule)
        add_model.dindcons = Constraint(add_model.sset1, add_model.sset1,
                                        rule=complex_rule)
        # This is not possible in Pyomo.
        #add_model.ccons1 = Constraint(add_model.cset1, rule=simple_rule)
        add_model.ccons2 = Constraint(add_model.cset2, rule=complex_rule)

        # Perform adding without parameters.
        for i in range(3):
            int_inst.add_constraints('scons', add_model.scons)
            int_inst.add_constraints('sindcons', add_model.sindcons)
            self.assertRaises(ValueError, int_inst.add_constraints,
                              'dindcons', add_model.dindcons)
            int_inst._consadd_count += 1
            self.assertRaises(ValueError, int_inst.add_constraints,
                              'ccons2', add_model.ccons2)
            int_inst._consadd_count += 1

        scons = int_inst.model().component('scons')
        sindcons = int_inst.model().component('sindcons')

        self.assertEqual(len(scons), 3)
        self.assertEqual(len(sindcons), 9)

        add_model.list = ConstraintList()
        for v in scons:
            add_model.list.add(scons[v].expr)
        for v in sindcons:
            add_model.list.add(sindcons[v].expr)

        int_inst.add_constraints('list', add_model.list)

        list = int_inst.model().component('list')
        self.assertEqual(len(list), 12)

        # Test different parameter types.
        int_inst = int_inst_base._clone()
        add_model.sset3 = Set(initialize=['g', 'h', 'i'])
        int_inst.model().v = Var(add_model.sset2)

        def param_rule_1(model, i):
            return 1

        def param_rule_2(model, i, j):
            return 1

        def param_rule_3(model, i, j, k):
            return 1

        # sset1 is the constraint index set, if constraint is not
        # a SimpleCons

        add_model.s = Param(initialize=1)
        add_model.in0_s = Param(add_model.sset2, rule=param_rule_1)
        add_model.in0_m = Param(add_model.sset2, add_model.sset2,
                                rule=param_rule_2)
        add_model.in1_s = Param(add_model.sset1, rule=param_rule_1)
        add_model.in1_m1 = Param(add_model.sset1, add_model.sset2,
                                 rule=param_rule_2)
        add_model.in1_m2 = Param(add_model.sset2, add_model.sset1,
                                 rule=param_rule_2)
        add_model.in2_s = Param(add_model.sset1, add_model.sset1,
                                rule=param_rule_2)
        add_model.in2_m = Param(add_model.sset2, add_model.sset1,
                                add_model.sset1, rule=param_rule_3)

        for i in range(3):
            int_inst.add_constraints('scons', add_model.scons,
                                     {'s': add_model.s,
                                      'in0_s': add_model.in0_s,
                                      'in0_m': add_model.in0_m})

        # implement checks here
        s = int_inst.model().s
        in0_s = int_inst.model().in0_s
        in0_m = int_inst.model().in0_m
        self.assertEqual(len(s), 3)
        self.assertEqual(s.index_set().name, '_scons_Set')
        self.assertEqual(len(in0_s.index_set().set_tuple), 2)
        self.assertEqual(in0_s.index_set().set_tuple[0].name, '_scons_Set')
        self.assertEqual(len(in0_m.index_set().set_tuple), 3)
        self.assertEqual(in0_m.index_set().set_tuple[0].name, '_scons_Set')
        self.assertEqual(len(int_inst.model()._scons_Set), 3)

        int_inst = int_inst_base._clone()
        int_inst.model().v = Var(add_model.sset2)

        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in2_s': add_model.in2_s})

        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in2_m': add_model.in2_m})

        for i in range(3):
            int_inst.add_constraints('sindcons', add_model.sindcons,
                                     {'s': add_model.s,
                                      'in0_s': add_model.in0_s,
                                      'in0_m': add_model.in0_m,
                                      'in1_s': add_model.in1_s,
                                      'in1_m1': add_model.in1_m1,
                                      'in1_m2': add_model.in1_m2})

        consadd_count = int_inst._consadd_count
        index = '_{}_b'.format(consadd_count - 2)

        # implement checks here
        s = int_inst.model().s
        in0_s = int_inst.model().in0_s
        in0_m = int_inst.model().in0_m
        in1_s = int_inst.model().in1_s
        in1_m1 = int_inst.model().in1_m1
        in1_m2 = int_inst.model().in1_m2
        self.assertEqual(len(s), 9)
        self.assertEqual(s.index_set().name, '_sindcons_Set')
        self.assertEqual(len(in0_s.index_set().set_tuple), 2)
        self.assertEqual(s[index], 1)
        self.assertEqual(in0_s.index_set().set_tuple[0].name, '_sindcons_Set')
        self.assertEqual(len(in0_m.index_set().set_tuple), 3)
        self.assertEqual(in0_s[index, 'e'], 1)
        self.assertEqual(in0_m.index_set().set_tuple[0].name, '_sindcons_Set')
        self.assertEqual(len(int_inst.model()._sindcons_Set), 9)
        self.assertEqual(in0_m[index, 'e', 'f'], 1)

        self.assertEqual(len(in1_s), 9)
        self.assertEqual(in1_s.index_set().name, '_sindcons_Set')
        self.assertEqual(in1_s[index], 1)
        self.assertEqual(len(in1_m1.index_set().set_tuple), 2)
        self.assertEqual(in1_m1.index_set().set_tuple[0].name, '_sindcons_Set')
        self.assertEqual(in1_m1[index, 'e'], 1)
        self.assertEqual(len(in1_m2.index_set().set_tuple), 2)
        self.assertEqual(in1_m2.index_set().set_tuple[1].name, '_sindcons_Set')
        self.assertEqual(in1_m2['e', index], 1)

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_s': add_model.s})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_m': add_model.s})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_m1': add_model.s})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_m2': add_model.s})

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'s': add_model.in0_s})

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_m': add_model.in0_s})

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_s': add_model.in0_s})

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_m2': add_model.in0_s})

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'s': add_model.in0_m})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_s': add_model.in0_m})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_s': add_model.in0_m})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_m1': add_model.in0_m})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_m2': add_model.in0_m})

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_s': add_model.in1_s})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_m': add_model.in1_s})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_m1': add_model.in1_s})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_m2': add_model.in1_s})

        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'s': add_model.in1_m2})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_s': add_model.in1_m2})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in0_m': add_model.in1_m2})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_s': add_model.in1_m2})
        Instance._consadd_count += 1
        self.assertRaises(ValueError, int_inst.add_constraints,
                          'sindcons', add_model.sindcons,
                          {'in1_s': add_model.in1_m2})


class SimpleInstance(unittest.TestCase):


    def setUp(self):
        """Generates a simple instance and saves the resulting
        constructed ConcreteModel. Additionally creates two dummy
        constraint handler and a relaxation solver.
        """
        # Create a concrete model.
        model = ConcreteModel()

        # Sets
        model.V = Set(initialize=[1, 2])

        # Bounds
        def var_bounds_rule(model, k):
            return -np.Inf, np.Inf

        # Variables
        model.X = Var(model.V, bounds=var_bounds_rule, domain=Reals)

        model.XS = Var(bounds=(-1, 1))

        # Objective
        def obj_rule(model):
            return model.X[1] + model.X[2]

        model.Obj = Objective(rule=obj_rule, sense=minimize)

        # Constraints
        def cons_rule_1(model):
            return model.X[1] >= 0

        model.C1 = Constraint(rule=cons_rule_1)

        def cons_rule_2(model):
            return model.X[2] >= 0

        model.C2 = Constraint(rule=cons_rule_2)

        self.instance = model
        self.dummy1 = ConsHandlerDummy1()
        self.dummy2 = ConsHandlerDummy2()
        self.relax_solver = [SolverFactory('cbc'), SolverFactory('glpk')]


    def relaxation_solution_test(self):
        """This test focuses on the correct implementation of all data
        that are associated with the solve_relaxation() method.

        Tested functions:
            Instance.derive_instance(instance)
            solve_relaxation(relaxation_solver)
            relaxation_solved()
            feasible()
            has_optimal_solution()
            relax_infeasible()
            relax_termination_condition()
            objective_function_value()
            violated_sets(conshandler)
            nviolated(conshandler)
            relax_model()
            add_constraints(constype, constraints, params)
            change_varbounds(varname, lower_bound, upper_bound)
        """
        int_inst = Instance.create_instance(self.instance)
        model = int_inst.model().clone()
        self.dummy1.set_relax(True)
        self.dummy2.set_relax(False)
        int_inst.register_conshandler(model.C1.name, self.dummy1)
        self.dummy1.add_constypes(['C1'])
        int_inst.register_conshandler(model.C2.name, self.dummy2)
        self.dummy2.add_constypes(['C2'])

        self.assertEqual(int_inst.relaxation_solved(), False)

        # Unbounded case.
        for relax_solver in self.relax_solver:
            int_inst.solve_relaxation(relax_solver)

            self.assertEqual(int_inst.relaxation_solved(), True)

            self.assertEqual(int_inst.relax_termination_condition(),
                             TerminationCondition.unbounded)
            int_inst.relax_model()
            self.assertEqual(int_inst.has_optimal_solution(), False)
            self.assertEqual(int_inst.relax_infeasible(), False)
            self.assertRaises(AssertionError, int_inst.feasible)
            self.assertRaises(AssertionError,
                              int_inst.objective_function_value)
            self.assertRaises(AssertionError, int_inst.violated_sets,
                              self.dummy1)
            self.assertRaises(AssertionError, int_inst.nviolated,
                              self.dummy2)

        def cons_rule_3(model):
            return model.X[2] >= -5

        model.Cons = Constraint(rule=cons_rule_3)

        int_inst.add_constraints('C3', model.Cons)

        self.assertEqual(int_inst.relaxation_solved(), False)

        new_cons = int_inst.model().component('C3')
        for c in new_cons:
            int_inst.register_conshandler(new_cons[c].name, self.dummy1)
        self.dummy1.add_constypes(['C3'])

        # Optimal case, infeasible for actual instance.
        for relax_solver in self.relax_solver:
            int_inst.solve_relaxation(relax_solver)

            self.assertEqual(int_inst.relaxation_solved(), True)

            self.assertEqual(int_inst.relax_termination_condition(),
                             TerminationCondition.optimal)
            int_inst.relax_model()
            self.assertEqual(int_inst.has_optimal_solution(), True)
            self.assertEqual(int_inst.relax_infeasible(), False)
            self.assertEqual(int_inst.feasible(), False)
            self.assertEqual(int_inst.objective_function_value(), -5)
            self.assertEqual(int_inst.nviolated(self.dummy1), 0)
            self.assertEqual(int_inst.nviolated(self.dummy2), 1)
            self.assertEqual(int_inst.violated_sets(self.dummy2),
                             {'C2': [None]})
            self.assertEqual(int_inst.violated_sets(self.dummy2),
                             int_inst.conshandler_sets(self.dummy2))

        child_inst = Instance.derive_instance(int_inst)

        self.assertEqual(child_inst.relaxation_solved(), False)
        child_inst.change_varbounds('X[2]', lower_bound=0)
        self.assertEqual(int_inst.relaxation_solved(), True)

        # Optimal case, feasible for actual instance.
        for relax_solver in self.relax_solver:
            child_inst.solve_relaxation(relax_solver)

            self.assertEqual(child_inst.relaxation_solved(), True)

            self.assertEqual(child_inst.relax_termination_condition(),
                             TerminationCondition.optimal)
            child_inst.relax_model()
            self.assertEqual(child_inst.has_optimal_solution(), True)
            self.assertEqual(child_inst.relax_infeasible(), False)
            self.assertEqual(child_inst.feasible(), True)
            self.assertEqual(child_inst.objective_function_value(), 0)
            self.assertEqual(child_inst.nviolated(self.dummy1), 0)
            self.assertEqual(child_inst.nviolated(self.dummy2), 0)
            self.assertEqual(child_inst.violated_sets(self.dummy2),
                             {'C2': []})

        def cons_rule_4(model):
            return model.X[2] <= -1

        model.Cons2 = Constraint(rule=cons_rule_4)

        child_inst.add_constraints('C3', model.Cons2)

        self.assertEqual(child_inst.relaxation_solved(), False)

        new_cons = child_inst.model().component('C3')
        for c in new_cons:
            child_inst.register_conshandler(new_cons[c].name, self.dummy1)
        self.dummy1.add_constypes(['C3'])

        # Infeasible case.
        for relax_solver in self.relax_solver:
            child_inst.solve_relaxation(relax_solver)

            self.assertEqual(child_inst.relaxation_solved(), True)

            self.assertEqual(child_inst.relax_termination_condition(),
                             TerminationCondition.infeasible)
            child_inst.relax_model()
            self.assertEqual(child_inst.has_optimal_solution(), False)
            self.assertEqual(child_inst.relax_infeasible(), True)
            self.assertRaises(AssertionError, child_inst.feasible)
            self.assertRaises(AssertionError,
                              child_inst.objective_function_value)
            self.assertRaises(AssertionError, child_inst.violated_sets,
                              self.dummy1)
            self.assertRaises(AssertionError, child_inst.nviolated,
                              self.dummy2)

        # Additional test: Change varbounds of a simple variable.
        int_inst.change_varbounds('XS', lower_bound=0)

        self.assertEqual(int_inst.model().component('XS').lb, 0)


class ConsHandlerDummy1(ConsHandlerManager):
    def name(self):
        return 'dummy1'

class ConsHandlerDummy2(ConsHandlerManager):
    def name(self):
        return 'dummy2'

def main():
    suite1 = DiseaseEstimation()
    suite1.setUp()
    suite1.data_consistency_test()
    suite1.setUp()
    suite1.copy_depth_test()
    suite1.setUp()
    suite1.plugin_function_test()
    suite2 = SimpleInstance()
    suite2.setUp()
    suite2.relaxation_solution_test()

if __name__ == '__main__':
    unittest.main()
