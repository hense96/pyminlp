# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


import unittest

from pyomo.environ import *

from pyutilib.component.core import *

from pyminlp.solver import PyMINLP
from pyminlp.hub import SolvingStage
from pyminlp.hub import SolvingStageError
from pyminlp.hub import UserInputError
from pyminlp.conshdlr import *
from pyminlp.bnb import BranchAndBound
from pyminlp.bnb import UserInputStatus
from pyminlp.subprob import Instance
from pyminlp.stats import Stats


class InputTest(unittest.TestCase):

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

        def simple_rule(model):
            return sum(model.S[i] for i in model.S_SI) <= 0

        clone = model.clone()
        clone.Param1 = Param(initialize=1)
        clone.Param2 = Param(initialize=2)
        clone.Cons = Constraint(rule=simple_rule)
        self.add_param1 = clone.Param1
        self.add_param2 = clone.Param2
        self.add_cons = clone.Cons

        self.instance = instance
        self.int_inst = Instance.create_instance(instance)

        INTTEST_Hdlr_1._test = self
        INTTEST_Hdlr_2._test = self

    def set_up_test(self):
        """Test of different function usages for setting up the solver.
        """
        self.solver = PyMINLP()
        solver = self.solver
        self.coord = solver._coordinator
        coord = self.coord

        self.assertRaises(ValueError, solver.solve, self.instance)

        self.assertRaises(ValueError, solver.set_relaxation_solver, 2)
        solver.set_relaxation_solver(solver)
        solver.set_relaxation_solver(SolverFactory('glpk'))

        self.assertRaises(ValueError, solver.solve, self.instance)

        solver.set_epsilon(0.1, 0.2)
        self.assertEqual(coord._gap_epsilon, 0.1)
        self.assertEqual(coord._cons_epsilon, 0.2)

        solver.set_epsilon(constraint_epsilon=0.3)
        self.assertEqual(coord._gap_epsilon, 0.1)
        self.assertEqual(coord._cons_epsilon, 0.3)

        self.assertRaises(ValueError, solver.solve, self.instance)
        self.assertEqual(coord._stage, SolvingStage.SETUP)

        self.assertRaises(ValueError, solver.set_verbosity, '2')
        self.assertRaises(ValueError, solver.set_verbosity, 4)
        solver.set_verbosity(1)

        self.assertRaises(ValueError, solver.solve, self.instance)
        self.assertEqual(coord._stage, SolvingStage.SETUP)

        self.assertRaises(SolvingStageError, solver.add_constraints,
                          'Dummy', self.add_cons)
        self.assertRaises(SolvingStageError, solver.branch, 'S[3]', 0)
        self.assertRaises(SolvingStageError, solver.change_bounds, 'S[3]', 0)
        self.assertRaises(SolvingStageError, solver.match, 'Data[3]')

        solver.use_constraint_handler('INTTEST_Hdlr_1',
                                      ['InfDynamics', 'SusDynamics'],
                                      2)

        self.assertRaises(ValueError, solver.solve, self.instance)
        self.assertRaises(ValueError, solver.use_constraint_handler,
                          'INTTEST_Hdlr_1',
                          ['InfDynamics', 'SusDynamics', 'Data'], 2)

        solver.use_constraint_handler('INTTEST_Hdlr_2',
                                      ['InfDynamics', 'Data'], 3)

        self.assertRaises(UserInputError, solver.solve, self.instance)

    def solving_stage_test(self):
        """Test the call of different interface functions in the
        solving stage.
        """
        self.solver = PyMINLP()
        solver = self.solver
        self.coord = solver._coordinator
        coord = self.coord
        coord._bnb_tree = BranchAndBound.create(coord)
        Stats.initialise(1)

        # Test identify.
        solver.use_constraint_handler('INTTEST_Hdlr_1', ['Data'], 1, 1, True)
        solver.use_constraint_handler('INTTEST_Hdlr_2',
                                      ['InfDynamics', 'SusDynamics'], 2, 2)

        coord._stage = SolvingStage.SOLVING

        self.assertRaises(SolvingStageError, solver.set_epsilon)

        coord.set_instance(self.int_inst)

        self.assertRaises(UserInputError, coord.identify)

        # Test prepare.
        INTTEST_Hdlr_1._mode = 1
        INTTEST_Hdlr_2._mode = 1
        self.assertEqual(coord.prepare(), UserInputStatus.BRANCHED)
        INTTEST_Hdlr_1._mode = 2
        INTTEST_Hdlr_2._mode = 2
        self.assertEqual(coord.prepare(), UserInputStatus.INFEASIBLE)
        INTTEST_Hdlr_1._mode = 3
        INTTEST_Hdlr_2._mode = 3
        self.assertEqual(coord.prepare(), UserInputStatus.OK)
        INTTEST_Hdlr_1._mode = 4
        INTTEST_Hdlr_2._mode = 4
        self.assertRaises(UserInputError, coord.prepare)


class INTTEST_Hdlr_1(SingletonPlugin):
    implements(IPyMINLPConsHandler)

    _test = None
    _mode = 0

    def identify(self, sets, model, tsolver):
        test = INTTEST_Hdlr_1._test
        solver = test.solver
        if INTTEST_Hdlr_1._mode != 0:
            for type in sets.keys():
                indices = sets[type]
                for index in indices:
                    solver.match(model.component(type)[index])
        else:
            test.assertRaises(ValueError, solver.match, 2)
            nunc = test.coord._cur_instance.nunclassified()
            solver.match('Data[3]')
            test.assertEqual(test.coord._cur_instance.nunclassified() + 1, nunc)
            solver.match('Data[3]')
            test.assertEqual(test.coord._cur_instance.nunclassified() + 1, nunc)

    def prepare(self, sets, model, tsolver):
        test = INTTEST_Hdlr_1._test
        solver = test.solver
        if INTTEST_Hdlr_1._mode == 1:
            solver.add_constraints('Dummy1', test.add_cons)
            solver.branch('S[3]', 0)
        elif INTTEST_Hdlr_1._mode == 2:
            solver.add_constraints('Dummy2', test.add_cons)
            solver.declare_infeasible()
            solver.branch('S[3]', 0)
        elif INTTEST_Hdlr_1._mode == 3:
            solver.add_constraints('Dummy3', test.add_cons)
        else:
            solver.match('Data[3]')

    def enforce(self, sets, model, tsolver):
        test = INTTEST_Hdlr_1._test
        solver = test.solver
        if INTTEST_Hdlr_1._mode == 1:
            solver.add_constraints('Dummy1', test.add_cons)
            solver.branch('S[3]', 0)
        elif INTTEST_Hdlr_1._mode == 2:
            solver.add_constraints('Dummy2', test.add_cons)
            solver.declare_infeasible()
            solver.branch('S[3]', 0)
        elif INTTEST_Hdlr_1._mode == 3:
            solver.add_constraints('Dummy3', test.add_cons)
        elif INTTEST_Hdlr_1._mode == 4:
            solver.match('Data[3]')
        elif INTTEST_Hdlr_1._mode == 5:
            return


class INTTEST_Hdlr_2(SingletonPlugin):
    implements(IPyMINLPConsHandler)

    _test = None
    _mode = 0

    @classmethod
    def create(cls):
        return INTTEST_Hdlr_2()

    @classmethod
    def name(cls):
        return '#2'

    def identify(self, sets, model, tsolver):
        test = INTTEST_Hdlr_2._test
        solver = test.solver
        if INTTEST_Hdlr_2._mode != 0:
            for type in sets.keys():
                indices = sets[type]
                for index in indices:
                    solver.match(model.component(type)[index])

    def prepare(self, sets, model, tsolver):
        if INTTEST_Hdlr_2._mode == 1 or INTTEST_Hdlr_2._mode == 2:
            raise Exception('Should not be reached')

    def enforce(self, sets, model, tsolver):
        if INTTEST_Hdlr_2._mode == 1 or INTTEST_Hdlr_2._mode == 2:
            raise Exception('Should not be reached')


def main():
    suite1 = InputTest()
    suite1.setUp()
    suite1.set_up_test()
    suite1.setUp()
    suite1.solving_stage_test()


if __name__ == '__main__':
    unittest.main()
