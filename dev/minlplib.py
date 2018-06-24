# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


from pathlib import Path
import time

from pyminlp.solver import PyMINLP
from pyminlp.plugins.miqcqp import *
from pyminlp.stats import Stats

from dev.osil_parser.osilParser import OsilParser
from dev.osil_parser.variable import Variable
from dev.osil_parser.objectiveFunction import ObjectiveFunction
from dev.osil_parser.quadraticConstraint import QuadraticConstraint


class MINLPlibTester:
    """
    This class offers an environment to read and solve .osil from the
    MINLPlib. It makes use of the osil_parser module.

    Currently, only MIQCQP and subclasses are supported.
    """

    @classmethod
    def create(self, paths, include_convex_hdlr=True):
        """Factory method to generate a new MINLPlibTester object.
        :coordinator: The list of directory paths (str) containing the
        relevant .osil files.
        """
        tester = MINLPlibTester()
        for path in paths:
            tester.add_to_testset(path)
        tester._set_up_solver(include_convex_hdlr)
        return tester

    def __init__(self):
        self._testset = []
        self._solver = None
        self._output_file = None
        self._sol_file = None

    def add_to_testset(self, dir_str):
        """Adds all .osil files of the given directory (str) to the
        list of instances to be solved."""
        pathlist = Path(dir_str).glob('**/*.osil')
        for path in pathlist:
            self._testset.append(str(path))

    def add_file_to_testset(self, path):
        """Adds the .osil file of the given path (str) to the
        list of instances to be solved."""
        self._testset.append(path)

    def execute(self):
        """Solves all given instances."""
        if self._output_file is not None:
            curtime = time.asctime(time.localtime(time.time()))
            file = open(self._output_file, 'a')
            file.write('\n \n --> MINLPlibTester run on {}\n'.format(curtime))
            file.close()
        i = 0
        for filename in self._testset:
            self._solve_instance(filename)
            i += 1
            if self._output_file is not None:
                print('Solved instances: {}/{}'.format(i, len(self._testset)))

    def set_output_file(self, filename):
        """Registers a file to print the output to."""
        self._output_file = filename

    def set_sol_file(self, filename):
        """Registers the path (str) of the minlplib.solu file."""
        self._sol_file = filename

    def solver(self):
        """Returns the PyMINLP solver object. The user might manipulate
        the settings of the solver.
        """
        return self._solver

    # Internal functions.

    def _set_up_solver(self, include_convex_hdlr):
        """Generate the PyMINLP solver and set default settings."""
        solver = PyMINLP()

        solver.use_constraint_handler(name='MIQCQPLinear',
                                      constypes=['Quadcons1', 'Quadcons2',
                                                 'Cut'],
                                      identify_prio=1,
                                      enforce_prio=1,
                                      relaxation=True)
        if include_convex_hdlr:
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
        solver.set_verbosity(1)
        solver.set_time_limit(100)

        self._solver = solver

    def _reset_solver(self):
        """Call this function before solving a new instance with the
        same solver object.
        """
        reset_varlist()

    def _solve_instance(self, filename):
        """Function for solving the instance defined in the given file.
        :param filename: Path to desired instance file (str).
        """
        if self._output_file is None:
            print('Read file {}.'.format(filename))

        problem = self._read_problem(filename)
        is_min = problem.objectiveFunctions[0].maxOrMin == 'min'
        is_quad, fake_bounds = self._prepare(problem)
        instance = self._create_pyomo_instance(problem)

        if self._output_file is None:
            print('Solve instance {}.'.format(filename))

        self._reset_solver()
        solver = self._solver
        try:
            res = solver.solve(instance)
        except Exception:
            if self._output_file is None:
                print('Exception in solving process.')
            else:
                out = '\nException in solving process for instance {}.\n' \
                      ''.format(filename)
                file = open(self._output_file, 'a')
                file.write(out)
                file.close()
            return

        if self._output_file is None:
            # print(res)
            print(self._create_result_output(filename, is_min, is_quad,
                                             fake_bounds, res))
            # pass
        else:
            out = self._create_result_output(filename, is_min, is_quad,
                                             fake_bounds, res)
            file = open(self._output_file, 'a')
            file.write(out)
            file.close()

        if self._output_file is None:
            print('Instance {} done.'.format(filename))

    def _read_problem(self, filename):
        """Creates a osil_parser.quadraticProblem object from an osil
        file by parsing the file.
        """
        parser = OsilParser(filename)
        return parser.parseProblem()

    def _prepare(self, problem):
        """Standardises the problem instance to a minimisation problem
        with linear objective function and sets infinite variable
        bounds to (-)1,000,000 in order to meet the requirements of
        the solver.
        :param problem: An osil_parser.quadraticProblem object.
        :return: True if the given object has a quadratic objective and
        the number variable bounds set to (-)1,000,000.
        """
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
                var.lb = -100000000
                bound_count += 1
            if np.isinf(var.ub):
                var.ub = 100000000
                bound_count += 1
        if self._output_file is None:
            print('Randomly set variable bounds: {}'.format(bound_count))
        return quad_obj, bound_count

    def _create_result_output(self, filename, is_min, is_quad,
                              fake_bounds, res):
        """Creates a string summarising the solving process of an
        instance.
        """
        # Get additional data.
        if self._sol_file is not None:
            instance_name = (filename.split('/')[-1]).split('.')[0]
            lower, upper = self._read_bounds(instance_name)
        else:
            upper = 'unknown'
            lower = 'unknown'

        out = '\n--------------------------------------------------\n'
        out += 'Instance: \t {}\n'.format(filename)
        out += 'Objective: \t {} {}\n'.format('min' if is_min else 'max',
                                              'quad' if is_quad else 'lin')
        out += 'Upper bound: \t {}\n'.format(upper)
        out += 'Lower bound: \t {}\n'.format(lower)
        out += 'Constraints: \t {}\n'.format(
            Stats.get_bounding_initial_nconss())
        out += 'Variables: \t {}\n'.format(res.problem.number_of_variables)
        out += 'Nonzeros: \t {}\n'.format(res.problem.number_of_nonzeros)
        out += '\n'
        out += 'Solver time: \t {:.3f}\n'.format(res.solver.wallclock_time)
        out += 'Solution status: {}\n'.format(
            res.solver.termination_condition)
        out += 'Upper bound: \t {}\n'.format(res.problem.upper_bound)
        out += 'Lower bound: \t {}\n'.format(res.problem.lower_bound)

        key = 'Number of created subproblems'
        nsubprob = res.solver.statistics.branch_and_bound[key]
        out += 'Created nodes: \t {}\n'.format(nsubprob)
        key = 'Number of considered subproblems'
        nsubprob = res.solver.statistics.branch_and_bound[key]
        out += 'Handled nodes: \t {}\n'.format(nsubprob)

        out += 'Constraints:\n'
        hdlr_list = Stats.get_initial_hdlr_constraints()
        for (hdlr, nconss) in hdlr_list:
            out += '  {} \t {}\n'.format(nconss, hdlr)

        out += 'Fake varbounds:  {}\n'.format(fake_bounds)
        out += '--------------------------------------------------\n\n'
        return out

    def _read_bounds(self, instance_name):
        upper = float('inf')
        lower = float('-inf')
        file = open(self._sol_file, 'r')
        lines = []
        for line in file:
            if instance_name in line:
                lines.append(line)
        file.close()
        if len(lines) == 0:
            raise ValueError('Instance {} not found in solu file.'
                             ''.format(instance_name))
        for line in lines:
            if '=opt=' in line:
                val = float(line.split(' ')[-1])
                upper = val
                lower = val
                break
            elif '=inf=' in line:
                upper = 'infeasible'
                lower = 'infeasible'
                break
            elif '=best=' in line:
                val = float(line.split(' ')[-1])
                upper = val
            elif '=bestdual=' in line:
                val = float(line.split(' ')[-1])
                lower = val
        return lower, upper

    def _create_pyomo_instance(self, problem):
        """Generates a Pyomo instance from the given problem.
        :param problem: osil_parser.quadraticProblem object.
        :return: A Pyomo ConcreteModel.
        """
        # Create an abstract model for a MIQCQP with linear objective
        model = ConcreteModel()

        # Sets
        model.C = Set(initialize=self._create_name_set(problem.constraints))
        model.V = Set(initialize=self._create_name_set(problem.variables))

        # Constraint parameters
        model.A = Param(model.C, model.V, model.V, default=0,
                        initialize=self._create_A(problem))
        model.b = Param(model.C, model.V, default=0,
                        initialize=self._create_b(problem))

        # Constraint bounds
        cl, cu = self._create_bounds(problem.constraints)
        model.cl = Param(model.C, initialize=cl)
        model.cu = Param(model.C, initialize=cu)

        # Variable bounds
        xl, xu = self._create_bounds(problem.variables)
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
                        initialize=self._create_z(problem, obj_min))

        # Objective
        def obj_rule(model):
            return sum(model.z[k] * model.X[k] for k in model.V)

        model.Obj = Objective(rule=obj_rule, sense=minimize)

        return model

    # Helper functions for creating the Pyomo model.

    def _create_name_set(self, list):
        s = []
        for obj in list:
            s.append(obj.name)
        return s

    def _create_A(self, problem):
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

    def _create_b(self, problem):
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

    def _create_c(self, problem):
        return {}

    def _create_bounds(self, list):
        l = {}
        u = {}
        for obj in list:
            l[obj.name] = obj.lb
            u[obj.name] = obj.ub
        return l, u

    def _create_z(self, problem, is_min):
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
