# This module is the interface module for the user.
# It checks the input format and passes the information to the hub.


from pyomo.environ import *

from pyminlp.conshdlr import ConsHandlerManager
from pyminlp.hub import Coordinator
from pyminlp.hub import SolvingStage
from pyminlp.hub import SolvingStageError
from pyminlp.hub import UserInputError


class PyMINLP:
    """
    This is the interface class of the PyMINLP solver.

    The class offers functions for setting up the solver at the
    beginning as well as interface functions for the plugins to
    manipulate the solving process.

    Each function has a certain, well-defined effect on the solving
    process, which may also depend on the stage of solving.

    Some functions may only be called at certain stages. Additionally,
    they may require a certain input format.

    Private class attributes:
        _coordinator    Reference to the coordinator object that is
                            responsible for processing the user input.
        _handlers       TODO generate plugin structure.
    """

    def __init__(self):
        """Constructor function setting up the attributes."""
        self._coordinator = Coordinator.create()
        # TODO implement this as a plugin.
        self._known_hdlrs = []
        self._known_hdlrs.append(ConsHandlerManager.create('linear'))
        self._known_hdlrs.append(ConsHandlerManager.create('quadconv'))
        self._known_hdlrs.append(ConsHandlerManager.create('quadnonc'))
        for hdlr in self._known_hdlrs:
            # TODO find a better way for that.
            hdlr.solver = self

    # Functions for the set up.

    def set_relaxation_solver(self, relaxation_solver):
        """Registers a relaxation solver for the solving process.

        The function is designed for solver objects from the
        SolverFactory. It can, however, process any solver object that
        provides a solve(ConcreteModel) function that solves a Pyomo
        ConcreteModel. You can modify the solver's settings before
        setting it as relaxation solver if you wish to.

        Please make sure that the relaxation solver is able to handle
        all constraint types that are desired to be part of the
        relaxation as well as the objective function type.

        Preconditions: Solving stage needs to be SETUP.

        :param relaxation_solver: A suitable solver object.
        """
        # Check if relaxation solver has a solve function.
        solve_fct = getattr(relaxation_solver, 'solve', None)
        if not callable(solve_fct):
            raise ValueError('Given relaxation solver does not have a '
                             'solve(model) function for solving '
                             'relaxations.')
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SETUP:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. Expected SETUP.'.format(stage))
        # Call the internal function.
        self._coordinator.register_relaxation_solver(relaxation_solver)

    def set_epsilon(self, gap_epsilon=None, constraint_epsilon=None):
        """Registers the epsilon values for the solving process. Per
        default, both values are set to zero.

        Preconditions: Solving stage needs to be SETUP.

        :param gap_epsilon: When lower and upper bound of the optimal
        objective function value of a (sub-)problem differ in less than
        the gap epsilon, the (sub-)problem is considered solved.
        :param constraint_epsilon: The tolerance applied to constraint
        bounds when determining whether a constraint is met or not.
        """
        # TODO maybe check format of epsilon values.
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SETUP:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. Expected SETUP.'.format(stage))
        # Call the internal function.
        if gap_epsilon is not None:
            self._coordinator.set_gap_epsilon(gap_epsilon)
        if constraint_epsilon is not None:
            self._coordinator.set_cons_epsilon(constraint_epsilon)

    def set_verbosity(self, verbosity):
        """Sets the verbosity level for the solving process, i.e. the
        detail level of the solvers output. Per default, the level is 2.

        1 - No output.
        2 - Timed updates providing general information.
        3 - Print information for any step in the solving process.

        Preconditions: Solving stage needs to be SETUP.

        :param verbosity: int indicating the verbosity level.
        """
        # Check format of the verbosity attribute.
        if not isinstance(verbosity, int):
            raise ValueError('The given verbosity is not an int.')
        if not (verbosity == 1 or verbosity == 2 or verbosity == 3):
            raise ValueError('The given verbosity level {} not defined.'
                             ''.format(verbosity))
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SETUP:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. Expected SETUP.'.format(stage))
        # Call the internal function.
        self._coordinator.set_verbosity(verbosity)

    def use_constraint_handler(self, name, constypes, identify_prio,
                               enforce_prio=None, relaxation=False):
        """Integrates a constraint handler into the solving process.

        The constraint types are the names of the constraint objects
        in the Pyomo model. If no enforce_prio is given, the
        identify_prio is applied instead. If no relaxation flag is
        given, it is assumed to be False.

        Preconditions: Solving stage needs to be SETUP.

        :param name: The user defined name of the constraint handler
        (string).
        :param constypes: A list of constraint types (strings) the
        the constraint handler should consider.
        :param identify_prio: The priority for the identify methods
        (int). A constraint handler with a lower priority will be
        called first.
        If the first-called handler identifies a
        constraint, the successing constraint handlers can not identify
        this constraint anymore.
        :param enforce_prio: The priority for the prepare and
        enforce methods (int). A constraint handler with a lower
        priority will be called first.
        If the first-called handler calls an interface function, this
        may influence the successing constraint handlers in preparation
        and enforcement. For details, see descriptions of the interface
        functions.
        :param relaxation: Flag determining if all of the constraints
        belonging to this handler should be part of the relaxations
        (True) or not (False).
        """
        # Check input format.
        if not isinstance(name, str):
            raise ValueError('The given name is not a string.')
        if not isinstance(constypes, list):
            raise ValueError('The given constypes parameter is not a list.')
        if len(constypes) == 0:
            raise ValueError('The list of constypes is empty.')
        for type in constypes:
            if not isinstance(type, str):
                raise ValueError('The constype {} is not a string.'
                                 ''.format(type))
        if not isinstance(relaxation, bool):
            raise ValueError('The given relax parameter is not a boolean.')
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SETUP:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. Expected SETUP.'.format(stage))
        # Find handler and add.
        # TODO different once this is a plugin.
        for hdlr in self._known_hdlrs:
            if hdlr.name() == name:
                hdlr.set_relax(relaxation)
                hdlr.add_constypes(constypes)
                if enforce_prio is None:
                    hdlr.set_prio(identify_prio, identify_prio)
                else:
                    hdlr.set_prio(identify_prio, enforce_prio)
                self._coordinator.add_conshandler(hdlr)
                break

    # Functions for solving.

    def solve(self, py_model):
        """Solves the given instance.

        WARNING: Currently only minimisation problems allowed.

        Preconditions: Solving stage needs to be SETUP. The solver
        setup needs to be done, i.e. at least
            - constraint handlers covering all constraint types are
                  registered and
            - a relaxation solver is set.

        :param py_model: The Pyomo ConcreteModel to be solved.
        """
        if not isinstance(py_model, ConcreteModel):
            raise ValueError('The object to be solved is not a Pyomo '
                             'ConcreteModel.')
        stage = self._coordinator.stage
        if not stage == SolvingStage.SETUP:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. Expected SETUP.'.format(stage))

        return self._coordinator.solve(py_model)

    # Functions for implementing plugins.

    def add_constraints(self, constype, constraints, params={}):
        """Add a new set of constraints and parameters to the current
        subproblem.

        Effects on solving process:
            - In identify: Not permitted.
            - In prepare: No special effect.
            - In enforce: Resolve relaxation after this constraint
            handler has finished its enforce method.

        :param constype: The constraint type (string), i.e. the name of
        the constraint component.
        :param constraints: The set of constraints (Pyomo Constraint
        object).
        :param params: A dictionary of parameters. Key is the desired
        parameter name, content is the Pyomo parameter object. Pass the
        parameters with the constraints to associate them.

        Preconditions:
            - Solving stage needs to be SOLVING.
            - The function may not be used called from the identify
            function.
            - The dimension of the index set of the constraints is 0
            (SimpleConstraint) or 1.
            - If the constraint type already exists, the index set of
            this should be one-dimensional. This is for example the case
            if the respective constraint type has been generated by this
            function.
            - For adding parameters, always use the same index set
            structure for the same parameter names.
            - The index set of a parameter may contain the constraint
            index set 0 or 1 time(s).

        WARNING: It is recommended not to add to constraint and
        parameter types that already exist in the initial model
        definition. It may work in some cases, but this can not be
        guaranteed for any case.

        Exact effect on Pyomo model object:
            - If the constraint type does not exist yet, it will be
            created as an IndexedConstraint object with an underlying
            one-dimensional index set.

            - Each added constraint will get a new, unique
            one-dimensional index within the constraint type.

            - For parameters, that do not contain the index set of the
            given constraints, any of the parameters get an additional
            constraint index at their first position. The same holds if
            the constraints have no index set (SimpleCons).
            (Examples: p becomes p[consindex],
            p[varindex] becomes p[consindex, varindex])

            - For parameters, that contain the index set of the given
            constraints, this parameter index is replaced by the new
            unique constraint index.

            - The new unique constraint indices are saved in a SimpleSet
            called '_{constype}_Set' and added to the internal model.
        """
        if not type(constype) is str:
            raise ValueError('Given constype parameter is not a string.')
        if not isinstance(constraints, Constraint):
            raise ValueError('Given constraints parameter is not a Pyomo '
                             'Constraint object.')
        if not isinstance(params, dict):
            raise ValueError('Given params parameter is not a dictionary.')
        for p in params.keys():
            if not type(p) is str:
                raise ValueError('Given params key {} is not a string.'
                                 ''.format(p))
            if not isinstance(params[p], Param):
                raise ValueError('Given params entry {} is not a Pyomo Param '
                                 'object.'.format(params[p]))
        stage = self._coordinator.stage
        if not stage == SolvingStage.SOLVING:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. '
                                    'Expected SOLVING.'.format(stage))

        self._coordinator.add_constraints(constype, constraints, params)

    def branch(self, variable, point):
        """Branches the current problem into two subproblems. The
        branching strategy is defined by the branching variable and the
        branching point.

        Effects on solving process:
            - In identify: Not permitted.
            - In prepare: The current node will not be further
            considered after this constraint has finished its prepare
            method. A new node will be chosen.
            - In enforce: The current node will not be further
            considered after this constraint has finished its prepare
            method. A new node will be chosen. This effect dominates
            the effects of add_constraints and change_bounds.

        :param variable: The name of the variable (string) or the
        Pyomo Variable object.
        :param point: The branching point that will be the new upper
        bound for the given variable in one subproblem and the new
        lower bound in the other subproblem.
        """
        # Check input format.
        if not isinstance(variable, str):
            try:
                variable = variable.name
                if not isinstance(variable, str):
                    raise ValueError()
            except (AttributeError, ValueError):
                raise ValueError('Given parameter variable is neither a '
                                 'string nor an object with a name attribute.')
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SOLVING:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. '
                                    'Expected SOLVING.'.format(stage))
        # Call the internal function.
        self._coordinator.branch(variable, point)

    def change_bounds(self, variable, lower=None, upper=None):
        """Changes the variable bounds of the desired variable.

        Effects on solving process:
            - In identify: Not permitted.
            - In prepare: No special effect.
            - In enforce: Resolve relaxation after this constraint
            handler has finished its enforce method.

        :param variable: The name of the variable (string) or the
        Pyomo Variable object.
        :param lower: The new lower bound for the variable. If not
        specified or None, the bound will not change.
        :param upper: The new upper bound for the variable. If not
        specified or None, the bound will not change.
        """
        # Check input format.
        if not isinstance(variable, str):
            try:
                variable = variable.name
                if not isinstance(variable, str):
                    raise ValueError()
            except (AttributeError, ValueError):
                raise ValueError('Given parameter variable is neither a '
                                 'string nor an object with a name attribute.')
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SOLVING:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. '
                                    'Expected SOLVING.'.format(stage))
        # Call internal function.
        self._coordinator.change_bounds(variable, lower, upper)

    def declare_infeasible(self):
        """Declares that the current subproblem has no feasible
        solution.

        Effects on solving process:
            - In identify: Not permitted.
            - In prepare: The current node will not be further
            considered after this constraint has finished its prepare
            method. A new node will be chosen. This effect dominates
            the effect of branch, branch calls will be ignored.
            - In enforce: The current node will not be further
            considered after this constraint has finished its prepare
            method. A new node will be chosen. This effect dominates
            the effects of branch, add_constraints and change_bounds,
            i.e. calls of these functions will be ignored.
        """
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SOLVING:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. '
                                    'Expected SOLVING.'.format(stage))

        self._coordinator.declare_infeasible()

    def match(self, constraint):
        """Declares that the given constraint belongs to the calling
        constraint handler.

        Effects on solving process:
            - In identify: The constraint will can not be assigned to
            any subsequently called constraint handler anymore.
            - In prepare: Not permitted.
            - In enforce: Not permitted.

        :param constraint: The name of the constraint (string) or the
        Pyomo Constraint object."""
        # Check input format.
        if not isinstance(constraint, str):
            try:
                constraint = constraint.name
                if not isinstance(constraint, str):
                    raise ValueError()
            except (AttributeError, ValueError):
                raise ValueError('Given parameter constraint is neither a '
                                 'string nor an object with a name attribute.')
        # Check if the stage is correct.
        stage = self._coordinator.stage
        if not stage == SolvingStage.SOLVING:
            raise SolvingStageError('Can not call this function in solving '
                                    'stage {}. '
                                    'Expected SOLVING.'.format(stage))
        # Call internal function.
        self._coordinator.match(constraint)
