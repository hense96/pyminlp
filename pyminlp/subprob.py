

import copy

from pyomo.environ import *
from pyomo.opt import SolverStatus, TerminationCondition

from pyminlp.conshdlr import ConsHandlerManager

from pyomo.core.base.var import _GeneralVarData


class Instance:
    """
    An object of this class represents an optimisation problem instance.
    The objects are used to represent nodes in the branch and bound
    tree. Additional information such as constraint classification data
    are available. The class is the interface for everything related to
    (sub-)problem representation.

    Private class attributes:
        _model       A Pyomo representation of the optimisation problem.
        _consmap     Dictionary mapping Pyomo constraints to the
                         internal constraint representation. The key of
                         the dictionary is the constraint's name
                         (string).
        _classif     Dictionary mapping all used constraint handlers to
                         their constraint lists (list of internal
                         constraint representation). The key of the
                         dictionary is the constraint handler name.
        _consadd_count   Counts how often the add_constraints(...)
                         function is called (static).
        _relax       A _Solution object representation of the solved
                         relaxation of the optimisation problem. If
                         None, the relaxation of the current Instance
                         object is not solved. Note that this will also
                         be the case whenever the Instance object gets
                         changed.
    """

    @classmethod
    def create_instance(cls, py_model):
        """Factory method. Generates and returns a new internal
        Instance from a Pyomo ConcreteModel object. The given model is
        used in the Instance without being copied.

        :param py_model: A ConcreteModel containing an instance from
        Pyomo.
        :return: An new internal Instance containing all necessary data.
        """
        assert isinstance(py_model, Model)
        if not py_model.is_constructed():
            raise ValueError('The Pyomo model {} is not '
                             'constructed.'.format(py_model))
        inst = Instance()
        inst._init(py_model)
        return inst

    @classmethod
    def derive_instance(cls, instance):
        """Factory method. Derives a new instance from an existing
        instance. Deepcopies the model.

        Precondition: all constraints of the given instance are
        classified, i.e. they hava an assigned constraint handler.

        :param instance: An existing internal Instance object.
        :return: A new internal Instance object containing the same
        data.
        """
        assert type(instance) is Instance
        if instance.nunclassified() > 0:
            raise ValueError('The instance {} has unclassified constraints and'
                             ' can thus not be cloned.'.format(instance))
        # TODO maybe no model deepcopy for space efficiency.
        new_inst = instance._clone()
        return new_inst

    # Static counter for add_constraints() function.
    _consadd_count = 0

    def __init__(self):
        """Declares all attributes of an instance. See above for
        attribute description.
        """
        self._model = None
        self._consmap = {}
        self._classif = {}
        self._relax = None

    def _init(self, model):
        """Derives all attribute values from a given Pyomo model.
        :param model: A Pyomo ConcreteModel representing an instance.
        """
        self._model = model
        for ct in model.component_objects(ctype=Constraint):
            # Create _Cons objects and _consmap
            for c in ct:
                self._create_constraint(constraint=ct[c], constype=ct.name,
                                        index=c)

    def _clone(self):
        """Clone function for an internal Instance object. The
        relaxation object does not get cloned.
        :return: A new internal Instance with copied data.
        """
        inst = Instance()
        inst._model = self._model.clone()
        inst._consmap = copy.copy(self._consmap)
        # Copy _classif attribute to the desired depth.
        inst._classif = {}
        for key in self._classif.keys():
            inst._classif[key] = copy.copy(self._classif[key])
        return inst

    # Functions for providing data.

    def model(self):
        """Returns a Pyomo representation of the optimisation problem
        represented by Instance object.
        :return: A Pyomo ConcreteModel.
        """
        # TODO maybe only return copies.
        return self._model

    def relax_model(self):
        """Returns a Pyomo representation of the solved relaxation of
        the instance.  It contains the same constraints (non-relaxation)
        constraints are deactivated as well as solution data.
        Precondition: The relaxation of the instance is solved.
        :return: A Pyomo ConcreteModel with solution data.
        """
        assert self.relaxation_solved()
        return self._relax.model

    def relaxation_solved(self):
        """Returns True if the relaxation of the current Instance object
        has already been solved. If a manipulating function is called
        (i.e. add_constraints, change_varbounds) after solving the
        relaxation, this function will return False.
        :return: True if relaxation is solved, otherwise False.
        """
        return self._relax is not None

    def constypes(self):
        """Returns all constraint types in the model instance.
        :return: A list of strings representing constraint types."""
        list = []
        for cons in self._model.component_objects(Constraint):
            list.append(cons.name)
        return list

    def conshandler_sets(self, conshandler):
        """Returns the sets of indices of the constraints that
        belong to the given constraint handler.
        The sets (represented by lists) are saved as the values of a
        dict, where the keys is the respective constraint type of the
        constraint.
        If the constraint handler considers a constraint type, for
        which no constraint exists, an empty list will be in the return
        dict.
        If a violated constraint is a SimpleConstraint without index,
        the only index will be None.

        Example for a return value:
            {'Constype1':['index1, index2'], 'Constype2':[]}

        :param conshandler: A ConsHandlerManager object.
        :return: A dict.
        """
        assert isinstance(conshandler, ConsHandlerManager)
        if conshandler.name() in self._classif.keys():
            conss = self._classif[conshandler.name()]
        else:
            conss = []
        sets = {}
        for constype in conshandler.constypes():
            sets[constype] = []
        for cons in conss:
            sets[cons.constype].append(cons.index)
        return sets

    def unclassified(self, constype=None):
        """Returns the index set of all yet unclassified constraints,
        i.e. all constraints that do not belong to a constraint handler
        yet.
        :param constype: A string indicating the desired user defined
        constraint type. If unspecified or None, all constraint types
        are taken into consideration.
        :return: A list of indices. For a SimpleConstraint without
        index, the index will be None.
        """
        assert constype is None or type(constype) is str
        index_set = []
        for c in self._consmap:
            cons = self._consmap[c]
            if constype is None or constype == cons.constype:
                if cons.conshdlr is None:
                    index_set.append(cons.index)
        return index_set

    def nconstraints(self):
        """Returns the number of constraints this instance has.
        :return: Number of constraints (int).
        """
        return len(self._consmap.keys())

    def nunclassified(self):
        """Returns the number of constraints without constraint handler.
        :return: Number of unclassified constraints (int).
        """
        return len(self.unclassified())

    # Interface functions for solving process modules.

    def feasible(self):
        """Returns True if the solution of the relaxation of the
        instance is feasible for the instance.
        Precondition: The relaxation of the instance is solved and
        has an optimal solution.
        :return: True or False.
        """
        assert self.relaxation_solved()
        assert self.has_optimal_solution()
        return self._relax.nviolated == 0

    def has_optimal_solution(self):
        """Returns True if the relaxation of the instance has an optimal
        solution, i.e. the relaxation is neither infeasible nor
        unbounded.
        Precondition: The relaxation of the instance is solved.
        :return: True or False.
        """
        assert self.relaxation_solved()
        return self._relax.termination_condition\
            == TerminationCondition.optimal

    def relax_infeasible(self):
        """Returns True if the relaxation of the instance is infeasible.
        Precondition: The relaxation of the instance is solved.
        :return: True or False
        """
        assert self.relaxation_solved()
        return self._relax.termination_condition\
            == TerminationCondition.infeasible

    def relax_termination_condition(self):
        """Returns the TerminationCondition of the relaxation solver."""
        assert self.relaxation_solved()
        return self._relax.termination_condition

    def objective_function_value(self):
        """Returns the optimal objective function value of the solved
        relaxation of the instance.
        Precondition: The relaxation of the instance is solved and
        has an optimal solution.
        :return: An objective function value.
        """
        assert self.relaxation_solved()
        assert self.has_optimal_solution()
        obj = self._relax.model.component_objects(Objective)
        for o in obj:
            return value(o)

    def violated_sets(self, conshandler):
        """Returns the sets of indices of the constraints that are
        violated by the solution of the relaxation of the instance.
        The sets (represented by lists) are saved as the values of a
        dict, where the keys is the respective constraint type of the
        constraint.
        Only constraints that belong to the given constraint handler are
        considered. If the constraint handler considers a constraint
        type, for which no constraint is violated, an empty list will be
        in the return dict.
        If a violated constraint is a SimpleConstraint without index,
        the only index will be None.

        Example for a return value:
            {'Constype1':['index1, index2'], 'Constype2':[]}

        Precondition: The relaxation of the instance is solved and has
        optimal solution.

        :param conshandler: A ConsHandlerManager object indicating
        which constraints to consider.
        :return: A dict.
        """
        assert isinstance(conshandler, ConsHandlerManager)
        assert self.relaxation_solved()
        assert self.has_optimal_solution()
        sets = {}
        for constype in conshandler.constypes():
            sets[constype] = []
        if conshandler.name() not in self._relax.violated.keys():
            return sets
        else:
            conss = self._relax.violated[conshandler.name()]
            for cons in conss:
                sets[cons.constype].append(cons.index)
        return sets

    def nviolated(self, conshandler):
        """Returns the number of violated constraints that are assigned
        to the given constraint handler.
        Precondition: The relaxation of the instance is solved.
        :param conshandler: A constraint handler object.
        :return: A number (int).
        """
        assert isinstance(conshandler, ConsHandlerManager)
        assert self.relaxation_solved()
        assert self.has_optimal_solution()
        if conshandler.name() not in self._relax.violated.keys():
            return 0
        else:
            return len(self._relax.violated[conshandler.name()])

    def solve_relaxation(self, solver, epsilon=None):
        """Creates and solves the relaxation of the instance. Moreover,
        computes additional data related to the relaxation solution,
        such as the sets of violated constraints.
        :param solver: The Pyomo relaxation solver.
        :param epsilon: The epsilon for determining whether a
        constraint is violated.
        """
        assert self.nunclassified() == 0
        # Firstly, create the relaxation.
        relax = self._model.clone()
        nonrel_cons = []
        # Deactivate non-relaxation constraints and remember them.
        for c in self._consmap.keys():
            cons = self._consmap[c]
            py_cons = relax.component(cons.constype)[cons.index]
            if cons.conshdlr.relax():
                py_cons.activate()
            else:
                py_cons.deactivate()
                nonrel_cons.append((cons, py_cons))
        relax.preprocess()
        # Secondly, solve the relaxation.
        res = solver.solve(relax)
        if res.solver.status != SolverStatus.ok:
            raise PyomoException('Relaxation solver status after solving is '
                                 '{}. Expected ok.'.format(res.solver.status))
        term_cond = res.solver.termination_condition
        self._relax = _Solution.create(model=relax,
                                       termination_condition=term_cond)
        # If there is a solution, find the violated constraints.
        if term_cond == TerminationCondition.optimal:
            # Firstly, postprocess relaxation.
            # If there is any variable without value, assign randomly.
            # This also works for simple variables.
            for vartype in relax.component_objects(Var):
                for v in vartype:
                    if vartype[v].value is None:
                        if vartype[v].lb is not None:
                            vartype[v].value = vartype[v].lb
                        elif vartype[v].ub is not None:
                            vartype[v].value = vartype[v].ub
                        else:
                            raise ValueError('Variable {} has no bounds.'
                                             ''.format(vartype[v]))
                            # vartype[v].value = 0
            # Now iterate of all constraints.
            for (cons, py_cons) in nonrel_cons:
                # Catch weird special case.
                if py_cons.upper is None and py_cons.lower is None:
                    continue
                # Check whether a constraint is within the tolerance.
                if epsilon is None:
                    if not value(py_cons.get_value()):
                        self._relax.add_violated(cons)
                elif (py_cons.lower is not None
                       and not py_cons.lower - epsilon <= value(py_cons.body))\
                   or (py_cons.upper is not None
                       and not value(py_cons.body) <= py_cons.upper + epsilon):
                    # TODO maybe consider special cases such as strict
                    # TODO constraints or non-numerical bounds?
                    self._relax.add_violated(cons)

    # Interface functions for constraint handler plugins.

    def add_constraints(self, constype, constraints, params={}):
        """Add a new set of constraints and parameters to the internal
        problem representation. Deletes any relaxation solution data.
        TODO try to eliminate this warning.
        WARNING: It is recommended not to add to constraint and
        parameter types that already exist in the initial model
        definition. It may work in some cases, but this can not be
        guaranteed for any case so far.

        :param constype: The constraint type (string), i.e. the name of
        the constraint component.
        :param constraints: The set of constraints (Pyomo constraint
        object).
        :param params: Dictionary of parameters. Key is the desired
        parameter name, content is the Pyomo parameter object.

        Preconditions:
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

        Exact behaviour:
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
        assert type(constype) is str
        assert isinstance(constraints, Constraint)
        assert type(params) is dict
        # TODO catch all cases of bad use
        if constraints.index_set() is not {None}\
            and hasattr(constraints.index_set(), 'set_tuple'):
            raise ValueError('Index set of added constraints {} has a '
                             'dimension greater than 1.'.format(constype))
        # Check if parameters and constraints already exist.
        component = self._model.component(constype)
        if component is None:
            # Sanity check: constraint index set appears at most once
            # in any parameter index set.
            if constraints.index_set() is not {None}:
                cons_set = constraints.index_set()
                for p in params:
                    param_index = params[p].index_set()
                    if param_index is not {None}\
                        and hasattr(param_index, 'set_tuple'):
                        param_index = param_index.set_tuple
                        if param_index.count(cons_set) > 1:
                            raise ValueError('Parameter {} contains the index'
                                             ' set {} of the added constraints'
                                             ' more than once.'.format(p,
                                                                cons_set.name))
            # Add components.
            self._add_components(constype, constraints.index_set(), params)
            component = self._model.component(constype)

        # Read the underlying set.
        # Assumption: Maximal one index for new consraints. Every param
        # has zero or one indices. Multiple indices are serialized.
        # TODO what happens if this is {None}?
        set = component.index_set()

        # Add the new constraints.
        for el in constraints:
            cons = constraints[el]
            index = '_{}_{}'.format(Instance._consadd_count, el)
            # Add constraint to instance.
            set.add(index)
            # In the constraint expression, Replace the variable objects
            # of the user copy of the model by the actual variables.
            expr = self._transform_expr(cons.expr)
            component.add(index, expr)
            new_cons = component[index]
            # Add internal constraint representation.
            self._create_constraint(constraint=new_cons,
                                    constype=component.name, index=index)

        # Deal with parameters.
        for p in params.keys():
            # Sanity check. Do the index sets of the added parameters
            # match the index sets of the existing parameters.
            add_index = params[p].index_set()
            if add_index == {None}:
                add_index = []
            elif hasattr(add_index, 'set_tuple'):
                add_index = add_index.set_tuple
            else:
                add_index = [add_index]
            ex_index = self.model().component(p).index_set()
            if ex_index == {None}:
                ex_index = []
            elif hasattr(ex_index, 'set_tuple'):
                ex_index = ex_index.set_tuple
            else:
                ex_index = [ex_index]
            cons_set_count = add_index.count(constraints.index_set())
            if cons_set_count == 0:
                try:
                    if len(add_index) + 1 != len(ex_index):
                        raise ValueError
                    for i in range(len(add_index)):
                        if add_index[i] != ex_index[i+1]:
                            raise ValueError
                except ValueError:
                    raise ValueError('The index set of the parameter '
                                     '{} does not match the existing'
                                     'parameters with this name.'.format(p))
            elif cons_set_count == 1:
                try:
                    if len(add_index) != len(ex_index):
                        raise ValueError
                    for i in range(len(add_index)):
                        if add_index[i] != ex_index[i]\
                           and add_index[i] != constraints.index_set():
                            raise ValueError
                except ValueError:
                    raise ValueError('The index set of the parameter '
                                     '{} does not match the existing'
                                     'parameters with this name.'.format(p))
            else:
                raise ValueError('Parameter {} contains the index set '
                                 '{} of the added constraints more than'
                                 ' once.'.format(p, cons_set.name))
            # Now, add the parameter values.
            new_param = self._model.component(p)
            given_param = params[p]
            # Case: single given parameter. Add parameter for every
            # index of the new constraints.
            if given_param.index_set() == {None}:
                for el in set:
                    new_param[el] = given_param.value
            # Case: Single parameter with other indices.
            elif (not hasattr(given_param.index_set(), 'set_tuple')
                      and constraints.index_set()
                      != given_param.index_set()) \
                  or (hasattr(given_param.index_set(), 'set_tuple') and
                      constraints.index_set()
                      not in given_param.index_set().set_tuple):
                for key in given_param:
                    # If parameter has default, nothing id to do.
                    if given_param[key] == given_param.default():
                        continue
                    for cons_key in constraints:
                        index = '_{}_{}'.format(Instance._consadd_count,
                                        cons_key)
                        # Write to new parameter with replaced set.
                        new_key = list([key]) if type(key) is str \
                            else list(copy.deepcopy(key))
                        new_key.insert(0, index)
                        new_param[tuple(new_key)] = given_param[key]
            # Case: constraint index set is in parameter index set.
            else:
                # Iterate over all constraint and parameter indices.
                for key in given_param:
                    # If parameter has default, nothing id to do.
                    if given_param[key] == given_param.default():
                        continue
                    for cons_key in constraints:
                        index = '_{}_{}'.format(Instance._consadd_count,
                                        cons_key)
                        # Write to new parameter with replaced set.
                        # Handle different cases.
                        if cons_key == key:
                            new_param[index] = given_param[key]
                        elif type(key) is tuple and cons_key in key:
                            new_key = list(copy.deepcopy(key))
                            new_key[key.index(cons_key)] = index
                            new_param[tuple(new_key)] = given_param[key]

        # Update other affected attributes.
        self._relax = None
        Instance._consadd_count += 1

    def change_varbounds(self, varname, lower_bound=None, upper_bound=None):
        """Changes the bounds of a variable. Deletes any relaxation
        solution data.
        :param varname: The name of the variable (string).
        :param lower_bound: The new lower bound. If unspecified or None,
        no changes are applied.
        :param upper_bound: The new upper bound. If unspecified or None,
        no changes are applied.
        """
        assert type(varname) is str
        # Find the correct variable.
        var = self._find_var(varname)
        if var is None:
            raise KeyError('Variable {} does not exist in Instance'
                           ' {}'.format(varname, self))
        # Manipulate the bounds.
        if lower_bound is not None:
            var.setlb(lower_bound)
        if upper_bound is not None:
            var.setub(upper_bound)
        # Update other affected attributes.
        self._relax = None

    def register_conshandler(self, constraint, conshdlr):
        """Assigns a constraint to a constraint handler.
        One may only assign a constraint to a constraint handler once.
        :param constraint: The name of the constraint (string).
        :param conshdlr: The constraint handler object.
        """
        assert type(constraint) is str
        assert isinstance(conshdlr, ConsHandlerManager)
        try:
            in_cons = self._consmap[constraint]
        except KeyError:
            raise ValueError('Constraint {} does not exist in'
                             ' Instance {}'.format(constraint, self))
        in_cons.set_conshdlr(conshdlr)
        if conshdlr.name() in self._classif.keys():
            self._classif[conshdlr.name()].append(in_cons)
        else:
            self._classif[conshdlr.name()] = [in_cons]

    # Miscellaneous functions.

    def _add_components(self, constype, cons_index_set, params):
        """Adds the constype and the parameter types to the internal
        Pyomo model. Creates a new set that works as an extensible index
        set for adding constraint and parameter objects later.

        :param constype: The constraint type to be added. It must not be
        already part of the Pyomo model.
        :param cons_index_set: The index set of the constraints to be
        added.
        :param params: The dictionary of parameters.
        """
        # Add set and constraint component.
        setname = '_{}_Set'.format(constype)
        self._model.add_component(setname, Set())
        set = self._model.component(setname)
        self._model.add_component(constype, Constraint(set))
        # Add parameters.
        for p in params.keys():
            # Generate the new index set by replacing the constraint
            # set by the new set. Consider different cases...
            new_index_set = params[p].index_set()
            if not new_index_set == {None} \
                    and not cons_index_set is new_index_set:
                if new_index_set.dimen == 1:
                    # Case: the index set has one set that is not the
                    # constraint set.
                    new_index_set = [set, new_index_set]
                elif cons_index_set in new_index_set.set_tuple:
                    # Case: the constraint set is part of the parameter
                    # index set.
                    new_index_set = list(new_index_set.set_tuple)
                    new_index_set[
                        new_index_set.index(cons_index_set)] = set
                else:
                    # Case: the index set has multiple sets, but none of
                    # them is the constraint index set
                    new_index_set = list(new_index_set.set_tuple)
                    new_index_set.insert(0, set)
                self._model.add_component(p, Param(*new_index_set,
                                                   mutable=True,
                                                   default=params[
                                                       p].default()))
            else:
                self._model.add_component(p, Param(set, mutable=True,
                                                   default=params[
                                                       p].default()))

    def _create_constraint(self, constraint, constype, index):
        """Function to generate an internal constraint representation
        and to update the affected data structures, i.e. _consmap.

        :param constraint: The Pyomo constraint.
        :param constype: The constraint type of the constraint (string).
        :param index: The index of the constraint.
        """
        assert type(constype) is str
        in_cons = _Cons.create(pyrep=constraint, name=constraint.name,
                               constype=constype, index=index)
        self._consmap[constraint.name] = in_cons

    def _find_var(self, varname):
        """Returns the variable object of the variable with the given
        variable name.
        :varname: The name of the variable (string).
        :return: A Pyomo variable object, None if there is no match."""
        assert type(varname) is str
        for vartype in self._model.component_objects(Var):
            for v in vartype:
                if vartype[v].name == varname:
                    return vartype[v]
        return None

    def _transform_expr(self, expr):
        """Replaces the variable objects of the given expression by the
        variable objects of the given model. The names of the variables
        in the expression have to appear in the model.
        :param expr: An Expression object.
        :param model: A Pyomo concrete object
        :return: The modified Expression object.
        """
        # TODO This implementation is a workaround. Use proper functions
        # TODO when Pyomo 5.6 is released.
        # TODO It does not even work in any case, probably.
        model = self._model
        # Standard case for expressions.
        if expr._args is not None:
            for i in range(len(expr._args)):
                arg = expr._args[i]
                if hasattr(arg, '_args'):
                    self._transform_expr(arg)
                else:
                    if isinstance(arg, _GeneralVarData):
                        # Find new var object.
                        name, index = arg.name.split('[')
                        index = index[:-1]
                        try:
                            new_var = model.component(name)[index]
                        except KeyError:
                            new_var = model.component(name)[int(index)]
                        finally:
                            expr._args[i] = new_var
            return expr
        # Special cases.
        elif hasattr(expr, '_numerator') and hasattr(expr, '_denominator'):
            for i in range(len(expr._numerator)):
                arg = expr._numerator[i]
                if hasattr(arg, '_args'):
                    self._transform_expr(arg)
                else:
                    if isinstance(arg, _GeneralVarData):
                        # Find new var object.
                        name, index = arg.name.split('[')
                        index = index[:-1]
                        new_var = model.component(name)[index]
                        expr._numerator[i] = new_var
            for i in range(len(expr._denominator)):
                arg = expr._denominator[i]
                if hasattr(arg, '_args'):
                    self._transform_expr(arg)
                else:
                    if isinstance(arg, _GeneralVarData):
                        # Find new var object.
                        name, index = arg.name.split('[')
                        index = index[:-1]
                        new_var = model.component(name)[index]
                        expr._denominator[i] = new_var
            return expr
        else:
            raise Exception('Unknown Expression type. Can not transform'
                            ' expr.')


class _Cons:
    """
    This internal class stores additional constraint data that are not
    represented in the Pyomo model. These data are global for each
    constraint, i.e. they are the same in each Instance object.

    Use the factory method _Cons.create(...) for generating a new
    instance.

    Public class attributes:
        id        A unique identifier for the constraint representation.
        pyrep     A Pyomo object representing the constraint. This is
                      not necessarily a reference to the Pyomo
                      representation in Instance object that uses this
                      _Cons object.
        name      The Pyomo name of the constraint (string). It consists
                      of constype[index].
        constype  The user defined constraint type string.
        index     The set index of the constraint.
        conshdlr  The constraint handler object considering this
                      constraint. None if no constraint handler is
                      registered. So far, only one constraint handler is
                      allowed.

    Private class attributes:
        _counter  A static attribute counting how many _Cons objects
                      have been created.
    """

    @classmethod
    def create(cls, pyrep, name, constype, index):
        """Factory method to generate a new _Cons object. Parameters as
        described above.
        """
        assert type(name) is str
        assert type(constype) is str
        if (index is not None\
            and not '{}[{}]'.format(constype, index) == name)\
            or not name == pyrep.name:
            raise ValueError('Parameter values do not match.')
        cons = _Cons()
        cons._pyrep = pyrep
        cons._name = name
        cons._constype = constype
        cons._index = index
        return cons

    # Counts how many objects of this type are created.
    _counter = 0

    def __init__(self):
        """Constructor setting attributes to None."""
        self._id = _Cons._counter
        _Cons._counter += 1
        self._pyrep = None
        self._name = None
        self._constype = None
        self._index = None
        self._conshdlr = None


    @property
    def id(self):
        """Access the constraint's id."""
        return self._id

    @property
    def pyrep(self):
        """Access the constraint's Pyomo representation.
        Object identity of the Pyomo representation can not be granted.
        """
        return self._pyrep

    @property
    def name(self):
        """Access the constraint's name."""
        return self._name

    @property
    def constype(self):
        """Access the user defined constraint type. It is represented as
        a string.
        """
        return self._constype

    @property
    def index(self):
        """Access the constraint's index, represented as a string."""
        return self._index

    @property
    def conshdlr(self):
        """Access the constraint's constraint handler. None if there is
        no constraint handler registered yet.
        """
        return self._conshdlr

    def set_conshdlr(self, conshdlr):
        """Manipulate the constraint's constraint handler.
        One may only set the constraint handler once.
        """
        assert isinstance(conshdlr, ConsHandlerManager)
        if self._conshdlr is not None:
            ValueError('Constraint {} is already assigned to constraint'
                       ' handler {}.'.format(self.name, self.conshdlr.name()))
        self._conshdlr = conshdlr


class _Solution:
    """
    This internal class stores all relevant data of a relaxation
    solution, i.e. the underlying relaxation model holding solution data
    as well as some additional data.

    Use the factory method _Solution.create(...) for generating a new
    instance.

    Public class attributes:
        model     The Pyomo ConcreteModel representing the relaxation.
                      It holds the solution data.
        termination_condition
                  The termination condition of the solver. It is a
                      Pyomo TerminationCondition object.
        violated  The dictionary of violated constraints. The values are
                      lists of internal _Cons objects, while the key
                      is the names of the respective constraint handler.
        nviolated The total number of violated constraints.
    """

    @classmethod
    def create(cls, model, termination_condition):
        """Factory method to generate a new _Solution object. Parameters
        as described above.
        """
        sol = _Solution()
        sol._model = model
        sol._term_cond = termination_condition
        return sol

    def __init__(self):
        """Constructor setting attributes to None."""
        self._model = None
        self._term_cond = None
        self._violated = {}
        self._nviolated = 0

    @property
    def model(self):
        """Access the Pyomo representation of the solution."""
        return self._model

    @property
    def termination_condition(self):
        """Access the Pyomo representation of the solution."""
        return self._term_cond

    @property
    def violated(self):
        """Returns the dictionary of violated constraints. The values
        are lists of internal _Cons objects, while the key is the names
        of the respective constraint handler. If a constraint handler
        does not have any violated constraints, its name will not
        appear in the dictionary.
        """
        return self._violated

    @property
    def nviolated(self):
        """Access the total number of violated constraints."""
        return self._nviolated

    def add_violated(self, constraint):
        """Add a violated constraint to the list.
        :param constraint: An internal _Cons object.
        """
        assert type(constraint) is _Cons
        key = constraint.conshdlr.name()
        if key in self._violated.keys():
            self._violated[key].append(constraint)
        else:
            self._violated[key] = [constraint]
        self._nviolated += 1


class PyomoException(Exception):
    """This class is an Exception for unexpected values that Pyomo
    returns.
    """
    pass
