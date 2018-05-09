# Interface for the subproblem representation.


import copy

# TODO more precisely
from pyomo.environ import *

from pyminlp.conshdlr import ConsHandler


class Instance:
    """
    An object of this class represents an optimisation problem instance.
    The objects are used to represent nodes in the branch and bound
    tree. Information such as constraint classification data are
    available. The class is the interface for everything related to
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
        _n_unclassif     The number of internal constraint objects that
                         do not have a constraint classifier assigned.
        _consadd_count   Counts how often the add_constraints(...)
                         function is called (static).
    """

    @classmethod
    def create_instance(cls, py_model):
        """Factory method. Generates and returns a new internal
        instance from a Pyomo concrete model object.

        :param py_model: A ConcreteModel containing an instance from
        Pyomo.
        :return: An new internal Instance containing all necessary data.
        """
        assert isinstance(py_model, Model)
        inst = Instance()
        # TODO standardise all constraints to <= 0.. maybe offer option
        # TODO to turn this off
        #
        # TODO maybe no deepcopy for space efficiency
        inst._init(py_model)
        return inst

    @classmethod
    def derive_instance(cls, instance):
        """Factory method. Derives a new instance from an existing
        instance. Deepcopies all data.

        :param instance: An existing internal Instance object.
        :return: A new internal Instance object containing the same
        data.
        """
        assert type(instance) is Instance
        new_inst = instance._clone()
        return new_inst

    # Static counter for add_constraints() function.
    _consadd_count = 0

    def __init__(self):
        """Declares all attributes of an instance. See above for
        attribute description."""
        self._model = None
        self._consmap = {}
        self._classif = {}
        self._n_unclassif = 0

    def _init(self, model):
        """Derives all attribute values from a given Pyomo model.
        :param model: A Pyomo ConcreteModel representing an instance.
        """
        self._model = model
        for ct in model.component_objects(ctype=Constraint):
            # Create _Cons objects and _consmap
            for c in ct:
                # TODO there may be a problem with object identities if
                # TODO pyrep reference is used
                py_cons = ct[c]
                in_cons = _Cons.create(pyrep=py_cons, name=py_cons.name,
                                       constype=ct.name, index=c)
                self._consmap[py_cons.name] = in_cons
                self._n_unclassif += 1

    def _clone(self):
        """Clone function for an internal Instance object.
        :return: A new internal Instance with copied data.
        """
        inst = Instance()
        inst._model = self._model.clone()
        # TODO how deep are the copies
        inst._consmap = copy.copy(self._consmap)
        inst._classif = copy.copy(self._classif)
        inst._n_unclassif = self._n_unclassif
        return inst


    def model(self):
        """Returns a Pyomo representation of the optimisation problem
        represented by Instance object.
        :return: A Pyomo ConcreteObject.
        """
        # TODO maybe only return a copy
        return self._model

    def relaxation(self):
        """Returns a Pyomo representation of the relaxation of the
        optimisation problem represented by this Instance object.
        :return: A Pyomo ConcreteObject.
        """
        # TODO implement
        pass

    def get_unclassified(self, constype=None):
        """Returns the index set of all yet unclassified constraints,
        i.e. all constraints that do not belong to a constraint handler
        yet.
        :param constype: A string indicating the desired user defined
        constraint type. If unspecified or None, all constraint types
        are taken into consideration.
        :return: A list of indices.
        """
        assert type(constype) is str
        set = []
        for c in self._consmap:
            cons = self._consmap[c]
            if constype is None or constype == cons.constype:
                if cons.conshdlr is None:
                    set.append(cons.index)
        return set


    def add_constraints(self, constype, constraints, params={}):
        assert type(constype) is str
        assert type(params) is dict
        # TODO catch all cases of bad use
        # Check if parameters and constraints already exist
        component = self._model.component(constype)
        if component is None:
            # Add set, parameter, constraint objects.
            self._model.add_component('_{}_Set'.format(constype), Set())
            set = self._model.component('_{}_Set'.format(constype))
            self._model.add_component(constype, Constraint(set))
            for p in params.keys():
                new_index_set = params[p].index_set()
                if not new_index_set == {None} \
                       and not constraints.index_set() is new_index_set:
                    # TODO handle this case
                    if new_index_set.dimen == 1:
                        new_index_set = [set, new_index_set]
                    elif constraints.index_set() in new_index_set.set_tuple:
                        new_index_set = list(new_index_set.set_tuple)
                        new_index_set[new_index_set.index(constraints.index_set())] = set
                    # TODO handle this case
                    else:
                        new_index_set = list(new_index_set.set_tuple)
                        new_index_set.insert(0, set)
                    self._model.add_component(p, Param(*new_index_set,
                                                       mutable=True,
                                                       default=params[
                                                           p].default()))
                else:
                    self._model.add_component(p, Param(set, mutable=True,
                                                 default=params[p].default()))
            component = self._model.component(constype)

        # Find the underlying set.
        # TODO what happens for multiple index sets?
        # Assumption: Maximal one index for new consraints. Every param
        # has zero or one indices. Multiple indices are serialized.
        set = component.index_set()
        # Iterate over all elements of the given set to access all
        # constraints.
        for el in constraints:
            cons = constraints[el]
            # Add internal constraint representation.
            int_cons = _Cons()
            int_cons.pyrep = cons
            int_cons.repclass = constype
            int_cons.index = '_{}_{}'.format(Instance._consadd_count, el)
            self._n_unclassif += 1
            # Add constraint to instance.
            set.add(int_cons.index)
            component.add(int_cons.index, cons.expr)

        # Deal with parameters.
        for p in params.keys():
            new_param = self._model.component(p)
            cur_param = params[p]
            if params[p].index_set() == {None}:
                for el in set:
                    new_param[el] = cur_param.value
            else:
                new_par = []
                for el in constraints:
                    for key in cur_param:
                        if cur_param[key] == cur_param.default():
                            continue
                        # replace by exact position statement
                        if el == key:
                            new_key = '_{}_{}'.format(Instance._consadd_count,
                                                      el)
                            new_param[new_key] = cur_param[key]
                        elif el in key:
                            new_key = list(copy.deepcopy(key))
                            new_key[key.index(el)] = \
                                '_{}_{}'.format(Instance._consadd_count, el)
                            new_param[tuple(new_key)] = cur_param[key]

        Instance._consadd_count += 1

    def change_varbounds(self, varname, lower_bound=None, upper_bound=None):
        """Changes the bounds of a variable.
        :param varname: The name of the variable (string).
        :param lower_bound: The new lower bound. If unspecified or None,
        no changes are applied.
        :param upper_bound: The new upper bound. If unspecified or None,
        no changes are applied.
        """
        assert type(varname) is str
        # Find the correct variable.
        # TODO do it smarter or put it into a seperated function.
        for vartype in self._model.component_objects(Var):
            for v in vartype:
                if vartype[v].name == varname:
                    var = vartype[v]
                    break
        # Manipulate the bounds.
        if lower_bound is not None:
            var.setlb(lower_bound)
        if upper_bound is not None:
            var.setub(upper_bound)

    def register_conshandler(self, constraint, conshdlr):
        """Assigns a constraint to a constraint handler.
        :param cons: The name of the constraint (string).
        :param conshdlr: The constraint handler object.
        """
        in_cons = self._consmap[constraint]
        if in_cons.conshdlr is None:
            self._n_unclassif -= 1
        in_cons.set_conshdlr(conshdlr)
        if conshdlr.name() in self._classif.keys():
            self._classif[conshdlr.name()].append(in_cons)
        else:
            self._classif[conshdlr.name()] = [in_cons]



class _Cons:
    """
    This internal class stores additional constraint data. that are not
    represented in the Pyomo model.

    Use the factory method _Cons.create(...) for generating a new
    instance.

    Public class attributes:
        id        A unique identifier for the constraint representation.
        pyrep     A Pyomo object representing the constraint.
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
        """Factory method to generate a new constraint. Parameters as
        described above.
        """
        # TODO maybe catch cases of bad usage, i.e. type checks for
        # TODO parameters.
        assert type(name) is str
        assert type(constype) is str
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
        """Manipulate the constraint's constraint handler."""
        assert isinstance(conshdlr, ConsHandler)
        self._conshdlr = conshdlr
