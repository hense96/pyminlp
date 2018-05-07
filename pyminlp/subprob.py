# Interface for the subproblem representation.


import copy

# TODO more precise
from pyomo.environ import *


class Instance:
    """An object of this class represents an optimisation problem
    instance. The objects are used to represent nodes in the branch and
    bound tree.
    """

    # Counts how often constraints are added.
    _consadd_count = 0


    @classmethod
    def create_instance(cls, py_model):
        """Factory method. Generates and returns a new internal
        instance from a Pyomo concrete model object.

        :param py_model: A ConcreteModel containing an instance from
        Pyomo.
        :return: An new internal Instance containing all necessary data.
        """
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
        new_inst = instance._clone()
        return new_inst

    def __init__(self):
        """Declare all attributes of an instance."""
        # TODO maybe do this in class description, see Pyomo

        # The Pyomo model representing this instance.
        self._model = None

        # List of strings identifying all constraint type names.
        self._constypes = []

        # Dictionary mapping Pyomo Constraint objects to the internal
        # constraint representation.
        # The key of the dictionary is the constraint's name.
        self._consmap = {}

        # Dictionary mapping all used conshandlers to their constraint
        # lists.
        # The key of the dictionary is the conshandler name.
        self._classif = {}

        # Number of constraints without registered consclassifier.
        self._n_unclassif = 0


    def _init(self, model):
        """Derive all attribute values from a given Pyomo model.
        :param model: A Pyomo ConcreteModel representing an instance.
        """
        self._model = model
        # Create _constypes
        for ct in model.component_objects(ctype=Constraint):
            self._constypes.append(ct.name)
            # Create _Cons objects and _consmap
            for c in ct:
                # TODO there may be a problem with object identities if
                # TODO pyrep reference is used
                py_cons = ct[c]
                in_cons = _Cons.create(pyrep=py_cons, name=py_cons.name,
                                       repclass=ct.name, index=c)
                self._consmap[py_cons.name] = in_cons
                self._n_unclassif += 1

    def _clone(self):
        """Clone function for an internal Instance object.
        :return: A new internal Instance with copied data.
        """
        inst = Instance()
        inst._model = self._model.clone()
        # TODO how deep are the copies
        inst._constypes = copy.copy(self._constypes)
        inst._consmap = copy.copy(self._consmap)
        inst._classif = copy.copy(self._classif)
        inst._n_unclassif = self._n_unclassif


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
        :param constype: A string indicating the desired constraint
        representation type. If unspecified or None, all constraint
        representation types are taken into consideration.
        :return: A list of indices.
        """
        set = []
        for c in self._consmap:
            cons = self._consmap[c]
            if constype is None or constype == cons.repclass:
                if cons.conshdlr is None:
                    set.append(cons.index)
        return set


    def add_constraints(self, constype, conss, params={}):
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
                       and not conss.index_set() is new_index_set \
                       and conss.index_set() in new_index_set.set_tuple:
                    new_index_set = list(new_index_set.set_tuple)
                    new_index_set[new_index_set.index(conss.index_set())] = set
                    self._model.add_component(p, Param(*new_index_set,
                                                 mutable=True,
                                                 default=params[p].default()))
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
        for el in conss:
            cons = conss[el]
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
                for el in conss:
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

    def change_varbounds(self, var, lower_bound=None, upper_bound=None):
        """Changes the bounds of a variable.
        :param var: The name of the variable (string).
        :param lower_bound: The new lower bound. If unspecified or None,
        no changes are applied.
        :param upper_bound: The new upper bound. If unspecified or None,
        no changes are applied.
        """
        # Find the correct variable.
        # TODO do it smarter or put it into a seperated function.
        for vartype in self._model.component_objects(Var):
            for v in vartype:
                if vartype[v].name == var.name:
                    var = vartype[v]
                    break
        # Manipulate the bounds.
        if lower_bound is not None:
            var.setlb(lower_bound)
        if upper_bound is not None:
            var.setub(upper_bound)

    def register_constype(self, cons, conshdlr):
        """Assigns a constraint to a constraint handler.
        :param cons: The name of the constraint (string).
        :param conshdlr: The constraint handler object.
        """
        in_cons = self._consmap[cons.name]
        if in_cons.conshdlr is None:
            self._n_unclassif -= 1
        in_cons._classif = conshdlr
        if conshdlr.name() in self._classif.keys():
            self._classif[conshdlr.name()].append(in_cons)
        else:
            self._classif[conshdlr.name()] = [in_cons]



class _Cons:
    """This internal class stores additional constraint data.
    """

    _counter = 0

    @classmethod
    def create(cls, pyrep, name, repclass, index):
        cons = _Cons
        cons.pyrep = pyrep
        cons.name = name
        cons.repclass = repclass
        cons.index = index
        return cons

    def __init__(self):
        self.id = _Cons._counter
        _Cons._counter += 1
        self.pyrep = None
        self.name = None
        self.repclass = None
        self.index = None
        self.conshdlr = None

    def set_conshdlr(self, conshdlr):
        self.conshdlr = conshdlr
