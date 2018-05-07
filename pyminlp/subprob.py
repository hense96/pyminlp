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



    def _init_new(self, py_model):
        # TODO cons normalisation
        # TODO finish
        self._model = py_model.clone()
        model = self._model
        for comp in model.component_objects(descend_into=False):
            model.del_component(comp)
        # Reconstruct unit with changed sets.
        for set in py_model.component_objects(ctype=Set):
            model.add_component(set.name, Set())
            for el in set:
                model.component(set.name).add(
                    '_{}_{}'.format(Instance._consadd_count, set[el]))





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
                py_cons = ct[c]
                in_cons = _Cons()
                # TODO there may be a problem with object identities if
                # TODO pyrep reference is used
                in_cons.pyrep = py_cons
                in_cons.name = py_cons.name
                in_cons.repclass = ct.name
                in_cons.index = c
                self._consmap[py_cons.name] = in_cons
                self._n_unclassif += 1

    def _clone(self):
        """Clone function for an internal Instance object.
        :return: A new internal Instance with copied data.
        """
        # TODO how deep are the copies
        inst = Instance()
        inst._model = self._model.clone()
        inst._constypes = copy.copy(self._constypes)
        inst._consmap = copy.copy(self._consmap)
        inst._classif = copy.copy(self._classif)
        inst._n_unclassif = self._n_unclassif


    def model(self):
        # TODO maybe only return a copy
        return self._model

    def relaxation(self):
        pass


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
        in_cons = self._consmap[cons.name]
        if in_cons.conshdlr is None:
            self._n_unclassif -= 1
        in_cons._classif = conshdlr
        if conshdlr.name() in self._classif.keys():
            self._classif[conshdlr.name()].append(in_cons)
        else:
            self._classif[conshdlr.name()] = [in_cons]



class _Cons:

    _counter = 0

    def __init__(self):
        self.id = _Cons._counter
        _Cons._counter += 1
        self.pyrep = None
        self.name = None
        self.repclass = None
        self.conshdlr = None
        self.index = None
