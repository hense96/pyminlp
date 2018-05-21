# This module coordinates the branch and bound process on an abstract
# level. For any user related functions, it calls the functions of the
# hub.


import numpy as np
from enum import Enum

from pyminlp.subprob import Instance
from pyminlp.stats import Stats


class BranchAndBound:
    """This class encapsulates the branch and bound algorithm.
    Particularly, it stores the current status of the algorithm and
    makes offers functions for modifying the process.

    Note that the BranchAndBound class makes use of the _Node class,
    but never passes _Node objects to other modules.

    Use the factory method _Cons.create(...) for generating a new
    instance.

    Private class attributes:
        _interface  A reference to the Coordinator object that works as
                        an interface for the branch and bound algorithm
                        for all user related functions.
        _open_nodes The list of _Node objects that have not been
                        considered yet in the branch and bound process.
        _root_node  The root node of the branch and bound tree (_Node
                        object).
        _cur_node   The node that is currently being considered within
                        the branch and bound algorithm.
        _best_node  The node containing the best found feasible
                        solution.
    """

    @classmethod
    def create(cls, coordinator):
        """Factory method to generate a new BranchAndBound object.
        :coordinator: A hub.Coordinator object.
        """
        obj = BranchAndBound()
        obj._interface = coordinator
        return obj

    def __init__(self):
        """Constructor setting attributes to None."""
        self._interface = None
        self._open_nodes = []
        self._root_node = None
        self._cur_node = None
        self._best_node = None

    def execute(self):
        """Performs the branch and bound algorithm.
        Precondition: An instance is registered."""
        assert self._root_node is not None

        Stats.start_bnb(self)

        # Initialize data.
        upper_bound = np.inf
        self._open_nodes = [self._root_node]
        open_nodes = self._open_nodes
        node_count = 0

        # Outer loop for all generated nodes.
        while len(open_nodes) > 0:
            node_count += 1
            # Perform node selection and register new instance.
            self._cur_node = open_nodes.pop(0)
            node = self._cur_node
            self._interface.set_instance(node.instance)

            Stats.start_node(self)

            # Assign all constraints of this node to a constraint
            # handler.
            result = self._interface.identify()
            if result != UserInputStatus.OK:
                raise ValueError('Status after identify method call is {}. '
                                 'Expected OK.'.format(result))

            # Call preparation routines.
            result = self._interface.prepare()
            if result == UserInputStatus.OK:
                node_done = False
            elif result == UserInputStatus.INFEASIBLE:
                node_done = True
            else:
                raise ValueError('Status after preparation method call is {}. '
                                 'Expected OK or INFEASIBLE.'.format(result))

            # Inner loop for a single node.
            while not node_done:

                Stats.start_rel_sol(self)

                # Solve relaxation.
                result = self._interface.solve_relaxation()
                if result != UserInputStatus.OK:
                    raise ValueError(
                        'Status after solve_relaxation method call is {}. '
                        'Expected OK.'.format(result))

                Stats.finish_rel_sol(self)

                if node.has_optimal_solution():
                    node.update_lower_bound()
                elif node.relax_infeasible():
                    # TODO conflict analysis?
                    break
                else:
                    raise ValueError('The relaxation solver terminated with '
                                     'termination condition {}.'.format(
                                 node.instance.relax_termination_condition()))

                # Check dual bound.
                sol = node.rel_sol_value()
                if sol >= upper_bound:
                    # Pruning.
                    node_done = True
                else:
                    # Feasibility checking.
                    if node.feasible():
                        node_done = True
                        # Update the global bounds.
                        upper_bound = sol
                        self._best_node = node
                        # Check whether new solution prunes any other
                        # nodes.
                        del_count = 0
                        for i in reversed(range(len(open_nodes))):
                            if open_nodes[i].lower_bound >= upper_bound:
                                open_nodes.pop(i)
                                del_count += 1
                    else:
                        # Call the enforce method.
                        result = self._interface.enforce()
                        if result == UserInputStatus.INFEASIBLE:
                            # TODO conflict analysis?
                            node_done = True
                        elif result == UserInputStatus.BRANCHED:
                            self._cur_node.set_branched()
                            node_done = True
                        elif result == UserInputStatus.RESOLVE:
                            node_done = False
                        else:
                            raise ValueError(
                                'Status after solve_relaxation method'
                                'call is {}. Expected DONE, RESOLVE '
                                'or INFEASIBLE.'.format(result))

            Stats.finish_node(self)

        Stats.finish_bnb(self)

    # Interface functions to influence the solving process.

    def register_root_instance(self, instance):
        """Registers the instance that is to solve. It will be saved
        as root node of the branch and bound tree.
        """
        assert type(instance) is Instance
        self._root_node = _Node.create_node(instance)

    def register_child_instances(self, instance_list):
        """Registers the children instances of the current node.
        :param instance_list: A list of instances.
        """
        for inst in instance_list:
            node = _Node.create_node(instance=inst,
                                     parent_node=self._cur_node)
            self._open_nodes.append(node)


class UserInputStatus(Enum):
    """Enumeration for indicating what the result of a user interaction
    method is."""
    OK = 1
    RESOLVE = 2
    BRANCHED = 3
    INFEASIBLE = 4
    FAIL = 5


class _Node:
    """Objects of the _Node class for the branch and bound algorithm
    maintains the current problem instance as well as all kind of node
    related data relevant in the branch and bound process.

    Use the factory method _Node.create(...) for generating a new
    instance.

    Public class attributes:
        instance     The underlying problem instance (Instance object).
        number       A number identifying the node.
        depth        The depth of the node object in the branch and
                         bound tree.
        lower_bound  The lower bound for the objective function value,
                         derived from relaxation solutions from parent
                         nodes and the relaxation solution of this node.
        branched     A flag indicating whether this node is branched.
    """

    @classmethod
    def create_node(cls, instance, parent_node=None):
        """Factory method to create a _Node object. Provide the
        underlying instance. Provide a parent node, if existing."""
        node = _Node()
        node._instance = instance
        node._number = _Node._node_count
        _Node._node_count += 1
        if parent_node is not None:
            node._depth = parent_node._depth + 1
            node._lower_bound = parent_node._lower_bound
        return node

    # Counts how many nodes are created (static).
    _node_count = 0

    def __init__(self):
        """Constructor setting attributes to None or default values."""
        self._instance = None
        self._number = None
        self._depth = 0
        self._lower_bound = -np.inf
        self._branched = False

    @property
    def instance(self):
        return self._instance

    @property
    def number(self):
        return self._number

    @property
    def depth(self):
        return self._depth

    @property
    def lower_bound(self):
        return self._lower_bound

    def update_lower_bound(self):
        val = self.rel_sol_value()
        if val > self.lower_bound:
            self._lower_bound = val

    @property
    def branched(self):
        return self._branched

    def set_branched(self):
        """Sets that the node is branched."""
        self._branched = True

    def feasible(self):
        """Returns true, if the solution of the relaxation of the
        underlying instance is feasible."""
        if self.has_optimal_solution():
            if self._instance.feasible():
                return True
        else:
            return False

    def has_optimal_solution(self):
        """Returns True if the relaxation of the instance has an optimal
        solution, i.e. the relaxation is neither infeasible nor
        unbounded.
        """
        return self._instance.has_optimal_solution()

    def relax_infeasible(self):
        """Returns True if the relaxation of the instance is infeasible.
        """
        return self._instance.relax_infeasible()

    def rel_sol_value(self):
        """Returns the optimal objective function value of the solved
        relaxation of the instance.
        """
        return self._instance.objective_function_value()
