# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


import numpy as np
import heapq
from enum import Enum

from pyminlp.subprob import Instance
from pyminlp.stats import Stats


class BranchAndBound:
    """This class encapsulates the branch and bound algorithm.
    Particularly, it stores the current status of the algorithm and
    offers functions for modifying the process.

    Note that the BranchAndBound class makes use of the _Node class,
    but never passes _Node objects to other modules.

    Use the factory method _Cons.create(...) for generating a new
    instance.

    Private class attributes:
        _interface   A reference to the Coordinator object that works
                         as an interface for the branch and bound
                         algorithm for all user related functions.
        _open_nodes  The priority queue of _Node objects that have not
                         been considered yet in the branch and bound
                         process. The priority is given by the local
                         lower bound of the nodes. The objects in the
                         queue have the following shape:
                             (priority, object)
        _node_count  The number of nodes that have been considered.
                         Open nodes that have not been considered yet
                         do not influence this number.
        _root_node   The root node of the branch and bound tree (_Node
                         object).
        _cur_node    The node that is currently being considered within
                         the branch and bound algorithm (_Node object.)
        _best_node   The node containing the best found feasible
                         solution.
        _child_nodes A list of child nodes of the _cur_node in case
                         branching is performed.
        _lower_bound The current global lower bound for the optimal
                         objective function value of the root node.
        _upper_bound The current global upper bound for the optimal
                         objective function value of the root node.
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
        self._node_count = None
        self._root_node = None
        self._cur_node = None
        self._child_nodes = None
        self._best_node = None
        self._lower_bound = None
        self._upper_bound = None

    def execute(self, epsilon=None):
        """Performs the branch and bound algorithm.
        Precondition: An instance is registered."""
        assert self._root_node is not None

        Stats.start_bnb(self)

        # Initialize data.
        self._lower_bound = -np.inf
        self._upper_bound = np.inf
        open_nodes = self._open_nodes
        heapq.heappush(open_nodes,
                       (self._root_node.lower_bound, self._root_node))
        self._node_count = 0

        # Outer loop for all generated nodes.
        while len(open_nodes) > 0:

            # Update global lower bound if possible.
            lower, _ = open_nodes[0]
            if lower > self._lower_bound:
                self._lower_bound = lower

            # Check if current solution is good enough.
            if self._upper_bound - self._lower_bound < epsilon:
                break

            # Perform node selection and register the chosen instance.
            # TODO maybe offer a plugin for this.
            _, self._cur_node = heapq.heappop(open_nodes)
            node = self._cur_node
            self._interface.set_instance(node.instance)
            self._child_nodes = []
            self._node_count += 1

            # Read the minimal lower bound of all other nodes.
            if len(open_nodes) > 0:
                minimal_lower, _ = open_nodes[0]
            else:
                minimal_lower = None

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
            elif result == UserInputStatus.BRANCHED:
                if len(self._child_nodes) == 0:
                    raise ValueError('Branching performed but no child nodes '
                                     'were given.')
                for node in self._child_nodes:
                    heapq.heappush(self._open_nodes,
                                   (node.lower_bound, node))
                self._cur_node.set_branched()
                node_done = True
            else:
                raise ValueError('Status after preparation method call is {}. '
                                 'Expected OK, INFEASIBLE, '
                                 'BRANCHED.'.format(result))

            # Inner loop for a single node.
            while not node_done:

                Stats.start_rel_sol(self)

                # Solve relaxation.
                result = self._interface.solve_relaxation()
                # Handle the user input.
                if result != UserInputStatus.OK:
                    raise ValueError(
                        'Status after solve_relaxation method call is {}. '
                        'Expected OK.'.format(result))

                Stats.finish_rel_sol(self)

                # Evaluate result of the solution of the relaxation.
                if node.has_optimal_solution():
                    node.update_lower_bound()
                    # Update global lower bound if possible.
                    if node.lower_bound > self._lower_bound:
                        if minimal_lower is None:
                            self._lower_bound = node.lower_bound
                        elif minimal_lower > self._lower_bound:
                            self._lower_bound = min(minimal_lower,
                                                    node.lower_bound)
                elif node.relax_infeasible():
                    break
                else:
                    raise ValueError('The relaxation solver terminated with '
                                     'termination condition {}.'.format(
                                 node.instance.relax_termination_condition()))

                # Check dual bound.
                sol = node.rel_sol_value()
                if sol >= self._upper_bound:
                    # Pruning.
                    node_done = True
                else:
                    # Feasibility checking.
                    if node.feasible():
                        node_done = True
                        # Update the global bounds.
                        self._upper_bound = sol
                        self._best_node = node
                        # Check whether new solution prunes any other
                        # nodes.
                        del_count = 0
                        for i in reversed(range(len(open_nodes))):
                            _, open_node = open_nodes[i]
                            if open_node.lower_bound >= self._upper_bound:
                                open_nodes.pop(i)
                                del_count += 1
                    else:
                        # Enforce the violated constraints.
                        result = self._interface.enforce()
                        # Handle user decision.
                        if result == UserInputStatus.INFEASIBLE:
                            node_done = True
                        elif result == UserInputStatus.BRANCHED:
                            if len(self._child_nodes) == 0:
                                raise ValueError('Branching performed but no '
                                                 'child nodes were given.')
                            for node in self._child_nodes:
                                heapq.heappush(self._open_nodes,
                                               (node.lower_bound, node))
                            self._cur_node.set_branched()
                            node_done = True
                        elif result == UserInputStatus.RESOLVE:
                            node_done = False
                        else:
                            raise ValueError(
                                'Status after solve_relaxation method'
                                'call is {}. Expected OK, RESOLVE '
                                'or INFEASIBLE.'.format(result))

                if self._interface.time_limit_reached():
                    break

            Stats.finish_node(self)

            if self._interface.time_limit_reached():
                break

        if len(open_nodes) == 0 and self._best_node is not None:
            self._lower_bound = self._best_node.lower_bound

        Stats.finish_bnb(self)

    # Interface functions to influence the solving process.

    def register_root_instance(self, instance):
        """Registers the instance that is to solve. It will be saved
        as root node of the branch and bound tree.
        :param instance: An Instance object.
        """
        assert type(instance) is Instance
        self._root_node = _Node.create_node(instance)

    def register_child_instances(self, instance_list):
        """Registers the children instances of the current node.
        :param instance_list: A list of Instance objects.
        """
        for inst in instance_list:
            node = _Node.create_node(instance=inst,
                                     parent_node=self._cur_node)
            self._child_nodes.append(node)

    # Getter functions.

    def best_instance(self):
        """Returns the instance of the best node found in the branch
        and bound process. This instance holds the best feasible
        solution, for example. If None is returned, no feasible node
        has been found.
        :return: An Instance object or None.
        """
        if self._best_node is None:
            return None
        else:
            return self._best_node.instance

    def nconsiderednodes(self):
        """Returns the number of nodes that have been considered in the
        solving process.
        """
        assert self._node_count is not None
        return self._node_count

    def nopennodes(self):
        """Returns the number of nodes that have been created but not
        considered yet.
        """
        assert self._open_nodes is not None
        return len(self._open_nodes)

    def lower_bound(self):
        """Returns the best found lower bound for the objective
        function value.
        """
        assert self._lower_bound is not None
        return self._lower_bound

    def upper_bound(self):
        """Returns the best found upper bound for the objective
        function value.
        """
        assert self._upper_bound is not None
        return self._upper_bound


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
        upper_bound  TODO not implemented yet.
        branched     A flag indicating whether this node is branched.
    """

    @classmethod
    def create_node(cls, instance, parent_node=None):
        """Factory method to create a _Node object. Provide the
        underlying instance. Provide a parent node, if existing."""
        node = _Node()
        node._instance = instance
        node._number = _Node._node_count
        # node._instance.set_name('{}'.format(_Node._node_count))
        node._instance.set_name(_Node._node_count)
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
        # TODO not implemented yet.
        self._upper_bound = np.inf
        self._branched = False

    def __lt__(self, other):
        return self.number < other.number

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
    def upper_bound(self):
        return self._upper_bound

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
