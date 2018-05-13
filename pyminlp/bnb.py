import numpy as np

from pyminlp.hub import Coordinator



class BranchAndBound:

    @classmethod
    def create(cls, coordinator):
        obj = BranchAndBound()
        obj._interface = coordinator
        return obj

    def __init__(self):
        self._interface = None
        self._open_nodes = []
        self._root_node = None
        self._cur_node = None

    def execute(self):
        """Performs the branch and bound algorithm."""
        # Initialize data.
        upper_bound = np.inf
        best_node = None
        self._open_nodes = [self._root_node]
        open_nodes = self._open_nodes
        node_count = 0

        # Outer loop for all generated nodes.
        while len(open_nodes) > 0:
            node_count += 1
            # Perform node selection.
            self._cur_node = open_nodes.pop(0)
            node = self._cur_node
            # Add cuts to the initial problem without solving it before.
            # TODO necessary?
            self._interface.add_underestimators(node.instance)
            node_done = False

            # Inner loop for a single node.
            while not node_done:
                # Solve relaxation.
                self._interface.solve_relaxation(node.instance)
                # TODO handle unbounded case.
                if not node.has_optimal_solution():
                    # Pruning.
                    break
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
                        best_node = node
                        # Check whether new solution prunes any other
                        # nodes.
                        del_count = 0
                        for i in reversed(range(len(open_nodes))):
                            if open_nodes[i].lower_bound >= upper_bound:
                                open_nodes.pop(i)
                                del_count += 1
                    else:
                        # Tighten relaxation, bounds or perform
                        # branching.
                        self._interface.enforce()
                        if node.branched:
                            node_done = True

        return best_node, open_nodes

    def register_instance(self, instance):
        self._root_node = _Node.create_node(instance)
        pass

    def branching(self, child_instance_list):
        self._cur_node.set_branched()
        for inst in child_instance_list:
            node = _Node.create_node(instance=inst, parent_node=self._cur_node)
            self._open_nodes.append(node)

    def relaxation_solved(self):
        node = self._cur_node
        if node.has_solution():
            node.set_lower_bound(node.rel_sol_value())


class _Node:
    """Objects of the _Node class for the branch and bound algorithm
    maintains the current problem instance as well as all kind of node
    related data relevant in the branch and bound process.
    """

    _node_count = 0

    @classmethod
    def create_node(cls, instance, parent_node=None):
        node = _Node()
        node._instance = instance
        node._number = _Node._node_count
        _Node._node_count += 1
        if parent_node is not None:
            node._depth = parent_node._depth + 1
            node._lower_bound = parent_node._lower_bound
        return node

    def __init__(self):
        # The problem instance.
        self._instance = None
        # An identifying number for the node.
        self._number = None
        # The node's depth in the branch and bound tree.
        self._depth = 0
        # The lower bound for the solution's objective function value.
        self._lower_bound = -np.inf
        # A flag indicating whether the node is branched.
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

    def set_lower_bound(self, lower_bound):
        self._lower_bound = lower_bound

    @property
    def branched(self):
        return self._branched

    def set_branched(self):
        self._branched = True

    def feasible(self):
        return self._instance.feasible()

    def has_optimal_solution(self):
        return self._instance.has_optimal_solution()

    def rel_sol_value(self):
        return self._instance.objective_function_value()
