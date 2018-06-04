# This module collects statistics from a solving process, processes
# them and generates output data.
#
# TODO maybe: no access of private variables.


import numpy as np
import time
from enum import Enum


class Stats:

    _stats = None

    @classmethod
    def initialise(cls, verbosity=None):
        cls._stats = Stats()
        stats = cls._stats
        if type(verbosity) is int:
            stats.verb = Verbosity(verbosity)
        elif type(verbosity) is Verbosity:
            stats.verb = verbosity

    def __init__(self):
        # Options.
        self.verb = Verbosity.DEBUG
        self.STANDARD_WAIT = 10

        # Data for output control.
        self.last_print_time = None

        # Global level information.
        self.solver_start_time = None
        self.solver_time = None
        self.nnodes = 0
        self.lower = -np.Inf
        self.upper = np.Inf

        # Node level information.
        self.node_start_time = None
        self.node_time = None
        self.n_rel_sol = 0
        self.n_cuts = 0
        self.n_children = 0

        # Relaxation solution information.
        self.rel_sol_start_time = None
        self.rel_sol_time = None

    def reset_node_data(self):
        self.node_start_time = None
        self.node_time = None
        self.n_rel_sol = 0
        self.n_cuts = 0
        self.n_children = 0

    def standard_output(self, stage):
        assert type(stage) is int
        if stage == 0:
            out = '\nPyMINLP starts solving.\n'
        elif stage == 2:
            out = '\nPyMINLP finished solving.\n'
        else:
            out = '\nPyMINLP is solving.\n'
        out += '  Solving time: \t {:.3f} s\n'.format(time.clock()
                                                      - self.solver_start_time)
        out += '  Nodes: \t \t \t {}\n'.format(self.nnodes)
        out += '  Primal bound: \t {}\n'.format(self.upper)
        out += '  Dual bound: \t \t {}\n'.format(self.lower)
        return out

    @classmethod
    def start_bnb(cls, bnb):
        stats = cls._stats
        stats.solver_start_time = time.clock()

        # Printing.
        if stats.verb == Verbosity.STANDARD:
            print(stats.standard_output(0))
            stats.last_print_time = time.clock()

    @classmethod
    def finish_bnb(cls, bnb):
        stats = cls._stats
        stats.solver_time = time.clock() - stats.solver_start_time

        # Update bounds if possible.
        if bnb._lower_bound > stats.lower:
            stats.lower = bnb._lower_bound
        if bnb._upper_bound > stats.upper:
            stats.upper = bnb._upper_bound

        # Printing.
        if stats.verb == Verbosity.STANDARD:
            print(stats.standard_output(2))
        elif stats.verb == Verbosity.DEBUG:
            print(stats.standard_output(2))


    @classmethod
    def start_node(cls, bnb):
        stats = cls._stats
        stats.node_start_time = time.clock()
        stats.nnodes += 1

        # Update lower bound if possible.
        if bnb._lower_bound > stats.lower:
            stats.lower = bnb._lower_bound

        # Printing.
        if stats.verb == Verbosity.DEBUG:
            out = '\n--- NODE #{}, depth {} ---\n'.format(bnb._cur_node.number,
                                                          bnb._cur_node.depth)
            out += '   Primal bound: \t {}\n'.format(stats.upper)
            out += '   Dual bound: \t \t {}\n'.format(stats.lower)
            out += '   Solving time: \t {:.3f} s\n'.format(
                                        time.clock() - stats.solver_start_time)
            print(out)

    @classmethod
    def finish_node(cls, bnb):
        stats = cls._stats
        stats.node_time = time.clock() - stats.node_start_time

        # Update primal bound.
        if bnb._upper_bound < stats.upper:
            stats.upper = bnb._upper_bound

        if stats.verb == Verbosity.DEBUG:
            out = ' -> Finished node in {:.3f} s\n'.format(stats.node_time)
            if bnb._cur_node.branched:
                reason = 'branching'
            elif bnb._cur_node.feasible():
                reason = 'feasible'
            else:
                reason = 'infeasible'
            out += '   Reason: \t \t \t \t {}\n'.format(reason)
            out += '   Solved relaxations: \t {}\n'.format(stats.n_rel_sol)
            out += '   Cuts added: \t \t \t {}\n'.format(stats.n_cuts)
            out += '   Children: \t \t \t {}'.format(stats.n_children)
            print(out)

        stats.reset_node_data()

    @classmethod
    def start_rel_sol(cls, bnb):
        stats = cls._stats
        stats.rel_sol_start_time = time.clock()

        # Update lower bound if possible.
        if bnb._lower_bound > stats.lower:
            stats.lower = bnb._lower_bound


    @classmethod
    def finish_rel_sol(cls, bnb):
        stats = cls._stats
        stats.rel_sol_time = time.clock() - stats.rel_sol_start_time
        stats.n_rel_sol += 1

        if stats.verb == Verbosity.STANDARD:
            if time.clock() - stats.last_print_time > stats.STANDARD_WAIT:
                print(stats.standard_output(1))
                stats.last_print_time = time.clock()
        elif stats.verb == Verbosity.DEBUG:
            out = ' -> Solved relaxation in {:.3f} s\n'.format(
                                                      stats.rel_sol_time)
            if bnb._cur_node.relax_infeasible():
                out += '   Result: infeasible'
            elif bnb._cur_node.has_optimal_solution():
                out += '   Result: {}'.format(bnb._cur_node.rel_sol_value())
            print(out)

    @classmethod
    def identify(cls, coord, curdata):
        stats = cls._stats

    @classmethod
    def prepare(cls, coord, curdata):
        stats = cls._stats
        stats.n_cuts += curdata.ncuts

        if stats.verb == Verbosity.DEBUG:
            print(' -> prepare ({}).'.format(curdata.handler.name()))

    @classmethod
    def separate(cls, coord, curdata):
        stats = cls._stats
        stats.n_cuts += curdata.ncuts
        stats.n_children += len(curdata.children)

        if stats.verb == Verbosity.DEBUG:
            print(' -> separate ({}).'.format(curdata.handler.name()))

    # Interface methods for accessing data inside the solver.

    @classmethod
    def get_solver_time(cls):
        stats = cls._stats
        assert stats.solver_time is not None
        return stats.solver_time


class Verbosity(Enum):
    """Defines different levels of verbosity."""
    NONE = 1
    STANDARD = 2
    DEBUG = 3
