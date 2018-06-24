# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


from dev.minlplib import MINLPlibTester
import cProfile


def execute():
    tester = set_up()
    tester.execute()


def set_up():
    paths = []
    paths.append('./instances/profiling/')
    tester = MINLPlibTester.create(paths)
    solver = tester.solver()
    solver.set_time_limit(120)
    solver.set_verbosity(1)
    tester.set_sol_file('./instances/minlplib_MIQCQP/sol/'
                        'minlplib.solu')
    return tester


pr = cProfile.Profile()
pr.enable()
execute()
pr.disable()
# after your program ends
pr.print_stats(sort='cumulative')
