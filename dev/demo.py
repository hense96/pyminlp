# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


from dev.minlplib import MINLPlibTester


def execute():
    tester = MINLPlibTester.create(paths=[], include_convex_hdlr=True)
    tester.add_file_to_testset('./instances/minlplib_MIQCQP/'
                               'osil/QCQP/ex9_2_6.osil')
    solver = tester.solver()
    solver.set_time_limit(120)
    solver.set_verbosity(2)
    tester.set_sol_file('./instances/minlplib_testset/sol/minlplib.solu')
    tester.execute()
