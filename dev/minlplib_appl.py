# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


from dev.minlplib import MINLPlibTester


def execute():
    tester = set_up(include_convex_hdlr=False)
    tester.set_output_file('./dev/nonconv_results.txt')
    tester.execute()

    tester = set_up(include_convex_hdlr=True)
    tester.set_output_file('./dev/conv_results.txt')
    tester.execute()


def set_up(include_convex_hdlr):
    paths = []
    paths.append('./instances/minlplib_MIQCQP/osil/MIQCQP/')
    paths.append('./instances/minlplib_MIQCQP/osil/QCQP/')
    tester = MINLPlibTester.create(paths, include_convex_hdlr)
    solver = tester.solver()
    solver.set_time_limit(250)
    solver.set_verbosity(1)
    tester.set_sol_file('./instances/minlplib_MIQCQP/sol/'
                        'minlplib.solu')
    return tester
