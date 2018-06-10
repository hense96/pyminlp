# ______________________________________________________________________
#
#    This module is part of the PyMINLP solver framework.
# ______________________________________________________________________


import unittest

from dev.minlplib import MINLPlibTester


class MixedQCQPTester(unittest.TestCase):
    """Uses the solver to solve a set of 45 different instances from
    the MINLPlib. ALl instances are MIQCQP subproblems (e.g. QP, QCQP,
    IQCQP, MIQCQP).
    The solving process is only printed, no error checks are performed.
    TODO maybe implement assertions on the results.
    """

    def setUp(self):
        paths = []
        paths.append('./instances/minlplib_testset/osil/QP/')
        paths.append('./instances/minlplib_testset/osil/QCQP/')
        paths.append('./instances/minlplib_testset/osil/MIQCQP/')
        paths.append('./instances/minlplib_testset/osil/IQCQP/')
        self._tester = MINLPlibTester.create(paths)

    def execute(self):
        solver = self._tester.solver()
        solver.set_time_limit(15)
        solver.set_verbosity(2)
        #self._tester.set_output_file('./tests/results.txt')
        self._tester.set_sol_file('./instances/minlplib_testset/sol/'
                                  'minlplib.solu')
        solver.set_time_limit(3)
        self._tester.execute()


def main():
    suite1 = MixedQCQPTester()
    suite1.setUp()
    suite1.execute()


if __name__ == '__main__':
    unittest.main()
