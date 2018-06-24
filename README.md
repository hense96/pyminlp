# PyMINLP solver framework

This project aims at developing a global optimisation solver framework for mixed integer nonlinear optimisation problems in Python.

Coding conventions: PEP8 (https://www.python.org/dev/peps/pep-0008/)

Description:
The PyMINLP solver is a spatial branch and bound global optimisation framework. It can be customised in a way that it is capable of solving any kind of MINLP instance that can be modelled in Pyomo (http://www.pyomo.org). The solver offers a plugin structure that enables the user to vary the solving process and to integrate partial algorithms to improve the solver's performance.
The plugin environment leverages the usability and the features of the Python programming language and the flexibility of the Pyomo modelling language. Moreover, the solver offers a small, transparent set of interface functions.
