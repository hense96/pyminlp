
class Variable(object):

    def __init__(self, varType='C', lb=0, ub=float('inf'), name=None):
        self.varType = varType
        self.lb = lb
        self.ub = ub
        self.name = name
