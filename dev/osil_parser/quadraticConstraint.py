
class QuadraticConstraint(object):

    def __init__(self, name, lb=-float('inf'), ub=float('inf'), linear=None, quadratic=None):
        # TODO: Should we set things to None if zeroes?
        self.name = name
        self.lb = lb
        self.ub = ub
        self.linear = linear
        self.quadratic = quadratic
#        print("Initialising quad: lb is : " + str(lb))
