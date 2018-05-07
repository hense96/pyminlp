


from pyminlp.conshdlr import ConsHandler


class LinearHandler(ConsHandler):


    def name(cls):
        return 'linear'

    def identify(self, set, model):
        pass


class QuadConvHandler(ConsHandler):


    def name(cls):
        return 'quadconv'

    def identify(self, set, model):
        pass


class QuadNoncHandler(ConsHandler):


    def name(cls):
        return 'quadnonc'

    def identify(self, set, model):
        pass
