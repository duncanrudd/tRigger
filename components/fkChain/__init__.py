from tRigger import components
from tRigger.core import attribute
reload(components)

import pymel.core as pm

class TFkChain(components.TBaseComponent):
    def __init__(self, name, side='C', index=0):
        components.TBaseComponent.__init__(self, name, side, index, 'root')
        print 'Created FK Chain Component: %s' % self.comp_name