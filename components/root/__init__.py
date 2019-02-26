from tRigger import components
from tRigger.core import attribute
reload(components)

import pymel.core as pm

class TRoot(components.TBaseComponent):
    def __init__(self, name, side='C', index=0):
        components.TBaseComponent.__init__(self, name, side, index, 'root')
        print 'Created Root Component: %s' % self.comp_name

