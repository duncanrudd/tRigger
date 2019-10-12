from tRigger import components
from tRigger.core import attribute
reload(components)

import pymel.core as pm

class TRoot(components.TBaseComponent):
    def __init__(self, name, side='C', index=0):
        components.TBaseComponent.__init__(self, name, side, index, 'root')
        print 'Created Root Component: %s' % self.comp_name

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TRoot(name=guide.guide_name,
                 side=guide.guide_side,
                 index=guide.guide_index)

