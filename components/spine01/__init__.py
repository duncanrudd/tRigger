from tRigger import components
from tRigger.core import attribute
reload(components)

import pymel.core as pm

class TSpine01(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'spine01')
        print 'Created Spine01 Component: %s' % self.comp_name

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSpine01(guide)
