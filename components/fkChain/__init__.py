from tRigger import components
from tRigger.core import attribute
reload(components)

import pymel.core as pm

class TFkChain(components.TBaseComponent):
    def __init__(self, name, side, index, segments, axis, upAxis):
        components.TBaseComponent.__init__(self, name, side, index, 'root')
        print 'Created FK Chain Component: %s' % self.comp_name

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TFkChain(name=guide.guide_name,
                    side=guide.guide_side,
                    index=guide.guide_index,
                    segments=guide.num_segments,
                    axis=guide.axis,
                    upAxis=guide.up_axis)
