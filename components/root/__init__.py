from tRigger import components
from tRigger.core import attribute
import pymel.core as pm
reload(components)

import pymel.core as pm

class TRoot(components.TBaseComponent):
    def __init__(self, name, side='C', index=0):
        components.TBaseComponent.__init__(self, name, side, index, 'root')
        print 'Created Root Component: %s' % self.comp_name

    def addObjects(self, guide):
        for i in range(guide.num_ctrls):
            xform = pm.xform(pm.PyNode(guide.root), q=1, ws=1, m=1)
            parent = None
            if i > 0:
                parent = self.controls_list[-1]
            num = str(i+1).zfill(2)
            self.addCtrl(shape='squarePoint', size=20.0-(i*3),
                         name=self.getName('%s_ctrl' % num), xform=xform, parent=parent)
        components.TBaseComponent.addObjects(self, guide)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TRoot(name=guide.guide_name,
                 side=guide.guide_side,
                 index=guide.guide_index)

