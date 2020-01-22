from tRigger import components
from tRigger.core import attribute, transform, dag, icon
import pymel.core as pm
reload(components)
reload(transform)

import pymel.core as pm

class TControl(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'control')
        print 'Created Control Component: %s' % self.comp_name

    def addObjects(self, guide):
        for i in range(guide.num_ctrls):
            xform = pm.PyNode(guide.root).worldMatrix[0].get()
            parent = self.base_srt
            if i > 0:
                parent = self.controls_list[-1]
            num = str(i+1).zfill(2)
            self.addCtrl(shape='squarePoint', size=20.0-(i*3),
                         name=self.getName(num), xform=xform, parent=parent)
        self.mapToGuideLocs(self.controls_list[-1], guide.locs[-1])

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('jnt'))
            self.joints_list.append({'joint': j, 'driver': self.controls_list[-1]})
            self.mapJointToGuideLocs(j, guide.locs[-1])
        components.TBaseComponent.addObjects(self, guide)

    def finish(self):
        self.setColours(self.guide)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TControl(guide)

