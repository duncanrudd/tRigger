from tRigger import components
from tRigger.core import attribute, transform, dag
import pymel.core as pm
reload(components)
reload(transform)

import pymel.core as pm

class TControl(components.TBaseComponent):
    def __init__(self, guide):
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'control')
        self.guide = guide
        print 'Created Control Component: %s' % self.comp_name

    def addObjects(self, guide):
        for i in range(guide.num_ctrls):
            xform = pm.xform(pm.PyNode(guide.root), q=1, ws=1, m=1)
            parent = None
            if i > 0:
                parent = self.controls_list[-1]
            else:
                parent = dag.addChild(self.controls, 'group', name=self.getName('buffer_srt'))
                pm.xform(parent, ws=1, m=xform)
            num = str(i+1).zfill(2)
            self.addCtrl(shape='squarePoint', size=20.0-(i*3),
                         name=self.getName('%s_ctrl' % num), xform=xform, parent=parent)
        self.mapToGuideLocs(self.controls_list[-1], guide.locs[-1])

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('jnt'))
            self.joints_list.append(j)
            self.mapJointToGuideLocs(j, guide.locs[-1])
        components.TBaseComponent.addObjects(self, guide)

    def addConnections(self, rig):
        parent = self.guide.getGuideParent()
        if parent:
            compObj = pm.PyNode('_'.join(parent.name().split('_')[:2]) + '_comp')
            parentObj = rig[compObj.name()].guideToRigMapping[parent]
            output = rig[compObj.name()].addOutput(parentObj)
            input = attribute.addMatrixAttr(self.input, 'parent_in_mtx')
            output.connect(input)
            self.connectToInput(input, self.controls_list[0].getParent())
            print 'Parent Object: %s' % parentObj.name()
        else:
            print 'No parent found: %s' % self.guide.root.name()


def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TControl(guide)

