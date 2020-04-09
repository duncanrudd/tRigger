from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps
import pymel.core as pm
reload(components)
reload(transform)

import pymel.core as pm

class TControlLocal(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'controlLocal')
        print 'Created Control Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = (self.guide.guide_side == 'R')
        xform = pm.PyNode(guide.root).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        for i in range(guide.num_ctrls):
            parent = self.base_srt
            if i > 0:
                parent = self.controls_list[-1]
            num = str(i+1).zfill(2)
            self.addCtrl(shape='squarePoint', size=20.0-(i*3),
                         name=self.getName(num), xform=xform, parent=parent, metaParent=parent)

        self.out_srt = dag.addChild(self.rig, 'group', name=self.getName('out_srt'))
        self.mapToGuideLocs(self.out_srt, guide.locs[-1])

        offsetMtx = mathOps.composeMatrixFromMatrix(self.base_srt.worldMatrix[0].get())

        if not self.invert:
            mtx = mathOps.multiplyMatrices([self.controls_list[0].worldMatrix[0],
                                            self.base_srt.worldInverseMatrix[0],
                                            offsetMtx.outputMatrix],
                                           name=self.getName('outSrt_mtx'))
        else:
            xform = mathOps.createInverseHandedMatrix(self.controls_list[-1].worldMatrix[0],
                                                      name=self.getName('out'))
            mtx = mathOps.multiplyMatrices([xform.matrixSum,
                                            self.base_srt.worldInverseMatrix[0],
                                            offsetMtx.outputMatrix],
                                           name=self.getName('outSrt_mtx'))

        mtx.matrixSum.connect(self.out_srt.offsetParentMatrix)

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('jnt'))
            self.joints_list.append({'joint': j, 'driver': self.out_srt})
            self.mapJointToGuideLocs(j, guide.locs[-1])
        components.TBaseComponent.addObjects(self, guide)

    def finish(self):
        self.setColours(self.guide)
        attribute.channelControl(nodeList=self.controls_list, attrList=['rotateOrder'], keyable=1, lock=0)
        attrList = ['visibility']
        attribute.channelControl(nodeList=self.controls_list, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TControlLocal(guide)

