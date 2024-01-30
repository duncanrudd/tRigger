from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps
import pymel.core as pm
reload(components)
reload(transform)

import pymel.core as pm

class TCockler(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'cockler')
        print 'Created Cockler Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = (self.guide.guide_side == 'R')
        xform = pm.PyNode(guide.root).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)

        ctrlSize = mathOps.getDistance(guide.locs[0], guide.locs[1])
        parent = self.base_srt
        self.ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize,
                                 name=self.getName('base'), xform=xform, parent=parent, metaParent=parent)

        self.rig_srt = dag.addChild(self.rig, 'group', name=self.getName('rig_srt'))
        self.ctrl.worldMatrix[0].connect(self.rig_srt.offsetParentMatrix)

        self.inner_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('inner_srt'))
        self.inner_srt.offsetParentMatrix.set(
            guide.locs[1].worldMatrix[0].get() * self.rig_srt.worldInverseMatrix.get())

        self.outer_srt = dag.addChild(self.inner_srt, 'group', name=self.getName('outer_srt'))
        self.outer_srt.offsetParentMatrix.set(
            guide.locs[2].worldMatrix[0].get() * self.inner_srt.worldInverseMatrix.get())

        self.front_srt = dag.addChild(self.outer_srt, 'group', name=self.getName('front_srt'))
        self.front_srt.offsetParentMatrix.set(
            guide.locs[3].worldMatrix[0].get() * self.outer_srt.worldInverseMatrix.get())

        self.back_srt = dag.addChild(self.front_srt, 'group', name=self.getName('back_srt'))
        self.back_srt.offsetParentMatrix.set(
            guide.locs[4].worldMatrix[0].get() * self.front_srt.worldInverseMatrix.get())

        self.out_srt = dag.addChild(self.back_srt, 'group', name=self.getName('out_srt'))
        self.out_srt.offsetParentMatrix.set(
            guide.root.worldMatrix[0].get() * self.back_srt.worldInverseMatrix.get())
        if self.invert:
            self.out_srt.sx.set(-1)

        self.mapToGuideLocs(self.out_srt, guide.locs[0])

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('jnt'))
            self.joints_list.append({'joint': j, 'driver': self.out_srt})
            self.mapJointToGuideLocs(j, guide.locs[-1])
        components.TBaseComponent.addObjects(self, guide)

        # Attach params shape to last control
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.controls_list[-1], lw=1, wt=1)
        pm.delete(tempJoint)

    def addAttributes(self):
        attribute.addAngleAttr(self.ctrl, 'side_to_side')
        attribute.addAngleAttr(self.ctrl, 'front_to_back')

    def addSystems(self):
        self.ctrl.side_to_side.connect(self.outer_srt.rz)
        pm.transformLimits(self.outer_srt, erz=(0, 1), rz=(-180, 0))
        self.ctrl.side_to_side.connect(self.inner_srt.rz)
        pm.transformLimits(self.inner_srt, erz=(1, 0), rz=(0, 180))
        self.ctrl.front_to_back.connect(self.front_srt.rx)
        pm.transformLimits(self.front_srt, erx=(1, 0), rx=(0, 180))
        self.ctrl.front_to_back.connect(self.back_srt.rx)
        pm.transformLimits(self.back_srt, erx=(0, 1), rx=(-180, 0))


    def finish(self):
        self.setColours(self.guide)

        nodes = [node for node in self.controls_list if not node == self.params]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        attrList = ['visibility']
        attribute.channelControl(nodeList=self.controls_list, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TCockler(guide)

