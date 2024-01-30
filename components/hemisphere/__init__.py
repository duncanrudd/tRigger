from tRigger import components
from tRigger.core import attribute, transform, dag, mathOps, icon
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)

import pymel.core as pm
import math

class THemisphere(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'hemisphere')
        print 'Created Hemisphere Component: %s' % self.comp_name

    def addObjects(self, guide):
        start = guide.locs[1]
        self.radius = mathOps.getDistance(guide.root, start)

        # place base srt
        transform.bakeSrtToOffsetParentMtx(self.base_srt)
        #self.base_srt.offsetParentMatrix.set(guide.root.worldMatrix[0].get())

        # Generate controls
        xform = self.base_srt.worldMatrix[0].get()
        self.invert = (self.guide.guide_side == 'R')
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        self.ctrl = self.addCtrl(shape='arrowRight', size=self.radius,
                                 name=self.getName('main'), xform=xform, parent=self.base_srt,
                                 metaParent=self.base_srt)

        self.mapToGuideLocs(self.controls_list[1], guide.locs[0])

        # Add output srts
        self.srts = []
        self.rig_srt = dag.addChild(self.rig, 'group', name=self.getName('rig_srt'))
        self.rig_srt.s.set((self.radius, self.radius, self.radius))
        self.rig_srt.offsetParentMatrix.set(self.ctrl.worldMatrix[0].get())
        srtGrp = self.rig_srt
        if self.invert:
            invertSrt = dag.addChild(self.rig_srt, 'group', name=self.getName('negScale_srt'))
            invertSrt.s.set(1, 1, -1)
            srtGrp = invertSrt
        if guide.local_rig:
                self.ctrl.t.connect(self.rig_srt.t)
                self.ctrl.r.connect(self.rig_srt.r)
                self.ctrl.ro.connect(self.rig_srt.ro)
        else:
            self.ctrl.worldMatrix[0].connect(self.rig_srt.offsetParentMatrix)
        for i in range(self.guide.num_spans):
            num = str(i+1).zfill(2)
            srt = dag.addChild(srtGrp, 'group', name=self.getName('%s_srt' % num))
            self.srts.append(srt)

        # Add joints
        if guide.add_joint:
            for srt in self.srts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                if srt != self.srts[0]:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': srt})

        components.TBaseComponent.addObjects(self, guide)

    def addAttributes(self):
        attribute.addFloatAttr(self.ctrl, 'height', minValue=-1, maxValue=1, value=1)

    def addSystems(self):
        for index, srt in enumerate(self.srts):
            param = ((1.0 / (self.guide.num_spans -1) * index)*2)-1
            num = str(index+1).zfill(2)
            range = mathOps.remap(self.ctrl.height, -1, 1, -1, param, name=self.getName('%s_range' % num))
            uc = mathOps.convert(range.outValueX, math.pi, name=self.getName('%s_mult' % num))
            euler2Quat = pm.createNode('eulerToQuat', name=self.getName('%s_euler2Quat' % num))
            uc.output.connect(euler2Quat.inputRotateX)
            posAttr = euler2Quat.outputQuatX
            if self.guide.flip:
                flipPos = mathOps.multiply(posAttr, -1, name=self.getName('%s_flip' % num))
                posAttr = flipPos.output
            posAttr.connect(srt.tx)
            euler2Quat.outputQuatW.connect(srt.sy)
            euler2Quat.outputQuatW.connect(srt.sz)



    def finish(self):
        self.setColours(self.guide)

        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodes = [node for node in self.controls_list if not node == self.params]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        spaceAttrs = [attr for attr in ['orbit_ctrl_parent_space', 'orbit_ctrl_translate_space',
                                        'orbit_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.orbit_ctrl)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return THemisphere(guide)

