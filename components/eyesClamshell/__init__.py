from tRigger import components
import math
from tRigger.core import attribute, transform, dag, mathOps, curve, icon
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)
reload(curve)

import pymel.core as pm

class TEyesClamshell(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'eyesClamshell')
        print 'Created EyesClamshell Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = guide.guide_side == 'R'
        if self.invert:
            self.negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
        ctrlSize = 5
        self.rig_srt = dag.addChild(self.rig, 'group', self.getName('rig_base_srt'))
        self.rig_srt.offsetParentMatrix.set(self.base_srt.worldMatrix[0].get())

        self.ctrl = self.addCtrl(shape='box', size=ctrlSize*5,
                                 name=self.getName(''), xform=self.base_srt.worldMatrix[0].get(),
                                 parent=self.base_srt, metaParent=self.base_srt)
        self.eyeball_ctrl = self.addCtrl(shape='arrowForward', size=ctrlSize*5,
                                 name=self.getName('eyeball'), xform=self.base_srt.worldMatrix[0].get(),
                                 parent=self.controls, metaParent=self.ctrl)
        if self.invert:
            self.ctrl.offsetParentMatrix.set(self.negMtx.outputMatrix.get() * self.ctrl.offsetParentMatrix.get())
        self.aimCtrl = self.addCtrl(shape='circlePoint', size=ctrlSize*5,
                                 name=self.getName('aim'), xform=guide.locs[1].worldMatrix[0].get(),
                                 parent=self.controls, metaParent=self.ctrl)
        self.tugSrt = dag.addChild(self.rig, 'group', self.getName('tug_srt'))
        self.tugSrt.offsetParentMatrix.set(self.base_srt.worldMatrix[0].get())

        if self.guide.add_joint:
            root_j = pm.createNode('joint', name=self.getName('root_jnt'))
            driver = self.ctrl
            if self.invert:
                self.root_srt = dag.addChild(self.rig, 'group', self.getName('root_out_srt'))
                rootNegMtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, self.ctrl.worldMatrix[0]],
                                                      name=self.getName('root_neg_mtx'))
                rootNegMtx.matrixSum.connect(self.root_srt.offsetParentMatrix)
                driver = self.root_srt
            self.joints_list.append({'joint': root_j, 'driver': driver})
            eyeball_j = pm.createNode('joint', name=self.getName('eyeball_jnt'))
            eyeball_j.setParent(root_j)
            self.joints_list.append({'joint': eyeball_j, 'driver': self.eyeball_ctrl})
            tug_j = pm.createNode('joint', name=self.getName('tug_jnt'))
            tug_j.setParent(root_j)
            self.joints_list.append({'joint': tug_j, 'driver': self.tugSrt})

        # GUIDE TO RIG MAPPING
        self.mapToGuideLocs(self.ctrl, guide.locs[0])
        self.mapToGuideLocs(self.aimCtrl, guide.locs[1])

        # Add hemisphere output srts
        self.upper_srts = []
        self.upper_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('upper_srt'))
        self.upperRadius = mathOps.getDistance(guide.root, guide.locs[2])
        self.upper_srt.s.set((self.upperRadius, self.upperRadius, self.upperRadius))

        self.lower_srts = []
        self.lower_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('lower_srt'))
        self.lowerRadius = mathOps.getDistance(guide.root, guide.locs[3])
        self.lower_srt.s.set((self.lowerRadius, self.lowerRadius, self.lowerRadius))

        self.eyeball_srts = []
        self.eyeball_srt = dag.addChild(self.rig, 'group', name=self.getName('eyeball_srt'))
        self.eyeballRadius = mathOps.getDistance(guide.root, guide.locs[4])
        self.eyeball_srt.s.set((self.eyeballRadius, self.eyeballRadius, self.eyeballRadius))

        for i in range(self.guide.lidSpans):
            num = str(i + 1).zfill(2)
            srt = dag.addChild(self.upper_srt, 'group', name=self.getName('%s_upper_srt' % num))
            self.upper_srts.append(srt)

            srt = dag.addChild(self.lower_srt, 'group', name=self.getName('%s_lower_srt' % num))
            self.lower_srts.append(srt)

        for i in range(self.guide.ballSpans):
            num = str(i + 1).zfill(2)
            srt = dag.addChild(self.eyeball_srt, 'group', name=self.getName('%s_eyeball_srt' % num))
            self.eyeball_srts.append(srt)

        # Add joints
        if guide.add_joint:
            for srt in self.upper_srts + self.lower_srts + self.eyeball_srts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                if srt != self.eyeball_srts[0] or srt != self.lower_srts[0]:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': srt})

    def addAttributes(self):
        attribute.addDividerAttr(self.params, 'UPPER')
        attribute.addAngleAttr(self.params, 'upper_lid')
        attribute.addAngleAttr(self.params, 'upper_twist')
        attribute.addFloatAttr(self.params, 'upper_height', minValue=0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'upper_hide_trigger', minValue=0.0, maxValue=1.0, value=.1)
        attribute.addDividerAttr(self.params, 'LOWER')
        attribute.addAngleAttr(self.params, 'lower_lid')
        attribute.addAngleAttr(self.params, 'lower_twist')
        attribute.addFloatAttr(self.params, 'lower_height', minValue=0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'lower_hide_trigger', minValue=0.0, maxValue=1.0, value=.1)
        attribute.addDividerAttr(self.params, 'SHARED')
        attribute.addFloatAttr(self.params, 'dilate', minValue=0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'blink', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'blink_height', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'auto_lids', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'skin_tug', minValue=0.0, maxValue=1.0)

        attribute.addBoolAttr(self.output, 'upper_vis')
        attribute.addBoolAttr(self.output, 'lower_vis')

    def addSystems(self):
        # AIM STUFF
        aimMtx = transform.createNonRollMatrix(self.ctrl, self.aimCtrl, axis='z',
                                               name=[self.getName('aim_orbit_mtx'),
                                                     self.getName('aim_elevation_mtx')])
        mtxAttr = aimMtx[1].outputMatrix
        if self.invert:
            negAimMtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, mtxAttr],
                                                 name=self.getName('neg_aim_mtx'))
            mtxAttr = negAimMtx.matrixSum
        mtxAttr.connect(self.eyeball_ctrl.offsetParentMatrix)

        mtxAttr = self.ctrl.worldMatrix[0]
        if self.invert:
            negAimMtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, mtxAttr],
                                                 name=self.getName('neg_tug_mtx'))
            mtxAttr = negAimMtx.matrixSum
        offset = transform.pmMtx2fourFourMtx(mtxAttr.get() * self.eyeball_ctrl.worldInverseMatrix[0].get(),
                                             name=self.getName('ctrl2Aim_offset_mtx'))
        ctrlMtx = mathOps.multiplyMatrices([offset.output, self.eyeball_ctrl.worldMatrix[0]],
                                           name=self.getName('tug_ctrl_mtx'))
        tugMtx = transform.blendMatrices(mtxAttr, ctrlMtx.matrixSum,
                                         name=self.getName('tug_mtx'))
        self.params.skin_tug.connect(tugMtx.target[0].weight)
        tugMtx.outputMatrix.connect(self.tugSrt.offsetParentMatrix)

        ctrlMtx = self.eyeball_ctrl.worldMatrix[0]
        if self.invert:
            mtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, ctrlMtx],
                                           name=self.getName('eyeball_neg_mtx'))
            ctrlMtx = mtx.matrixSum
            self.rig_srt.sx.set(-1)
        autoMtx = mathOps.multiplyMatrices([ctrlMtx, self.ctrl.worldInverseMatrix[0]],
                                           name=self.getName('auto_mtx'))
        autoMtx2Srt = mathOps.decomposeMatrix(autoMtx.matrixSum, name=self.getName('auto_mtx2Srt'))
        autoMult = mathOps.multiplyAngleByScalar(autoMtx2Srt.outputRotateX, self.params.auto_lids,
                                                 name=self.getName('auto_mult'))
        autoMult.output.connect(self.rig_srt.rx)
        tugMult = mathOps.multiplyAngleByScalar(autoMtx2Srt.outputRotateY, self.params.skin_tug,
                                                 name=self.getName('tug_mult'))
        tugMult.output.connect(self.rig_srt.ry)

        if not self.guide.root.local_rig.get():
            self.ctrl.worldMatrix[0].connect(self.rig_srt.offsetParentMatrix)
            self.eyeball_ctrl.worldMatrix[0].connect(self.eyeball_srt.offsetParentMatrix)
        else:
            self.eyeball_ctrl.r.connect(self.eyeball_srt.r)
            if self.invert:
                self.rig_srt.offsetParentMatrix.set(self.negMtx.outputMatrix.get()*self.rig_srt.offsetParentMatrix.get())

        # Blinks
        blinkBlend = mathOps.pairBlend(name=self.getName('blink_blend'), weight=self.params.blink_height)
        self.params.upper_lid.connect(blinkBlend.inRotateX2)
        upperAttr = self.params.upper_twist
        if self.invert:
            inv = mathOps.multiply(upperAttr, -1, name=self.getName('upper_twist_invert'))
            upperAttr = inv.output
        upperAttr.connect(blinkBlend.inRotateZ2)
        self.params.lower_lid.connect(blinkBlend.inRotateX1)
        lowerAttr = self.params.lower_twist
        if self.invert:
            inv = mathOps.multiply(lowerAttr, -1, name=self.getName('lower_twist_invert'))
            lowerAttr = inv.output
        lowerAttr.connect(blinkBlend.inRotateZ1)

        blinkUpper = mathOps.pairBlend(rotateA=(0, 0, 0), rotateB=blinkBlend.outRotate, weight=self.params.blink,
                                       name=self.getName('blink_upper_Blend'))
        self.params.upper_lid.connect(blinkUpper.inRotateX1)
        upperAttr.connect(blinkUpper.inRotateZ1)

        blinkLower = mathOps.pairBlend(rotateA=(0, 0, 0), rotateB=blinkBlend.outRotate, weight=self.params.blink,
                                       name=self.getName('blink_lower_Blend'))
        self.params.lower_lid.connect(blinkLower.inRotateX1)
        lowerAttr.connect(blinkLower.inRotateZ1)

        blinkUpper.outRotate.connect(self.upper_srt.r)
        blinkLower.outRotate.connect(self.lower_srt.r)

        heightBlend = mathOps.blendScalarAttrs(self.params.lower_height, self.params.upper_height,
                                               blend=self.params.blink_height,
                                               name=self.getName('blink_height_blend'))
        upperHeight = mathOps.blendScalarAttrs(self.params.upper_height, heightBlend.output,
                                               blend=self.params.blink,
                                               name=self.getName('upper_height_blend'))
        lowerHeight = mathOps.blendScalarAttrs(self.params.lower_height, heightBlend.output,
                                               blend=self.params.blink,
                                               name=self.getName('lower_height_blend'))

        # Hemisphere stuff
        for index, srt in enumerate(self.upper_srts):
            param = ((1.0 / (self.guide.lidSpans -1) * index)*2)-1
            num = str(index+1).zfill(2)
            range = mathOps.remap(upperHeight.output, 0, 1, param, -1, name=self.getName('upper_%s_range' % num))
            uc = mathOps.convert(range.outValueX, math.pi, name=self.getName('upper_%s_mult' % num))
            euler2Quat = pm.createNode('eulerToQuat', name=self.getName('upper_%s_euler2Quat' % num))
            uc.output.connect(euler2Quat.inputRotateX)
            flipPos = mathOps.multiply(euler2Quat.outputQuatX, -1, name=self.getName('lower_%s_flip' % num))
            posAttr = flipPos.output
            posAttr.connect(srt.ty)
            euler2Quat.outputQuatW.connect(srt.sx)
            euler2Quat.outputQuatW.connect(srt.sz)

        for index, srt in enumerate(self.lower_srts):
            param = ((1.0 / (self.guide.lidSpans -1) * index)*2)-1
            num = str(index+1).zfill(2)
            range = mathOps.remap(lowerHeight.output, 0, 1, -1, param,name=self.getName('lower_%s_range' % num))
            uc = mathOps.convert(range.outValueX, math.pi, name=self.getName('lower_%s_mult' % num))
            euler2Quat = pm.createNode('eulerToQuat', name=self.getName('lower_%s_euler2Quat' % num))
            uc.output.connect(euler2Quat.inputRotateX)
            posAttr = euler2Quat.outputQuatX
            posAttr.connect(srt.ty)
            euler2Quat.outputQuatW.connect(srt.sx)
            euler2Quat.outputQuatW.connect(srt.sz)

        for index, srt in enumerate(self.eyeball_srts):
            param = ((1.0 / (self.guide.ballSpans -1) * index)*2)-1
            num = str(index+1).zfill(2)
            range = mathOps.remap(self.params.dilate, 0, 1, param, -1, name=self.getName('eyeball_%s_range' % num))
            uc = mathOps.convert(range.outValueX, math.pi, name=self.getName('eyeball_%s_mult' % num))
            euler2Quat = pm.createNode('eulerToQuat', name=self.getName('eyeball_%s_euler2Quat' % num))
            uc.output.connect(euler2Quat.inputRotateX)
            posAttr = euler2Quat.outputQuatX
            posAttr.connect(srt.tz)
            euler2Quat.outputQuatW.connect(srt.sx)
            euler2Quat.outputQuatW.connect(srt.sy)

        # Auto hide for lids
        upperSwitch = pm.createNode('condition', name=self.getName('upper_vis_switch'))
        upperHeight.output.connect(upperSwitch.firstTerm)
        self.params.upper_hide_trigger.connect(upperSwitch.secondTerm)
        upperSwitch.operation.set(2)
        upperSwitch.outColorR.connect(self.output.upper_vis)

        lowerSwitch = pm.createNode('condition', name=self.getName('lower_vis_switch'))
        lowerHeight.output.connect(lowerSwitch.firstTerm)
        self.params.lower_hide_trigger.connect(lowerSwitch.secondTerm)
        lowerSwitch.operation.set(4)
        lowerSwitch.outColorR.connect(self.output.lower_vis)

        # Attach params shape to eyeball_ctrl
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.ctrl, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):
        self.setColours(self.guide)

        attrList = ['visibility']
        nodeList = [self.ctrl]
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['visibility', 'sx', 'sy', 'sz', 'tx', 'ty', 'tz']
        nodeList = [self.eyeball_ctrl]
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['visibility', 'sx', 'sy', 'sz', 'rx', 'ry', 'rz']
        nodeList = [self.aimCtrl]
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = self.controls_list[4:]
        attrList = ['visibility', 'rx', 'ry', 'rz', 'sx', 'sy', 'sz', 'tz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TEyesClamshell(guide)

