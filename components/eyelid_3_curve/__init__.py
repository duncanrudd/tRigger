from tRigger import components
import math
from tRigger.core import attribute, transform, dag, mathOps, curve, icon
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)
reload(curve)

import pymel.core as pm

class TEyelid_3_curve(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'eyelid_3_curve')
        print 'Created Eyelid_3_curve Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = guide.guide_side == 'R'
        if self.invert:
            self.negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))

        ctrlSize = mathOps.getDistance(guide.root, guide.locs[-1]) * .01

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
                                 name=self.getName('aim'), xform=guide.locs[-1].worldMatrix[0].get(),
                                 parent=self.controls, metaParent=self.ctrl)
        self.tugSrt = dag.addChild(self.rig, 'group', self.getName('tug_srt'))
        self.tugSrt.offsetParentMatrix.set(self.base_srt.worldMatrix[0].get())
        
        offsets = [-ctrlSize, 0, ctrlSize]
        if self.invert:
            offsets = [ctrlSize, 0, -ctrlSize]

        self.upper_inner_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('upper_inner_srt'))
        self.upper_mid_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('upper_mid_srt'))
        self.upper_outer_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('upper_outer_srt'))

        for index, srt in enumerate([self.upper_inner_srt, self.upper_mid_srt, self.upper_outer_srt]):
            srt.tx.set(offsets[index])
            srt.ty.set(ctrlSize*.1)
            srt.tz.set(ctrlSize)

        self.lower_inner_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('lower_inner_srt'))
        self.lower_mid_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('lower_mid_srt'))
        self.lower_outer_srt = dag.addChild(self.rig_srt, 'group', name=self.getName('lower_outer_srt'))

        for index, srt in enumerate([self.lower_inner_srt, self.lower_mid_srt, self.lower_outer_srt]):
            srt.tx.set(offsets[index])
            srt.ty.set(ctrlSize*-.1)
            srt.tz.set(ctrlSize)

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
            for srt in [self.upper_inner_srt, self.upper_outer_srt, self.upper_mid_srt,
                        self.lower_inner_srt, self.lower_mid_srt, self.lower_outer_srt]:
                j = pm.createNode('joint', name=self.getName(srt.name().replace('_srt', '_jnt')))
                self.joints_list.append({'joint': j, 'driver': srt})

        # GUIDE TO RIG MAPPING
        self.mapToGuideLocs(self.aimCtrl, guide.locs[-1])

    def addAttributes(self):
        attribute.addAngleAttr(self.params, 'upper_inner')
        attribute.addAngleAttr(self.params, 'upper_outer')
        attribute.addAngleAttr(self.params, 'upper_lid')
        attribute.addAngleAttr(self.params, 'lower_lid')
        attribute.addAngleAttr(self.params, 'lower_inner')
        attribute.addAngleAttr(self.params, 'lower_outer')
        attribute.addFloatAttr(self.params, 'blink', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'blink_height', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'auto_lids', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'skin_tug', minValue=0.0, maxValue=1.0)

    def addSystems(self):
        # AIM STUFF
        aimMtx = transform.createNonRollMatrix(self.ctrl, self.aimCtrl, axis='z',
                                               name=[self.getName('aim_orbit_mtx'),
                                                     self.getName('aim_elevation_mtx')])
        mtxAttr = aimMtx[1].outputMatrix
        if self.invert:
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
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
        autoMtx = mathOps.multiplyMatrices([self.eyeball_ctrl.worldMatrix[0], self.tugSrt.worldInverseMatrix[0]],
                                           name=self.getName('auto_mtx'))
        autoMtx2Srt = mathOps.decomposeMatrix(autoMtx.matrixSum, name=self.getName('auto_mtx2Srt'))
        autoMult = mathOps.multiplyAngleByScalar(autoMtx2Srt.outputRotateX, self.params.auto_lids,
                                                 name=self.getName('auto_mult'))
        autoMult.output.connect(self.rig_srt.rx)

        ##################################################################################################
        # LIDS
        blink_height_rev = mathOps.reverse(self.params.blink_height, name=self.getName('blinkHeight_rev'))
        blink_rev = mathOps.reverse(self.params.blink, name=self.getName('blinkHeight_rev'))

        # Inner
        upperInnerSum = mathOps.addAngles(self.params.upper_lid, self.params.upper_inner,
                                          name=self.getName('upper_inner_sum'))
        lowerInnerSum = mathOps.addAngles(self.params.lower_lid, self.params.lower_inner,
                                          name=self.getName('lower_inner_sum'))
        inner_blend = mathOps.addAngles(upperInnerSum.output, lowerInnerSum.output,
                                          name=self.getName('inner_blend'))
        self.params.blink_height.connect(inner_blend.weightA)
        blink_height_rev.outputX.connect(inner_blend.weightB)
        upper_inner_blend = mathOps.addAngles(upperInnerSum.output, inner_blend.output,
                                          name=self.getName('upper_inner_blend'))
        blink_rev.outputX.connect(upper_inner_blend.weightA)
        self.params.blink.connect(upper_inner_blend.weightB)
        lower_inner_blend = mathOps.addAngles(lowerInnerSum.output, inner_blend.output,
                                              name=self.getName('lower_inner_blend'))
        blink_rev.outputX.connect(lower_inner_blend.weightA)
        self.params.blink.connect(lower_inner_blend.weightB)

        upperInnerMtx = pm.createNode('composeMatrix', name=self.getName('upper_inner_mtx'))
        upper_inner_blend.output.connect(upperInnerMtx.inputRotateX)
        upperInnerMtx.outputMatrix.connect(self.upper_inner_srt.offsetParentMatrix)

        lowerInnerMtx = pm.createNode('composeMatrix', name=self.getName('lower_inner_mtx'))
        lower_inner_blend.output.connect(lowerInnerMtx.inputRotateX)
        lowerInnerMtx.outputMatrix.connect(self.lower_inner_srt.offsetParentMatrix)
        
        # Outer
        upperOuterSum = mathOps.addAngles(self.params.upper_lid, self.params.upper_outer,
                                          name=self.getName('upper_outer_sum'))
        lowerOuterSum = mathOps.addAngles(self.params.lower_lid, self.params.lower_outer,
                                          name=self.getName('lower_outer_sum'))
        outer_blend = mathOps.addAngles(upperOuterSum.output, lowerOuterSum.output,
                                        name=self.getName('outer_blend'))
        self.params.blink_height.connect(outer_blend.weightA)
        blink_height_rev.outputX.connect(outer_blend.weightB)
        upper_outer_blend = mathOps.addAngles(upperOuterSum.output, outer_blend.output,
                                              name=self.getName('upper_outer_blend'))
        blink_rev.outputX.connect(upper_outer_blend.weightA)
        self.params.blink.connect(upper_outer_blend.weightB)
        lower_outer_blend = mathOps.addAngles(lowerOuterSum.output, outer_blend.output,
                                              name=self.getName('lower_outer_blend'))
        blink_rev.outputX.connect(lower_outer_blend.weightA)
        self.params.blink.connect(lower_outer_blend.weightB)

        upperOuterMtx = pm.createNode('composeMatrix', name=self.getName('upper_outer_mtx'))
        upper_outer_blend.output.connect(upperOuterMtx.inputRotateX)
        upperOuterMtx.outputMatrix.connect(self.upper_outer_srt.offsetParentMatrix)

        lowerOuterMtx = pm.createNode('composeMatrix', name=self.getName('lower_outer_mtx'))
        lower_outer_blend.output.connect(lowerOuterMtx.inputRotateX)
        lowerOuterMtx.outputMatrix.connect(self.lower_outer_srt.offsetParentMatrix)

        # Mid
        upperMidSum = mathOps.addAngles(upperInnerSum.output, upperOuterSum.output,
                                          name=self.getName('upper_mid_sum'))
        upperMidSum.weightA.set(0.5)
        upperMidSum.weightB.set(0.5)
        lowerMidSum = mathOps.addAngles(lowerInnerSum.output, lowerOuterSum.output,
                                        name=self.getName('lower_mid_sum'))
        lowerMidSum.weightA.set(0.5)
        lowerMidSum.weightB.set(0.5)
        mid_blend = mathOps.addAngles(upperMidSum.output, lowerMidSum.output,
                                        name=self.getName('mid_blend'))
        self.params.blink_height.connect(mid_blend.weightA)
        blink_height_rev.outputX.connect(mid_blend.weightB)
        upper_mid_blend = mathOps.addAngles(upperMidSum.output, mid_blend.output,
                                              name=self.getName('upper_mid_blend'))
        blink_rev.outputX.connect(upper_mid_blend.weightA)
        self.params.blink.connect(upper_mid_blend.weightB)
        lower_mid_blend = mathOps.addAngles(lowerMidSum.output, mid_blend.output,
                                              name=self.getName('lower_mid_blend'))
        blink_rev.outputX.connect(lower_mid_blend.weightA)
        self.params.blink.connect(lower_mid_blend.weightB)

        upperMidMtx = pm.createNode('composeMatrix', name=self.getName('upper_mid_mtx'))
        upper_mid_blend.output.connect(upperMidMtx.inputRotateX)
        upperMidMtx.outputMatrix.connect(self.upper_mid_srt.offsetParentMatrix)

        lowerMidMtx = pm.createNode('composeMatrix', name=self.getName('lower_mid_mtx'))
        lower_mid_blend.output.connect(lowerMidMtx.inputRotateX)
        lowerMidMtx.outputMatrix.connect(self.lower_mid_srt.offsetParentMatrix)
        
        if not self.guide.root.local_rig.get():
            self.base_srt.worldMatrix[0].connect(self.rig_srt.offsetParentMatrix)

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

        attrList = ['upper_lid', 'upper_inner', 'upper_outer', 'lower_lid',
                    'lower_inner', 'lower_outer', 'blink', 'blink_height',
                    'auto_lids', 'skin_tug']
        for attr in attrList:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.ctrl)

        spaceAttrs = [attr for attr in ['aim_ctrl_parent_space', 'aim_ctrl_translate_space',
                                        'aim_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.aimCtrl)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TEyelid_3_curve(guide)

