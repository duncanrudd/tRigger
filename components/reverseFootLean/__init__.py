from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps, anim
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)
reload(anim)

import pymel.core as pm

# 0 [nt.Transform(u'foot_C0_root_guide'),
# 1 nt.Transform(u'foot_C0_heel_guide'),
# 2 nt.Transform(u'foot_C0_ball_guide'),
# 3 nt.Transform(u'foot_C0_tip_guide'),
# 4 nt.Transform(u'foot_C0_toe_guide'),
# 5 nt.Transform(u'foot_C0_outer_guide'),
# 6 nt.Transform(u'foot_C0_inner_guide'),
# 7 nt.Transform(u'foot_C0_tarsi_02_guide')]


class TReverseFootLean(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'reverseFootLean')
        self.invert = (guide.guide_side == 'R')
        print 'Created reverseFootLean Component: %s' % self.comp_name

    def addObjects(self, guide):
        ctrlSize = mathOps.getDistance(guide.locs[1], guide.locs[3])*.25

        def _getVec(start, end, invert=0):
            start, end = mathOps.getStartAndEnd(start, end)
            if invert:
                return (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
            else:
                return (pm.datatypes.Vector(start) - pm.datatypes.Vector(end)).normal()

        # ----------------------
        # IK controls
        # ----------------------
        self.ik_base_srt = dag.addChild(self.controls, 'group', self.getName('ik_base_srt'))
        transform.align(self.ik_base_srt, self.base_srt)
        ikHeelXform = guide.locs[1].worldMatrix[0].get()
        ikBallXform = guide.locs[2].worldMatrix[0].get()
        ikTipXform = guide.locs[3].worldMatrix[0].get()
        ikLeanXform = guide.locs[7].worldMatrix[0].get()
        if self.invert:
            ikHeelXform = mathOps.invertHandedness(ikHeelXform)
            ikBallXform = mathOps.invertHandedness(ikBallXform)
            ikTipXform = mathOps.invertHandedness(ikTipXform)
            ikLeanXform = mathOps.invertHandedness(ikLeanXform)
        self.ikHeel_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                        name=self.getName('ik_heel'), xform=ikHeelXform,
                                        parent=self.ik_base_srt, metaParent=self.base_srt)

        self.ikInner_srt = dag.addChild(self.ikHeel_ctrl, 'group', self.getName('ik_inner_srt'))
        pm.xform(self.ikInner_srt, ws=1, m=mathOps.getInverseHandedMatrix(guide.locs[6].worldMatrix[0].get()))
        self.ikInner_srt.s.set(1, 1, 1)
        self.ikOuter_srt = dag.addChild(self.ikInner_srt, 'group', self.getName('ik_outer_srt'))
        pm.xform(self.ikOuter_srt, ws=1, m=mathOps.getInverseHandedMatrix(guide.locs[5].worldMatrix[0].get()))
        self.ikOuter_srt.s.set(1, 1, 1)
        self.ikBall_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                        name=self.getName('ik_ball'), xform=ikBallXform,
                                        parent=self.ikOuter_srt, metaParent=self.ikHeel_ctrl, buffer=1)
        self.ikBall_ctrl.getParent().s.set(1, 1, 1)
        self.ikTip_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                       name=self.getName('ik_tip'), xform=ikTipXform,
                                       parent=self.ikBall_ctrl, metaParent=self.ikBall_ctrl)
        # Build toe matrix
        aimVec = _getVec(guide.locs[4], guide.locs[3], 0)
        upVecTemp = mathOps.getMatrixAxisAsVector(self.ikTip_ctrl.worldMatrix[0].get(), 'z')
        sideVec = upVecTemp.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(guide.locs[4], q=1, ws=1, t=1)


        if self.invert:
            toeXform = pm.datatypes.Matrix(aimVec, -sideVec, upVec, startPos)
        else:
            toeXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)


        self.ikToe_ctrl = self.addCtrl(shape='pringle', size=ctrlSize,
                                       name=self.getName('ik_toe'), xform=toeXform, buffer=1,
                                       parent=self.ikTip_ctrl, metaParent=self.ikTip_ctrl)
        # self.ikToe_ctrl.s.set(1, 1, 1)

        # Lean control
        self.ikLean_ctrl = self.addCtrl(shape='pringle', size=ctrlSize,
                                       name=self.getName('ik_lean'), xform=ikLeanXform, buffer=1,
                                       parent=self.ikToe_ctrl, metaParent=self.ikToe_ctrl)

        # End srt - this is what will be measured against the root to determine ik ankle displacement
        self.ikEnd_srt = dag.addChild(self.controls_list[-1], 'group', name=self.getName('ik_end_srt'))
        xform = self.base_srt.worldMatrix[0].get()
        if self.invert:
            xform = mathOps.getInverseHandedMatrix(xform)
        pm.xform(self.ikEnd_srt, ws=1, m=xform)

        # ----------------------
        # FK controls
        # ----------------------
        parent = self.base_srt
        if self.invert:
            self.fk_negScale_srt = dag.addChild(self.base_srt, 'group', name=self.getName('fk_negScale_srt'))
            parent = self.fk_negScale_srt
            transform.align(parent, self.ikEnd_srt, scale=1)
        self.fkToe_ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.67,
                                       name=self.getName('fk_toe'), xform=toeXform, buffer=1,
                                       parent=parent, metaParent=self.base_srt)

        if guide.root.add_joint.get():
            if self.invert:
                self.negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
            j = pm.createNode('joint', name=self.getName('base_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.base_srt})

            j = pm.createNode('joint', name=self.getName('toe_jnt'))
            j.setParent(self.joints_list[0]['joint'])
            driver = self.fkToe_ctrl
            if self.invert:
                out_srt = dag.addChild(self.rig, 'group', self.getName('out_toe_srt'))
                out_mtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, driver.worldMatrix[0]],
                                                   name=self.getName('out_toe_mtx'))
                out_mtx.matrixSum.connect(out_srt.offsetParentMatrix)

                # Add mapping connection to control - used when mapping controller tags
                self.mapToControl(driver, out_srt)

                driver = out_srt
            self.joints_list.append({'joint': j, 'driver': driver})

            j = pm.createNode('joint', name=self.getName('lean_jnt'))
            j.setParent(self.joints_list[0]['joint'])
            self.lean_srt = dag.addChild(self.rig, 'group', self.getName('lean_srt'))
            self.joints_list.append({'joint': j, 'driver': self.lean_srt})

        # MAP TO GUIDE LOCS
        mappingPairs = [[self.ik_base_srt, guide.locs[1]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'foot_scale', value=1.0)
        #attribute.addFloatAttr(self.params, 'ikfk_blend', minValue=0, maxValue=1)
        if self.guide.attr_driven:
            attribute.addDividerAttr(self.params, 'Roll')
            for attr in ['roll_foot', 'toe_lift_angle', 'ball_straight_angle']:
                attribute.addFloatAttr(self.params, attr)
            for attr in  ['roll_heel', 'roll_ball', 'roll_tip']:
                attribute.addAngleAttr(self.params, attr)

            attribute.addDividerAttr(self.params, 'Spin')
            for attr in  ['spin_heel', 'spin_ball', 'spin_tip']:
                attribute.addAngleAttr(self.params, attr)

            attribute.addDividerAttr(self.params, 'Lean')
            for attr in ['lean_side', 'lean_front', 'lean_edge']:
                attribute.addAngleAttr(self.params, attr)

            attribute.addDividerAttr(self.params, 'Twist')
            for attr in ['twist_heel', 'twist_ball', 'twist_tip']:
                attribute.addAngleAttr(self.params, attr)


    def addSystems(self):
        if self.guide.attr_driven:
            footRollNegClamp = mathOps.clamp(self.params.roll_foot, -1000, 0, name=self.getName('footRoll_neg_clamp'))
            rollHeelSum = mathOps.addAngles(footRollNegClamp.outputR, self.params.roll_heel,
                                            name=self.getName('roll_heel_sum'))
            rollHeelSum.output.connect(self.ikHeel_ctrl.rz)

            footRollPosClamp = mathOps.clamp(self.params.roll_foot, 0, 1000, name=self.getName('footRoll_pos_clamp'))
            footRollBallRemap = mathOps.remap(footRollPosClamp.outputR, self.params.toe_lift_angle,
                                              self.params.ball_straight_angle, 1, 0,
                                              name=self.getName('footRoll_ball_remap'))
            rollBallSum = mathOps.addAngles(footRollPosClamp.outputR, self.params.roll_ball,
                                            name=self.getName('roll_ball_sum'))
            footRollBallEase = anim.easeCurve(footRollBallRemap.outValueX,
                                              name=self.getName('footRoll_ball_ease_animCurve'))
            footRollBallEase.output.connect(rollBallSum.weightA)
            rollBallSum.output.connect(self.ikToe_ctrl.rz)

            footRollToeMult = mathOps.reverse(footRollBallRemap.outValueX, name=self.getName('footRoll_toe_remap'))
            rollToeSum = mathOps.addAngles(footRollPosClamp.outputR, self.params.roll_tip,
                                            name=self.getName('roll_ball_sum'))
            footRollToeEase = anim.easeCurve(footRollToeMult.outputX, easeOut=0,
                                              name=self.getName('footRoll_ball_ease_animCurve'))
            footRollToeEase.output.connect(rollToeSum.weightA)
            rollToeSum.output.connect(self.ikTip_ctrl.rz)

            self.params.spin_heel.connect(self.ikHeel_ctrl.ry)
            self.params.spin_ball.connect(self.ikBall_ctrl.ry)
            self.params.spin_tip.connect(self.ikTip_ctrl.ry)

            self.params.twist_heel.connect(self.ikHeel_ctrl.rx)
            self.params.twist_ball.connect(self.ikToe_ctrl.rx)
            self.params.twist_tip.connect(self.ikTip_ctrl.rx)

            self.params.lean_edge.connect(self.ikBall_ctrl.rx)
            self.params.lean_side.connect(self.ikLean_ctrl.rx)
            self.params.lean_front.connect(self.ikLean_ctrl.rz)
                
        # Ik foot edge roll system
        buffer = self.ikBall_ctrl.getParent()
        sideRollNeg = mathOps.multiplyAngleByScalar(self.ikBall_ctrl.rx, -1, name=self.getName('ik_sideRoll_invert'))
        sideRollNeg.output.connect(buffer.rx)
        inner, outer = self.ikInner_srt, self.ikOuter_srt
        if not self.invert:
            outer, inner = self.ikInner_srt, self.ikOuter_srt

        pm.transformLimits(inner, rx=(0, 1000), erx=(1, 0))
        pm.transformLimits(outer, rx=(-1000, 0), erx=(0, 1))
        rAttr = self.ikBall_ctrl.rx
        if self.invert:
            rAttr = sideRollNeg.output
        rAttr.connect(self.ikInner_srt.rx)
        rAttr.connect(self.ikOuter_srt.rx)

        self.params.foot_scale.connect(self.ik_base_srt.sx)
        self.params.foot_scale.connect(self.ik_base_srt.sy)
        self.params.foot_scale.connect(self.ik_base_srt.sz)
        self.params.foot_scale.connect(self.base_srt.sx)
        self.params.foot_scale.connect(self.base_srt.sy)
        self.params.foot_scale.connect(self.base_srt.sz)

        # Fk toe
        footMtx = mathOps.multiplyMatrices([self.ikToe_ctrl.worldMatrix[0], self.ikEnd_srt.worldInverseMatrix[0]],
                                           name=self.getName('fk_foot_offset_mtx'))
        toeMtx = mathOps.multiplyMatrices([self.ikTip_ctrl.worldMatrix[0], self.ikToe_ctrl.worldInverseMatrix[0]],
                                           name=self.getName('fk_toe_offset_mtx'))
        toeRotMtx = transform.blend_T_R_matrices(pm.datatypes.Matrix(), toeMtx.matrixSum,
                                                    name=self.getName('fk_toe_rot_mtx'))
        tempDM = mathOps.decomposeMatrix(self.fkToe_ctrl.getParent().offsetParentMatrix)
        self.fkToe_ctrl.getParent().rz.set(180 - tempDM.outputRotateZ.get())
        pm.delete(tempDM)
        footMtx.matrixSum.connect(self.fkToe_ctrl.getParent().offsetParentMatrix)
        toeRotMtx.outputMatrix.connect(self.fkToe_ctrl.offsetParentMatrix)
        # Attach params shape to base srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.base_srt, lw=1, wt=1)
        pm.delete(tempJoint)

        # fk lean
        if self.invert:
            leanOffsetMtx = mathOps.multiplyMatrices([self.ikLean_ctrl.getParent().worldMatrix[0],
                                                      self.ikEnd_srt.worldInverseMatrix[0],
                                                      self.fk_negScale_srt.worldMatrix[0]],
                                                     name=self.getName('fk_lean_offset_mtx'))
        else:
            leanOffsetMtx = mathOps.multiplyMatrices([self.ikLean_ctrl.getParent().worldMatrix[0],
                                                      self.ikEnd_srt.worldInverseMatrix[0],
                                                      self.base_srt.worldMatrix[0]],
                                                     name=self.getName('fk_lean_offset_mtx'))
        leanOffsetMtx.matrixSum.connect(self.lean_srt.offsetParentMatrix)

    def finish(self):
        self.setColours(self.guide)

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodes = [self.fkToe_ctrl]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = [node for node in self.controls_list if '_ik_' in node.name()]
        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        if self.guide.attr_driven:
            self.ikHeel_ctrl.getParent().v.set(0)
            for ctrl in [self.ikHeel_ctrl, self.ikTip_ctrl, self.ikBall_ctrl, self.ikToe_ctrl, self.ikLean_ctrl]:
                ctrl.is_tControl.set(0)




def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TReverseFootLean(guide)

