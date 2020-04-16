from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)

import pymel.core as pm

# 0 [nt.Transform(u'foot_C0_root_guide'),
# 1 nt.Transform(u'foot_C0_heel_guide'),
# 2 nt.Transform(u'foot_C0_ball_guide'),
# 3 nt.Transform(u'foot_C0_tip_guide'),
# 4 nt.Transform(u'foot_C0_toe_guide'),
# 5 nt.Transform(u'foot_C0_outer_guide'),
# 6 nt.Transform(u'foot_C0_inner_guide'),
# 7 nt.Transform(u'foot_C0_tarsi_02_guide')]


class TReverseFoot(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'reverseFoot')
        self.invert = (guide.guide_side == 'R')
        print 'Created reverseFoot Component: %s' % self.comp_name

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
        if self.invert:
            ikHeelXform = mathOps.invertHandedness(ikHeelXform)
            ikBallXform = mathOps.invertHandedness(ikBallXform)
            ikTipXform = mathOps.invertHandedness(ikTipXform)
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
        aimVec = _getVec(guide.locs[3], guide.locs[4], self.invert)
        upVecTemp = mathOps.getMatrixAxisAsVector(self.ikTip_ctrl.worldMatrix[0].get(), 'z')
        sideVec = upVecTemp.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(guide.locs[4], q=1, ws=1, t=1)

        toeXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        if self.invert:
            toeXform = mathOps.invertHandedness(toeXform)

        self.ikToe_ctrl = self.addCtrl(shape='pringle', size=ctrlSize,
                                       name=self.getName('ik_toe'), xform=toeXform, buffer=1,
                                       parent=self.ikTip_ctrl, metaParent=self.ikTip_ctrl)

        # Tarsi controls
        self.tarsiCtrls = []
        for index, tarsi in enumerate(guide.locs[7:]):
            aimVec = _getVec(self.controls_list[-1], guide.locs[index+7], self.invert)
            upVecTemp = mathOps.getMatrixAxisAsVector(self.controls_list[-1].worldMatrix[0].get(), 'z')
            sideVec = upVecTemp.cross(aimVec).normal()
            if self.invert:
                sideVec = aimVec.cross(upVecTemp).normal()
            upVec = aimVec.cross(sideVec).normal()
            startPos = pm.xform(guide.locs[index+7], q=1, ws=1, t=1)

            xform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
            if self.invert:
                xform = mathOps.invertHandedness(xform)

            ctrl = self.addCtrl(shape='pringle', size=ctrlSize,
                                name=self.getName('ik_tarsi_%s' % (str(index+1).zfill(2))), xform=xform, buffer=1,
                                parent=self.controls_list[-1], metaParent=self.controls_list[-1])
            self.tarsiCtrls.append(ctrl)

        # End srt - this is what will be measured against the root to determine ik ankle displacement
        self.ikEnd_srt = dag.addChild(self.controls_list[-1], 'group', name=self.getName('ik_end_srt'))
        transform.align(self.ikEnd_srt, self.base_srt)

        # ----------------------
        # FK controls
        # ----------------------
        refControls = self.controls_list[3:]
        refControls.append(self.ikEnd_srt)
        refControls.reverse()
        self.blendMtxList = []
        self.fkCtrls = []
        for index, ik in enumerate(refControls):
            if index == 1:
                parent = self.base_srt
                refParent = refControls[index-1]
            elif index > 1:
                parent = self.controls_list[-1]
                refParent = refControls[index-1].getParent()
            if 0 < index < len(refControls)-1:
                target = refControls[index+1]
                xform = ik.worldMatrix[0].get()
                num = str(index).zfill(2)
                ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.67,
                                    name=self.getName('fk_%s' % num), xform=xform,
                                    parent=parent, metaParent=parent)

                aimMtx = transform.createAimMatrix(ik.worldMatrix[0], target.worldMatrix[0],
                                                   name=self.getName('fk_%s_aim_mtx' % num))
                target.worldMatrix[0].connect(aimMtx.secondaryTargetMatrix)
                aimMtx.secondaryMode.set(2)
                aimMtx.secondaryInputAxis.set((0, 0, 1))
                aimMtx.secondaryTargetVector.set((0, 0, 1))
                if self.invert:
                    aimMtx.secondaryTargetVector.set((0, 0, -1))

                ctrlMtx = mathOps.multiplyMatrices([aimMtx.outputMatrix, refParent.worldInverseMatrix[0]],
                                                   name=self.getName('fk_%s_reverse_mtx' % num))
                fkMtx = mathOps.composeMatrixFromMatrix(ctrlMtx.matrixSum.get(),
                                                        name=self.getName('fk_%s_rest_mtx' % num))
                blendMtx = transform.blendMatrices(ctrlMtx.matrixSum, fkMtx.outputMatrix,
                                                   name=self.getName('fk_%s_mtx' % num))
                blendMtx.outputMatrix.connect(ctrl.offsetParentMatrix)
                self.blendMtxList.append(blendMtx)
                self.fkCtrls.append(ctrl)



        if guide.root.add_joint.get():
            if self.invert:
                negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
            for i in range(self.guide.tarsi_segs + 1):
                num = str(i+1).zfill(2)
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                if i == 0:
                    driver = self.base_srt
                else:
                    driver = self.fkCtrls[i-1]
                if self.invert and i > 0:
                    out_srt = dag.addChild(self.rig, 'group', self.getName('out_%s_srt' % num))
                    out_mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, driver.worldMatrix[0]],
                                                       name=self.getName('out_%s_mtx' % num))
                    out_mtx.matrixSum.connect(out_srt.offsetParentMatrix)

                    # Add mapping connection to control - used when mapping controller tags
                    self.mapToControl(self.controls_list[-1], out_srt)

                    driver = out_srt
                self.joints_list.append({'joint': j, 'driver': driver})
                self.mapJointToGuideLocs(j, guide.locs[len(guide.locs)-(i+1)])


        # MAP TO GUIDE LOCS
        mappingPairs = [[self.ik_base_srt, guide.locs[1]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'foot_scale', value=1.0)
        attribute.addFloatAttr(self.params, 'ikfk_blend', minValue=0, maxValue=1)
        if self.guide.attr_driven:
            attribute.addDividerAttr(self.params, 'Roll')
            for attr in  ['roll_heel', 'roll_ball', 'roll_tip']:
                attribute.addAngleAttr(self.params, attr)
            #attribute.proxyAttribute(self.ikHeel_ctrl.rz, self.params, 'roll_heel')
            #attribute.proxyAttribute(self.ikToe_ctrl.rz, self.params, 'roll_ball')
            #attribute.proxyAttribute(self.ikTip_ctrl.rz, self.params, 'roll_tip')
            for tarsi in range(self.guide.tarsi_segs-1):
                num = str(tarsi+1).zfill(2)
                attribute.addAngleAttr(self.params, 'roll_tarsi_%s' % num)
                #attribute.proxyAttribute(self.tarsiCtrls[tarsi].rz, self.params, 'roll_tarsi_%s' % num)

            attribute.addDividerAttr(self.params, 'Spin')
            for attr in  ['spin_heel', 'spin_ball', 'spin_tip']:
                attribute.addAngleAttr(self.params, attr)
            #attribute.proxyAttribute(self.ikHeel_ctrl.ry, self.params, 'spin_heel')
            #attribute.proxyAttribute(self.ikBall_ctrl.ry, self.params, 'spin_ball')
            #attribute.proxyAttribute(self.ikTip_ctrl.ry, self.params, 'spin_tip')
            for tarsi in range(self.guide.tarsi_segs-1):
                num = str(tarsi+1).zfill(2)
                attribute.addAngleAttr(self.params, 'spin_tarsi_%s' % num)
                #attribute.proxyAttribute(self.tarsiCtrls[tarsi].ry, self.params, 'spin_tarsi_%s' % num)

            attribute.addDividerAttr(self.params, 'Lean')
            for attr in ['lean_heel', 'lean_edge', 'lean_tip']:
                attribute.addAngleAttr(self.params, attr)
            #attribute.proxyAttribute(self.ikHeel_ctrl.rx, self.params, 'lean_heel')
            #attribute.proxyAttribute(self.ikBall_ctrl.rx, self.params, 'lean_edge')
            #attribute.proxyAttribute(self.ikTip_ctrl.rx, self.params, 'lean_tip')
            for tarsi in range(self.guide.tarsi_segs-1):
                num = str(tarsi+1).zfill(2)
                attribute.addAngleAttr(self.params, 'lean_tarsi_%s' % num)
                #attribute.proxyAttribute(self.tarsiCtrls[tarsi].rx, self.params, 'lean_tarsi_%s' % num)




    def addSystems(self):
        if self.guide.attr_driven:
            self.params.roll_heel.connect(self.ikHeel_ctrl.rz)
            self.params.roll_ball.connect(self.ikToe_ctrl.rz)
            self.params.roll_tip.connect(self.ikTip_ctrl.rz)
            for tarsi in range(self.guide.tarsi_segs-1):
                num = str(tarsi+1).zfill(2)
                attr = pm.general.Attribute('%s.roll_tarsi_%s' % (self.params.name(), num))
                attr.connect(self.tarsiCtrls[tarsi].rz)

            self.params.spin_heel.connect(self.ikHeel_ctrl.ry)
            self.params.spin_ball.connect(self.ikBall_ctrl.ry)
            self.params.spin_tip.connect(self.ikTip_ctrl.ry)
            for tarsi in range(self.guide.tarsi_segs - 1):
                num = str(tarsi + 1).zfill(2)
                attr = pm.general.Attribute('%s.spin_tarsi_%s' % (self.params.name(), num))
                attr.connect(self.tarsiCtrls[tarsi].ry)

            self.params.lean_heel.connect(self.ikHeel_ctrl.rx)
            self.params.lean_edge.connect(self.ikBall_ctrl.rx)
            self.params.lean_tip.connect(self.ikTip_ctrl.rx)
            for tarsi in range(self.guide.tarsi_segs - 1):
                num = str(tarsi + 1).zfill(2)
                attr = pm.general.Attribute('%s.lean_tarsi_%s' % (self.params.name(), num))
                attr.connect(self.tarsiCtrls[tarsi].rx)
                
                
        # Ik foot side roll system
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

        for mtx in self.blendMtxList:
            self.params.ikfk_blend.connect(mtx.target[0].weight)

        # Attach params shape to base srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.base_srt, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):
        self.setColours(self.guide)

        attribute.proxyAttribute(self.params.foot_scale,
                                 self.controls_list[len(self.controls_list)-self.guide.tarsi_segs])

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodes = [node for node in self.fkCtrls]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = [node for node in self.controls_list if '_ik_' in node.name()]
        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        if self.guide.attr_driven:
            self.ikHeel_ctrl.getParent().v.set(0)
            for ctrl in [self.ikHeel_ctrl, self.ikTip_ctrl, self.ikBall_ctrl, self.ikToe_ctrl] + self.tarsiCtrls:
                ctrl.is_tControl.set(0)




def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TReverseFoot(guide)

