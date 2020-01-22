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
                                        parent=self.ik_base_srt)

        self.ikInner_srt = dag.addChild(self.ikHeel_ctrl, 'group', self.getName('ik_inner_srt'))
        transform.align(self.ikInner_srt, guide.locs[6])
        self.ikInner_srt.s.set(1, 1, 1)
        self.ikOuter_srt = dag.addChild(self.ikInner_srt, 'group', self.getName('ik_outer_srt'))
        transform.align(self.ikOuter_srt, guide.locs[5])
        self.ikOuter_srt.s.set(1, 1, 1)
        self.ikBall_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                        name=self.getName('ik_ball'), xform=ikBallXform,
                                        parent=self.ikOuter_srt, buffer=1)
        self.ikBall_ctrl.getParent().s.set(1, 1, 1)
        self.ikTip_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                       name=self.getName('ik_tip'), xform=ikTipXform,
                                       parent=self.ikBall_ctrl)
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
                                       name=self.getName('ik_toe'), xform=toeXform,
                                       parent=self.ikTip_ctrl)

        # Tarsi controls
        for index, tarsi in enumerate(guide.locs[7:]):
            print guide.locs[index+6]
            print guide.locs[index+7]
            aimVec = _getVec(self.controls_list[-1], guide.locs[index+7], self.invert)
            upVecTemp = mathOps.getMatrixAxisAsVector(self.controls_list[-1].worldMatrix[0].get(), 'z')
            sideVec = upVecTemp.cross(aimVec).normal()
            upVec = aimVec.cross(sideVec).normal()
            startPos = pm.xform(guide.locs[index+7], q=1, ws=1, t=1)

            xform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
            if self.invert:
                xform = mathOps.invertHandedness(xform)

            ctrl = self.addCtrl(shape='pringle', size=ctrlSize,
                                name=self.getName('ik_tarsi_%s' % (str(index+1).zfill(2))), xform=xform,
                                parent=self.controls_list[-1])

        # End srt - this is what will be measured against the root to determine ik ankle displacement
        self.ikEnd_srt = dag.addChild(self.controls_list[-1], 'group', name=self.getName('ik_end_srt'))
        transform.align(self.ikEnd_srt, self.base_srt)

        # ----------------------
        # FK controls
        # ----------------------
        refControls = self.controls_list[3:]
        refControls.reverse()
        for index, ik in enumerate(refControls):
            parent = self.base_srt
            target = self.ikEnd_srt
            xform = ik.worldMatrix[0].get()
            if index > 0:
                parent = self.controls_list[-1]
                target = refControls[index-1]
            if self.invert:
                pass
                # xform = mathOps.invertHandedness(xform)
            num = str(index+1).zfill(2)
            ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.67,
                                name=self.getName('fk_%s' % num), xform=xform,
                                parent=parent)
            if index > 0:
                mtx = mathOps.multiplyMatrices([ik.worldMatrix[0], target.worldInverseMatrix[0]],
                                               name=self.getName('fk_%s_mtx' % num))
                mtx.matrixSum.connect(ctrl.offsetParentMatrix)

        if guide.root.add_joint.get():
            if self.invert:
                negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
            for i in range(self.guide.tarsi_segs + 1):
                num = str(i+1).zfill(2)
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                if i == 0:
                    driver = self.base_srt
                else:
                    ctrl = self.controls_list[len(self.controls_list) - 1 - self.guide.tarsi_segs + i]
                    driver = ctrl
                if self.invert and i > 0:
                    out_srt = dag.addChild(self.rig, 'group', self.getName('out_%s_srt' % num))
                    out_mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, driver.worldMatrix[0]],
                                                       name=self.getName('out_%s_mtx' % num))
                    out_mtx.matrixSum.connect(out_srt.offsetParentMatrix)
                    driver = out_srt
                self.joints_list.append({'joint': j, 'driver': driver})
                self.mapJointToGuideLocs(j, guide.locs[len(guide.locs)-(i+1)])


        # MAP TO GUIDE LOCS
        mappingPairs = [[self.ik_base_srt, guide.locs[1]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'foot_scale', value=1.0)


    def addSystems(self):
        # Ik foot side roll system
        buffer = self.ikBall_ctrl.getParent()
        sideRollNeg = mathOps.multiplyAngleByScalar(self.ikBall_ctrl.rx, -1, name=self.getName('ik_sideRoll_invert'))
        sideRollNeg.output.connect(buffer.rx)
        inner, outer = self.ikInner_srt, self.ikOuter_srt
        if self.invert:
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

    def finish(self):
        self.setColours(self.guide)

        attribute.proxyAttribute(self.params.foot_scale,
                                 self.controls_list[len(self.controls_list)-self.guide.tarsi_segs])

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = [node for node in self.controls_list if '_ik_' in node.name()]
        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)




def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TReverseFoot(guide)

