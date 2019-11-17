from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve
reload(components)
reload(mathOps)
reload(curve)

import pymel.core as pm

class TSpine01(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'spine01')
        print 'Created Spine01 Component: %s' % self.comp_name

    def addObjects(self, guide):
        # FK controls - base, lower upper
        xform = pm.xform(pm.PyNode(guide.root), q=1, ws=1, m=1)
        self.base_ctrl = self.addCtrl(shape='circlePoint', size=20.0,
                                      name=self.getName('fk_base'), xform=xform, parent=self.base_srt, buffer=1)
        xform = pm.xform(pm.PyNode(guide.divisionLocs[guide.num_divisions-1]), q=1, ws=1, m=1)
        self.lower_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                       name=self.getName('fk_base'), xform=xform, parent=self.base_ctrl, buffer=1)
        xform = pm.xform(pm.PyNode(guide.divisionLocs[len(guide.divisionLocs)-1]), q=1, ws=1, m=1)
        self.upper_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                       name=self.getName('fk_base'), xform=xform, parent=self.lower_ctrl, buffer=1)

        self.divs = []
        for i, div in enumerate(guide.divisionLocs):
            num = str(i+1).zfill(2)
            mtx = pm.xform(div, q=1, ws=1, m=1)
            parent = self.rig
            if i > 0:
                parent = self.divs[i-1]
            node = dag.addChild(parent, 'group', name=self.getName('div_%s_driven_srt' % num))
            pm.xform(node, ws=1, m=mtx)
            buffer = dag.addParent(node, 'group', name=self.getName('div_%s_buffer_srt' % num))
            self.mapToGuideLocs(node, div)
            self.divs.append(node)

        # IK controls - lower, mid, upper
        xform = pm.xform(self.base_ctrl, q=1, ws=1, m=1)
        self.ik_lower_ctrl = self.addCtrl(shape='squarePoint', size=10.0,
                                          name=self.getName('ik_lower'), xform=xform, parent=self.controls, buffer=1)
        xform = pm.xform(self.lower_ctrl, q=1, ws=1, m=1)
        self.ik_mid_ctrl = self.addCtrl(shape='squarePoint', size=8.0,
                                        name=self.getName('ik_mid'), xform=xform, parent=self.controls, buffer=1)
        xform = pm.xform(self.upper_ctrl, q=1, ws=1, m=1)
        self.ik_upper_ctrl = self.addCtrl(shape='squarePoint', size=10.0,
                                          name=self.getName('ik_base'), xform=xform, parent=self.controls, buffer=1)

        # Drive ik mid buffer
        dm = transform.decomposeMatrix(self.lower_ctrl.worldMatrix[0], name=self.getName('lower_ctrl_mtx2Srt'))
        pb = pm.createNode('pairBlend', name=self.getName('ik_mid_translate_blend'))
        pb.weight.set(self.guide.root.mid_point.get())
        ikDist = mathOps.distance(self.ik_lower_ctrl, self.ik_upper_ctrl, name=self.getName('ik_dist'))

        ikLowerAimLen = mathOps.multiply(ikDist.distance, 0.67*self.guide.root.mid_point.get(),
                                         name=self.getName('ik_lower_aim_len'))
        ikLowerAim = mathOps.createTransformedPoint((0, 0, 0), self.ik_lower_ctrl.worldMatrix[0],
                                                    name=self.getName('ik_lower_aim_point'))
        ikLowerAimLen.output.connect(ikLowerAim.input1Y)
        ikLowerAimRef = mathOps.createTransformedPoint((0, 0, 0), self.ik_lower_ctrl.getParent().worldMatrix[0],
                                                       name=self.getName('ik_lower_aim_ref_point'))
        ikLowerAimLen.output.connect(ikLowerAimRef.input1Y)
        ikLowerDisplace = mathOps.subtractVector([ikLowerAim.output, ikLowerAimRef.output],
                                                 name=self.getName('ik_lower_displace'))
        ikUpperAimLen = mathOps.multiply(ikDist.distance, -0.67*self.guide.root.mid_point.get(),
                                         name=self.getName('ik_upper_aim_len'))
        ikUpperAim = mathOps.createTransformedPoint((0, 0, 0), self.ik_upper_ctrl.worldMatrix[0],
                                                    name=self.getName('ik_upper_aim_point'))
        ikUpperAimLen.output.connect(ikUpperAim.input1Y)
        ikUpperAimRef = mathOps.createTransformedPoint((0, 0, 0), self.ik_upper_ctrl.getParent().worldMatrix[0],
                                                       name=self.getName('ik_Upper_aim_ref_point'))
        ikUpperAimLen.output.connect(ikUpperAimRef.input1Y)
        ikUpperDisplace = mathOps.subtractVector([ikUpperAim.output, ikUpperAimRef.output],
                                                 name=self.getName('ik_upper_displace'))

        ikLowerDisplace.output3D.connect(pb.inTranslate1)
        ikUpperDisplace.output3D.connect(pb.inTranslate2)
        midPosSum = mathOps.addVector([dm.outputTranslate, pb.outTranslate], name=self.getName('ik_mid_translate_sum'))
        midPosSum.output3D.connect(self.ik_mid_ctrl.getParent().t)
        dm = transform.decomposeMatrix(self.base_ctrl.worldMatrix[0], name=self.getName('fk_base_ctrl_mtx2Srt'))
        ikAimVec = mathOps.subtractVector([ikUpperAim.output, ikLowerAim.output], name=self.getName('ik_mid_aimVec'))
        ikAimVecOffset = mathOps.addVector([ikAimVec.output3D, dm.outputTranslate],
                                           name=self.getName('ik_mid_aimVec_offset'))
        ikAimVecLocal = mathOps.createTransformedPoint(ikAimVecOffset.output3D, self.base_ctrl.worldInverseMatrix[0],
                                                       name=self.getName('ik_aimVec_local'))
        ikAimVecNormal = mathOps.normalize(ikAimVecLocal.output, name=self.getName('ik_aimVec_normal'))
        ang = mathOps.angleBetween((0, 1, 0),ikAimVecLocal.output, name=self.getName('ik_mid_angle'))
        ikUpMtx = mathOps.createComposeMatrix(inputRotate=ang.euler, name=self.getName('ik_up_mtx'))
        ikUpVec = mathOps.createMatrixAxisVector(ikUpMtx.outputMatrix, (1, 0, 0), self.getName('ik_upVec'))
        ikSideVec = mathOps.createCrossProduct(ikUpVec.output, ikAimVecNormal.output, name=self.getName('ik_sideVec'))

        ikAimMtx = mathOps.vectors2Mtx44((ikUpVec.outputX, ikUpVec.outputY, ikUpVec.outputZ),
                                         (ikAimVecNormal.outputX, ikAimVecNormal.outputY, ikAimVecNormal.outputZ),
                                         (ikSideVec.outputX, ikSideVec.outputY, ikSideVec.outputZ),
                                         name=self.getName('ik_mid_mtx'))

        ikMidMtx = mathOps.multiplyMatrices([ikAimMtx.output, self.base_ctrl.worldMatrix[0]],
                                            name=self.getName('ik_mid_mtx'))

        dm = transform.decomposeMatrix(ikMidMtx.matrixSum, name=self.getName('ik_mid_mtx2Srt'))
        transform.connectSrt(dm, self.ik_mid_ctrl.getParent(), t=0)

        # Drive mid IK twist
        ikUpper_midSpace_mtx = mathOps.multiplyMatrices([self.ik_upper_ctrl.worldMatrix[0],
                                                         self.ik_mid_ctrl.getParent().worldInverseMatrix[0]],
                                                        name=self.getName('ikUpper_midSpace_mtx'))
        dm = transform.decomposeMatrix(ikUpper_midSpace_mtx.matrixSum, name=self.getName('ikUpper_midSpace_mtx2Srt'))
        upperTwist = mathOps.isolateRotationOnAxis(dm.outputRotate, axis='y', name=self.getName('ikUpper_twist'))
        upperTwistMult = mathOps.multiplyAngleByScalar(upperTwist[1].outputRotateY, self.guide.root.mid_point.get(),
                                                       name=self.getName('ikUpper_twist_mult'))

        ikLower_midSpace_mtx = mathOps.multiplyMatrices([self.ik_lower_ctrl.worldMatrix[0],
                                                         self.ik_mid_ctrl.getParent().worldInverseMatrix[0]],
                                                        name=self.getName('ikLower_midSpace_mtx'))
        dm = transform.decomposeMatrix(ikLower_midSpace_mtx.matrixSum, name=self.getName('ikLower_midSpace_mtx2Srt'))
        lowerTwist = mathOps.isolateRotationOnAxis(dm.outputRotate, axis='y', name=self.getName('ikLower_twist'))
        lowerTwistMult = mathOps.multiplyAngleByScalar(lowerTwist[1].outputRotateY, 1 - self.guide.root.mid_point.get(),
                                                       name=self.getName('ikLower_twist_mult'))
        twistSum = mathOps.addAngles(upperTwistMult.output, lowerTwistMult.output, name=self.getName('ik_twist_sum'))
        twistSrt = dag.addParent(self.ik_mid_ctrl, 'group', name=self.getName('ik_mid_twist_srt'))
        twistSum.output.connect(twistSrt.ry)

        # Default length
        l1 = mathOps.getDistance(self.ik_lower_ctrl, self.ik_mid_ctrl)
        l2 = mathOps.getDistance(self.ik_upper_ctrl, self.ik_mid_ctrl)
        defaultLen = l1 + l2

        lowerDist = mathOps.distance(self.ik_lower_ctrl, self.ik_mid_ctrl, name=self.getName('ik_lower_dist'))
        upperDist = mathOps.distance(self.ik_upper_ctrl, self.ik_mid_ctrl, name=self.getName('ik_upper_dist'))
        stretchLen = mathOps.addScalar([lowerDist.distance, upperDist.distance], name=self.getName('ik_stretch_len'))

        stretchFactor = mathOps.divide(stretchLen.output1D, defaultLen, name=self.getName('ik_stretch_factor'))


        #P1
        p1 = mathOps.decomposeMatrix(self.ik_lower_ctrl.worldMatrix[0], name=self.getName('ik_lower_ctrl_mtx2Srt'))

        print 'locs: %s' % self.guide.locs

        #P2
        lowerTangentPos = self.guide.locs[3].t.get()
        lowerTangentLen = mathOps.multiply(lowerTangentPos[1], stretchFactor.outputX,
                                           name=self.getName('ik_lowerTangent_len'))
        p2Base = mathOps.createTransformedPoint(lowerTangentPos, self.ik_lower_ctrl.worldMatrix[0],
                                            name=self.getName('ik_lower_tangentBase_point'))
        lowerTangentLen.output.connect(p2Base.input1Y)
        p2Ref = mathOps.createTransformedPoint(p2Base.output, self.ik_mid_ctrl.getParent().worldInverseMatrix[0],
                                            name=self.getName('ik_lower_tangentRef_point'))
        p2Disp = mathOps.createTransformedPoint(p2Ref.output, self.ik_mid_ctrl.matrix,
                                            name=self.getName('ik_lower_tangentDisplace_point'))
        p2 = mathOps.createTransformedPoint(p2Disp.output, self.ik_mid_ctrl.getParent().worldMatrix[0],
                                            name=self.getName('ik_lower_tangent_point'))

        #P3
        upperTangentPos = tuple(attr.get() * -1 for attr in [self.guide.locs[1].tx,
                                                             self.guide.locs[1].ty,
                                                             self.guide.locs[1].tz])
        upperTangentLen = mathOps.multiply(upperTangentPos[1], stretchFactor.outputX,
                                           name=self.getName('ik_upperTangent_len'))
        p3Base = mathOps.createTransformedPoint(upperTangentPos, self.ik_upper_ctrl.worldMatrix[0],
                                            name=self.getName('ik_upper_tangentBase_point'))
        upperTangentLen.output.connect(p3Base.input1Y)
        p3Ref = mathOps.createTransformedPoint(p3Base.output, self.ik_mid_ctrl.getParent().worldInverseMatrix[0],
                                            name=self.getName('ik_upper_tangentRef_point'))
        p3Disp = mathOps.createTransformedPoint(p3Ref.output, self.ik_mid_ctrl.matrix,
                                            name=self.getName('ik_upper_tangentDisplace_point'))
        p3 = mathOps.createTransformedPoint(p3Disp.output, self.ik_mid_ctrl.getParent().worldMatrix[0],
                                            name=self.getName('ik_upper_tangent_point'))


        #P4
        p4 = mathOps.decomposeMatrix(self.ik_upper_ctrl.worldMatrix[0], name=self.getName('ik_upper_ctrl_mtx2Srt'))

        # IK Curve and rail curves

        points = [(p, 0, 0) for p in range(4)]

        self.crv = curve.curveThroughPoints(name=self.getName('ik_crv'), positions=points)
        self.crv.setParent(self.rig)

        p1.outputTranslate.connect(self.crv.controlPoints[0])
        p2.output.connect(self.crv.controlPoints[1])
        p3.output.connect(self.crv.controlPoints[2])
        p4.outputTranslate.connect(self.crv.controlPoints[3])

        offsetLower = mathOps.createMatrixAxisVector(self.ik_lower_ctrl.worldMatrix[0], (0, 0, 1),
                                                name=self.getName('ik_railLower_offset_vec'))

        offsetMid = mathOps.createMatrixAxisVector(self.ik_mid_ctrl.worldMatrix[0], (0, 0, 1),
                                                name=self.getName('ik_railMid_offset_vec'))

        offsetUpper = mathOps.createMatrixAxisVector(self.ik_upper_ctrl.worldMatrix[0], (0, 0, 1),
                                                name=self.getName('ik_railUpper_offset_vec'))
        #u1
        u1 = mathOps.addVector([offsetLower.output, p1.outputTranslate], name=self.getName('ik_lower_rail_point'))
        #u2
        u2 = mathOps.addVector([offsetMid.output, p2.output], name=self.getName('ik_lowerTangent_rail_point'))

        #u3
        u3 = mathOps.addVector([offsetMid.output, p3.output], name=self.getName('ik_upperTangent_rail_point'))

        #u4
        u4 = mathOps.addVector([offsetUpper.output, p4.outputTranslate], name=self.getName('ik_upper_rail_point'))

        self.rail = curve.curveThroughPoints(name=self.getName('ik_rail'), positions=points)
        self.rail.setParent(self.rig)

        u1.output3D.connect(self.rail.controlPoints[0])
        u2.output3D.connect(self.rail.controlPoints[1])
        u3.output3D.connect(self.rail.controlPoints[2])
        u4.output3D.connect(self.rail.controlPoints[3])

        components.TBaseComponent.addObjects(self, guide)


    def addSystems(self):
        pass



def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSpine01(guide)
