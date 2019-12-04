from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve, anim
reload(components)
reload(mathOps)
reload(curve)
reload(anim)

import pymel.core as pm

class TSpine01(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'spine01')
        print 'Created Spine01 Component: %s' % self.comp_name

    def addObjects(self, guide):
        # FK controls - base, lower, mid, upper
        xform = pm.xform(pm.PyNode(guide.root), q=1, ws=1, m=1)
        self.base_ctrl = self.addCtrl(shape='circlePoint', size=20.0,
                                      name=self.getName('base'), xform=xform, parent=self.base_srt, buffer=1)

        self.fk_base_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                         name=self.getName('fk_base'), xform=xform, parent=self.base_ctrl, buffer=1)
        xform = pm.xform(pm.PyNode(guide.locs[1]), q=1, ws=1, m=1)
        self.fk_lower_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                          name=self.getName('fk_lower'), xform=xform, parent=self.fk_base_ctrl, buffer=1)
        xform = pm.xform(pm.PyNode(guide.locs[5]), q=1, ws=1, m=1)
        self.fk_mid_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                        name=self.getName('fk_mid'), xform=xform, parent=self.fk_lower_ctrl, buffer=1)
        xform = pm.xform(pm.PyNode(guide.locs[3]), q=1, ws=1, m=1)
        self.fk_upper_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                          name=self.getName('fk_upper'), xform=xform, parent=self.fk_mid_ctrl, buffer=1)

        self.divs = []
        for i, div in enumerate(guide.divisionLocs):
            num = str(i+1).zfill(2)
            mtx = pm.xform(div, q=1, ws=1, m=1)
            node = dag.addChild(self.rig, 'group', name=self.getName('div_%s_srt' % num))
            pm.xform(node, ws=1, m=mtx)
            self.mapToGuideLocs(node, div)
            self.divs.append(node)

        # Joints
        if guide.root.add_joint.get():
            for i, div in enumerate(self.divs):
                num = str(i+1).zfill(2)
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                self.joints_list.append({'joint': j, 'driver': div})
                self.mapJointToGuideLocs(j, self.guide.divisionLocs[i])
                if i != 0:
                    pass
                    # j.setParent(self.joints_list[0]['joint'])

        # IK controls - lower, mid, upper
        xform = pm.xform(self.base_ctrl, q=1, ws=1, m=1)
        self.ik_lower_ctrl = self.addCtrl(shape='squarePoint', size=10.0,
                                          name=self.getName('ik_lower'), xform=xform, parent=self.base_ctrl, buffer=1)
        xform = pm.xform(pm.PyNode(guide.divisionLocs[guide.num_divisions-1]), q=1, ws=1, m=1)
        self.ik_mid_ctrl = self.addCtrl(shape='squarePoint', size=8.0,
                                        name=self.getName('ik_mid'), xform=xform, parent=self.controls, buffer=1)
        xform = pm.xform(pm.PyNode(guide.locs[4]), q=1, ws=1, m=1)
        self.ik_upper_ctrl = self.addCtrl(shape='squarePoint', size=10.0,
                                          name=self.getName('ik_upper'), xform=xform, parent=self.fk_upper_ctrl, buffer=1)

        # Default length
        dm = mathOps.decomposeMatrix(self.base_ctrl.worldMatrix[0], name=self.getName('base_ctrl_mtx2Srt'))
        l1 = mathOps.getDistance(self.ik_lower_ctrl, self.ik_mid_ctrl)
        l2 = mathOps.getDistance(self.ik_upper_ctrl, self.ik_mid_ctrl)
        defaultLen = mathOps.multiply((l1 + l2), dm.outputScaleX, name=self.getName('default_len'))

        lowerDist = mathOps.distance(self.ik_lower_ctrl, self.ik_mid_ctrl.getParent(), name=self.getName('ik_lower_dist'))
        upperDist = mathOps.distance(self.ik_upper_ctrl, self.ik_mid_ctrl.getParent(), name=self.getName('ik_upper_dist'))
        stretchLen = mathOps.addScalar([lowerDist.distance, upperDist.distance], name=self.getName('ik_stretch_len'))

        stretchFactor = mathOps.divide((1, 1, 1), (1, 1, 1), name=self.getName('ik_stretch_factor'))
        stretchLen.output1D.connect(stretchFactor.input1X)
        defaultLen.output.connect(stretchFactor.input2X)

        # Drive ik mid buffer
        lowerTangentVec = mathOps.multiplyTerms(guide.locs[1].t.get(), (1.5, 1.5, 1.5))
        lowerTangentScaledVec = mathOps.multiplyVector(guide.locs[1].t.get(), stretchFactor.output,
                                                 name=self.getName('ik_lower_scaled_tangent_point'))
        stretchFactor.outputX.connect(lowerTangentScaledVec.input2Y)
        stretchFactor.outputX.connect(lowerTangentScaledVec.input2Z)
        lowerTangentPos = mathOps.createTransformedPoint(lowerTangentScaledVec.output, self.ik_lower_ctrl.worldMatrix[0],
                                                         name=self.getName('ik_lower_tangent_point'))
        lowerMidPos = mathOps.createTransformedPoint(lowerTangentVec, self.ik_lower_ctrl.worldMatrix[0],
                                                         name=self.getName('ik_lower_mid_point'))
        upperTangentVec = mathOps.multiplyTerms(guide.locs[3].t.get(), (1.5, 1.5, 1.5))
        upperTangentScaledVec = mathOps.multiplyVector(guide.locs[3].t.get(), stretchFactor.output,
                                                 name=self.getName('ik_upper_scaled_tangent_point'))
        stretchFactor.outputX.connect(upperTangentScaledVec.input2Y)
        stretchFactor.outputX.connect(upperTangentScaledVec.input2Z)
        upperTangentPos = mathOps.createTransformedPoint(upperTangentScaledVec.output, self.ik_upper_ctrl.worldMatrix[0],
                                                         name=self.getName('ik_upper_tangent_point'))
        upperMidPos = mathOps.createTransformedPoint(upperTangentVec, self.ik_upper_ctrl.worldMatrix[0],
                                                         name=self.getName('ik_upper_mid_point'))

        lowerAimPos = mathOps.createTransformedPoint((0, 0, 0), self.ik_lower_ctrl.worldMatrix[0],
                                                     name=self.getName('ik_lower_aim_point'))
        upperAimPos = mathOps.createTransformedPoint((0, 0, 0), self.ik_upper_ctrl.worldMatrix[0],
                                                     name=self.getName('ik_upper_aim_point'))

        pb = mathOps.pairBlend(translateA=lowerMidPos.output, translateB=upperMidPos.output,
                               name=self.getName('mid_blend_point'))

        dm = transform.decomposeMatrix(self.base_ctrl.worldMatrix[0], name=self.getName('fk_base_ctrl_mtx2Srt'))
        ikAimVec = mathOps.subtractVector([upperAimPos.output, lowerAimPos.output],
                                          name=self.getName('ik_mid_aimVec'))
        ikAimVecOffset = mathOps.addVector([ikAimVec.output3D, dm.outputTranslate],
                                           name=self.getName('ik_mid_aimVec_offset'))
        ikAimVecLocal = mathOps.createTransformedPoint(ikAimVecOffset.output3D, self.base_ctrl.worldInverseMatrix[0],
                                                       name=self.getName('ik_aimVec_local'))
        ikAimVecNormal = mathOps.normalize(ikAimVecLocal.output, name=self.getName('ik_aimVec_normal'))
        ang = mathOps.angleBetween((0, 1, 0), ikAimVecLocal.output, name=self.getName('ik_mid_angle'))
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
        pb.outTranslate.connect(self.ik_mid_ctrl.getParent().t)

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



        #P1
        p1 = mathOps.decomposeMatrix(self.ik_lower_ctrl.worldMatrix[0], name=self.getName('ik_lower_ctrl_mtx2Srt'))

        #P3
        p3 = mathOps.decomposeMatrix(self.ik_mid_ctrl.worldMatrix[0], name=self.getName('ik_mid_ctrl_mtx2Srt'))

        #P5
        p5 = mathOps.decomposeMatrix(self.ik_upper_ctrl.worldMatrix[0], name=self.getName('ik_upper_ctrl_mtx2Srt'))

        # IK Curve and rail curves

        points = [(p, 0, 0) for p in range(5)]

        self.crv = curve.curveThroughPoints(name=self.getName('ik_crv'), positions=points, degree=3)
        self.crv.setParent(self.rig)

        p1.outputTranslate.connect(self.crv.controlPoints[0])
        lowerTangentPos.output.connect(self.crv.controlPoints[1])
        p3.outputTranslate.connect(self.crv.controlPoints[2])
        upperTangentPos.output.connect(self.crv.controlPoints[3])
        p5.outputTranslate.connect(self.crv.controlPoints[4])

        offsetLower = mathOps.createMatrixAxisVector(self.ik_lower_ctrl.worldMatrix[0], (0, 0, 1),
                                                     name=self.getName('ik_railLower_offset_vec'))

        offsetMid = mathOps.createMatrixAxisVector(self.ik_mid_ctrl.worldMatrix[0], (0, 0, 1),
                                                   name=self.getName('ik_railMid_offset_vec'))

        offsetLowerTangent = mathOps.pairBlend(translateA=offsetLower.output, translateB=offsetMid.output,
                                               name=self.getName('ik_railLowerTangent_offset_vec'))

        offsetUpper = mathOps.createMatrixAxisVector(self.ik_upper_ctrl.worldMatrix[0], (0, 0, 1),
                                                     name=self.getName('ik_railUpper_offset_vec'))

        offsetUpperTangent = mathOps.pairBlend(translateA=offsetUpper.output, translateB=offsetMid.output,
                                               name=self.getName('ik_railUpperTangent_offset_vec'))

        #u1
        u1 = mathOps.addVector([offsetLower.output, p1.outputTranslate], name=self.getName('ik_lower_rail_point'))
        #u2
        u2 = mathOps.addVector([offsetLowerTangent.outTranslate, lowerTangentPos.output],
                               name=self.getName('ik_lowerTangent_rail_point'))

        #u3
        u3 = mathOps.addVector([offsetMid.output, p3.outputTranslate], name=self.getName('ik_mid_rail_point'))

        #u4
        u4 = mathOps.addVector([offsetUpperTangent.outTranslate, upperTangentPos.output],
                               name=self.getName('ik_upperTangent_rail_point'))

        u5 = mathOps.addVector([offsetUpper.output, p5.outputTranslate], name=self.getName('ik_upper_rail_point'))

        self.rail = curve.curveThroughPoints(name=self.getName('ik_rail'), positions=points, degree=3)
        self.rail.setParent(self.rig)

        u1.output3D.connect(self.rail.controlPoints[0])
        u2.output3D.connect(self.rail.controlPoints[1])
        u3.output3D.connect(self.rail.controlPoints[2])
        u4.output3D.connect(self.rail.controlPoints[3])
        u5.output3D.connect(self.rail.controlPoints[4])

        components.TBaseComponent.addObjects(self, guide)


    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'bulge_amount')
        attribute.addFloatAttr(self.params, 'bulge_position')
        attribute.addFloatAttr(self.params, 'bulge_falloff')
        attribute.addFloatAttr(self.params, 'auto_bulge')



    def addSystems(self):
        stretchNode = pm.PyNode(self.getName('ik_stretch_factor'))
        stretchRev = mathOps.reverse(stretchNode.outputX, name=self.getName('stretch_rev'))
        stretchAuto = mathOps.multiply(stretchRev.outputX, self.params.auto_bulge,
                                       name=self.getName('bulge_auto_mult'))
        dm = mathOps.decomposeMatrix(self.base_ctrl.worldMatrix[0], recycle=1)
        # Motion path nodes

        for i, div in enumerate(self.divs):
            num = str(i+1).zfill(2)
            param = pm.listConnections(self.guide.divisionLocs[i], type='motionPath')[0].uValue.get()
            mp = curve.createMotionPathNode(self.crv, uValue=param, frontAxis='y', upAxis='z',
                                            name=self.getName('%s_mp' % num))
            railMp = curve.createMotionPathNode(self.rail, uValue=param, follow=0, name=self.getName('%s_rail_mp' % num))
            upVec = mathOps.subtractVector([railMp.allCoordinates, mp.allCoordinates],
                                           name=self.getName('%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            mp.allCoordinates.connect(div.t)
            mp.rotate.connect(div.r)

            # Squash n Stretch
            bulgeDist = pm.createNode('distanceBetween', name=self.getName('%s_bulge_dist' % num))
            self.params.bulge_position.connect(bulgeDist.point1X)
            bulgeDist.point2X.set(param)
            bulgeRemap = mathOps.remap(bulgeDist.distance, 0, self.params.bulge_falloff, 1, 0,
                                       name=self.getName('%s_bulge_remap' % num))
            bulgeEase = anim.easeCurve(input=bulgeRemap.outValueX,
                                             name=self.getName('%s_bulgeAmount_easeCrv' % num))
            bulgeAmount = mathOps.multiply(bulgeEase.output, self.params.bulge_amount,
                                           name=self.getName('%s_bulge_amount' % num))
            bulgeAuto = mathOps.multiply(bulgeEase.output, stretchAuto.output,
                                         name=self.getName('%s_bulge_auto_amount' % num))
            bulgeSum = mathOps.addScalar([dm.outputScaleX, bulgeAmount.output, bulgeAuto.output],
                                         name=self.getName('%s_bulge_sum' % num))

            bulgeSum.output1D.connect(div.sx)
            bulgeSum.output1D.connect(div.sz)
            dm.outputScaleY.connect(div.sy)






def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSpine01(guide)
