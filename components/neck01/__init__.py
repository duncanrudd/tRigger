from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve, anim
reload(components)
reload(mathOps)
reload(curve)
reload(anim)

import pymel.core as pm

class TNeck01(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'neck01')
        print 'Created Neck01 Component: %s' % self.comp_name

    def addObjects(self, guide):
        ctrlSize = mathOps.getDistance(guide.locs[-1], guide.root)*.3

        def _getVec(start, end, invert=0):
            start, end = mathOps.getStartAndEnd(start, end)
            if invert:
                return (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
            else:
                return (pm.datatypes.Vector(start) - pm.datatypes.Vector(end)).normal()

        # FK controls
        self.fk_ctrls = []
        for index in range(guide.num_ctrls-1):
            num = str(index+1).zfill(2)
            loc = guide.ctrlLocs[index]
            if index > 0:
                upVecTemp = mathOps.getMatrixAxisAsVector(self.controls_list[-1].worldMatrix[0], 'z')
                aimVec = _getVec(guide.ctrlLocs[index+1], loc)
                sideVec = aimVec.cross(upVecTemp).normal()
                upVec = sideVec.cross(aimVec).normal()
                startPos = pm.xform(loc, q=1, ws=1, t=1)
                xform = pm.datatypes.Matrix(sideVec, aimVec, upVec, startPos)
                parent = self.controls_list[-1]
            else:
                xform = self.base_srt.worldMatrix[0].get()
                parent = self.base_srt
            ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                name=self.getName('fk_%s' % num), xform=xform, parent=parent)
            self.fk_ctrls.append(ctrl)

        # FK tip
        self.fk_tip = dag.addChild(self.rig, 'group', self.getName('fk_tip'))
        self.fk_ctrls[-1].worldMatrix[0].connect(self.fk_tip.offsetParentMatrix)
        self.fk_tip.ty.set(mathOps.getDistance(self.fk_ctrls[-1], guide.locs[3]))

        # Aim ctrl
        xform = transform.getMatrixFromPos(guide.locs[4].worldMatrix.get().translate.get())
        self.aim_ctrl = self.addCtrl(shape='cross', size=ctrlSize,
                                     name=self.getName('aim'), xform=xform, parent=self.controls)

        # IK control
        self.ik_end_ctrl = self.addCtrl(shape='box', size=ctrlSize,
                                        name=self.getName('ik_end'), xform=guide.locs[3].worldMatrix[0].get(),
                                        parent=self.controls, buffer=1)

        # mid_ctrl
        self.ik_mid_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize*1.25,
                                        name=self.getName('ik_mid'), xform=self.root.worldMatrix[0].get(),
                                        parent=self.controls)
        # mid twist srt
        self.mid_twist_srt = dag.addChild(self.rig, 'group', name=self.getName('mid_twist_srt'))
        self.ik_mid_ctrl.worldMatrix[0].connect(self.mid_twist_srt.offsetParentMatrix)

        # mid position mtx
        d = mathOps.decomposeMatrix(self.ik_end_ctrl.worldMatrix[0], name=self.getName('ik_end_mtx2Srt'))
        baseMtx = self.fk_ctrls[len(self.fk_ctrls)/2].worldMatrix[0]
        if len(self.fk_ctrls) % 2:
            blend = transform.blendMatrices(baseMtx, self.fk_ctrls[(len(self.fk_ctrls)/2)+1].worldMatrix[0],
                                            name=self.getName('ik_mid_pos_blend_mtx'))
            baseMtx = blend.outputMatrix
        tipMtx2Srt = mathOps.decomposeMatrix(self.fk_tip.worldMatrix[0])
        tipOffset = mathOps.subtractVector([d.outputTranslate, tipMtx2Srt.outputTranslate],
                                           name=self.getName('tip_offset_vec'))
        tipOffsetMult = mathOps.multiplyVector(tipOffset.output3D, (.5, .5, .5),
                                               name=self.getName('ik_mid_tip_offset_mult'))
        baseMtx2Srt = mathOps.decomposeMatrix(baseMtx, name=self.getName('ik_mid_base_pos_mtx2Srt'))
        ik_mid_pos_sum = mathOps.addVector([baseMtx2Srt.outputTranslate, tipOffsetMult.output],
                                           name=self.getName('ik_mid_pos_sum'))
        ikMidPosMtx = mathOps.createComposeMatrix(inputTranslate=ik_mid_pos_sum.output3D,
                                                  name=self.getName('ik_mid_pos_mtx'))

        # mid rotate mtx
        endLocalPos = mathOps.createTransformedPoint(d.outputTranslate, self.fk_ctrls[0].worldInverseMatrix[0],
                                                     name=self.getName('ik_end_2_base_space_pos'))
        ikAimAngle = mathOps.angleBetween((0, 1, 0), endLocalPos.output, name=self.getName('ik_aim_angle'))
        ikAngleMtx = mathOps.createComposeMatrix(inputRotate=ikAimAngle.euler, name=self.getName('ik_angle_mtx'))
        ikAimMtx = mathOps.multiplyMatrices([ikAngleMtx.outputMatrix, self.fk_ctrls[0].worldMatrix[0]],
                                            name=self.getName('ik_aim_mtx'))

        blend = transform.blendMatrices(ikMidPosMtx.outputMatrix, ikAimMtx.matrixSum, name=self.getName('ik_mid_mtx'))
        blend.target[0].weight.set(1)
        blend.target[0].useTranslate.set(0)

        # Mid offset
        midMtx2Srt = mathOps.decomposeMatrix(self.ik_mid_ctrl.worldMatrix[0], name=self.getName('ik_mid_mtx2Srt'))
        midBaseMtx2Srt = mathOps.decomposeMatrix(blend.outputMatrix, name=self.getName('ik_mid_base_mtx2Srt'))
        midOffset = mathOps.subtractVector([midMtx2Srt.outputTranslate, midBaseMtx2Srt.outputTranslate],
                                           name=self.getName('ik_mid_offset_vec'))

        # curve
        rootMtx2Srt = mathOps.decomposeMatrix(self.fk_ctrls[0].worldMatrix[0], name=self.getName('fk_01_mtx2Srt'))
        points = [rootMtx2Srt.outputTranslate]
        for index, ctrl in enumerate(self.fk_ctrls[1:]):
            num = str(index+2).zfill(2)
            mult = (1.0 / len(self.fk_ctrls))*(index+1)
            dm = mathOps.decomposeMatrix(ctrl.worldMatrix[0], name=self.getName('fk_%s_mtx2Srt' % num))
            tipOffsetMult = mathOps.multiplyVector(tipOffset.output3D, (mult, mult, mult),
                                                   name=self.getName('ik_%s_tip_offset_mult' % num))
            mult = 1.0 - mathOps.getDistance((0, 0, 0), ((mult-0.5), 0, 0))
            midOffsetMult = mathOps.multiplyVector(midOffset.output3D, (mult, mult, mult),
                                                   name=self.getName('ik_%s_mid_offset_mult' % num))
            posSum = mathOps.addVector([dm.outputTranslate, tipOffsetMult.output, midOffsetMult.output],
                                       name=self.getName('ik_%s_pos_sum' % num))
            points.append(posSum.output3D)
        points.append(d.outputTranslate)

        initPoints = [(i, 0, 0) for i in range(guide.num_ctrls)]
        self.crv = curve.curveThroughPoints(name=self.getName('ik_crv'), positions=initPoints, degree=2)
        self.crv.setParent(self.rig)
        for index, point in enumerate(points):
            point.connect(self.crv.controlPoints[index])

        self.startTangent = pm.createNode('pointOnCurveInfo', name=self.getName('start_crv_tangent'))
        self.crv.worldSpace[0].connect(self.startTangent.inputCurve)
        self.startTangent.parameter.set(.01)
        self.endTangent = pm.createNode('pointOnCurveInfo', name=self.getName('end_crv_tangent'))
        self.crv.worldSpace[0].connect(self.endTangent.inputCurve)
        self.endTangent.parameter.set(.99)

        baseSideVec = mathOps.createMatrixAxisVector(self.fk_ctrls[0].worldMatrix[0], (-1, 0, 0),
                                                     name=self.getName('start_rail_side_vec'))
        baseOffsetVec = mathOps.createCrossProduct(self.startTangent.result.normalizedTangent, baseSideVec.output,
                                                   name=self.getName('start_rail_offset_vec'))
        midOffsetVec = mathOps.createMatrixAxisVector(self.mid_twist_srt.worldMatrix[0], (0, 0, 1),
                                                      name=self.getName('mid_rail_offset_vec'))
        endSideVec = mathOps.createMatrixAxisVector(self.ik_end_ctrl.worldMatrix[0], (-1, 0, 0),
                                                    name=self.getName('end_rail_side_vec'))
        endOffsetVec = mathOps.createCrossProduct(self.endTangent.result.normalizedTangent, endSideVec.output,
                                                  name=self.getName('start_rail_offset_vec'))

        # Mid twist
        endCrvNormal = mathOps.createCrossProduct(self.endTangent.result.normalizedTangent, endOffsetVec.output,
                                                  name=self.getName('end_crv_normal_vec'))
        endCrvMtx = mathOps.vectors2Mtx44(endCrvNormal.output,
                                          self.endTangent.result.normalizedTangent,
                                          endOffsetVec.output,
                                          d.outputTranslate,
                                          name=self.getName('end_crv_mtx'))

        blendInverse = mathOps.inverseMatrix(blend.outputMatrix, name=self.getName('ik_mid_baseInverse_mtx'))
        midTwistRefMtx = mathOps.multiplyMatrices([endCrvMtx.output, blendInverse.outputMatrix],
                                                  name=self.getName('mid_twist_ref_mtx'))
        midTwistMtx2Srt = mathOps.decomposeMatrix(midTwistRefMtx.matrixSum, name=self.getName('mid_twist_mtx2Srt'))
        midTwist = mathOps.isolateRotationOnAxis(midTwistMtx2Srt.outputRotate, 'y', name=self.getName('mid_twist'))

        twistInvert = mathOps.multiplyAngleByScalar(midTwist[1].outputRotateY, .5,
                                                    name=self.getName('ik_mid_twist_invert'))
        twistInvert.output.connect(self.mid_twist_srt.ry)

        blend.outputMatrix.connect(self.ik_mid_ctrl.offsetParentMatrix)

        # Rail
        railPoints = []
        for index in range(guide.num_ctrls):
            num = str(index+1).zfill(2)
            mult = (1.0 / len(self.fk_ctrls))*index
            mult = 1.0 - (mathOps.getDistance((0, 0, 0), ((mult-0.5), 0, 0))*2)
            targ = baseOffsetVec.output
            if index >= guide.num_ctrls / 2:
                targ = endOffsetVec.output
            offsetVec = mathOps.pairBlend(translateA=targ, translateB=midOffsetVec.output, weight=mult,
                                          name=self.getName('rail_%s_offset_vec' % num))
            railPoint = mathOps.addVector([points[index], offsetVec.outTranslate],
                                          name=self.getName('rail_%s_pos' % num))
            railPoints.append(railPoint.output3D)

        self.railCrv = curve.curveThroughPoints(name=self.getName('ik_rail_crv'), positions=initPoints, degree=2)
        self.railCrv.setParent(self.rig)
        for index, point in enumerate(railPoints):
            point.connect(self.railCrv.controlPoints[index])

        # MAP TO GUIDE LOCS
        mappingPairs = [[self.ik_end_ctrl.getParent(), guide.locs[3]], [self.aim_ctrl, guide.locs[4]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

        self.divs = []
        for i, div in enumerate(guide.divisionLocs):
            num = str(i+1).zfill(2)
            mtx = pm.xform(div, q=1, ws=1, m=1)
            node = dag.addChild(self.rig, 'group', name=self.getName('div_%s_srt' % num))
            pm.xform(node, ws=1, m=mtx)
            self.mapToGuideLocs(node, div)
            self.divs.append(node)

        # Head shear srt
        self.end_shear_srt = dag.addChild(self.rig, 'group', name=self.getName('end_shear_srt'))
        transform.align(self.end_shear_srt, self.ik_end_ctrl)
        transform.bakeSrtToOffsetParentMtx(self.end_shear_srt)
        endOffsetMtx = mathOps.createComposeMatrix(inputRotate=(0, 0, 90), name=self.getName('end_srt_offset_mtx'))
        endResultMtx = mathOps.multiplyMatrices([endOffsetMtx.outputMatrix, self.ik_end_ctrl.worldMatrix[0]],
                                                name=self.getName('end_result_mtx'))
        endResultMtx.matrixSum.connect(self.end_shear_srt.offsetParentMatrix)

        # Joints
        if guide.root.add_joint.get():
            for i, div in enumerate(self.divs):
                num = str(i+1).zfill(2)
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                self.joints_list.append({'joint': j, 'driver': div})
                self.mapJointToGuideLocs(j, self.guide.divisionLocs[i])

        # Head shear joint
        j = pm.createNode('joint', name=self.getName('end_shear_jnt'))
        self.joints_list.append({'joint': j, 'driver': self.end_shear_srt})




    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'aim', minValue=0, maxValue=1)

    def addSystems(self):
        # Base skew
        startTangentSum = mathOps.addVector([self.startTangent.result.normalizedTangent,
                                             self.startTangent.result.position],
                                            name=self.getName('start_tangent_sum'))

        startLocalTangent = mathOps.createTransformedPoint(startTangentSum.output3D,
                                                           self.base_srt.worldInverseMatrix[0],
                                                           name=self.getName('start_local_tangent'))
        startAngle = mathOps.angleBetween((0, 1, 0), startLocalTangent.output, name=self.getName('start_angle'))
        
        endTangentSum = mathOps.addVector([self.endTangent.result.normalizedTangent,
                                          self.endTangent.result.position],
                                          name=self.getName('end_tangent_sum'))

        endLocalTangent = mathOps.createTransformedPoint(endTangentSum.output3D,
                                                         self.ik_end_ctrl.worldInverseMatrix[0],
                                                         name=self.getName('end_local_tangent'))
        endAngle = mathOps.angleBetween((0, 1, 0), endLocalTangent.output, name=self.getName('end_angle'))


        
        # ---------------------------------
        # Motion path stuff
        # ---------------------------------
        skewXSum = None
        skewZSum = None
        for i, div in enumerate(self.divs):
            num = str(i+1).zfill(2)
            param = pm.listConnections(self.guide.divisionLocs[i], type='motionPath')[0].uValue.get()
            mp = curve.createMotionPathNode(self.crv, uValue=param, frontAxis='x', upAxis='z',
                                            name=self.getName('%s_mp' % num))
            railMp = curve.createMotionPathNode(self.railCrv, uValue=param, follow=0,
                                                name=self.getName('%s_rail_mp' % num))
            upVec = mathOps.subtractVector([railMp.allCoordinates, mp.allCoordinates],
                                           name=self.getName('%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            mp.allCoordinates.connect(div.t)
            mp.rotate.connect(div.r)

            # Skew
            skewXSum = mathOps.addAngles(startAngle.euler.eulerX, endAngle.euler.eulerX,
                                         name=self.getName('skew_x_sum'))
            skewXSum.weightA.set(1.0-param)
            skewXSum.weightB.set(param)

            skewXMult = mathOps.convert(skewXSum.output, .75,
                                        name=self.getName('%s_skew_x_mult' % num))
            skewXMult.output.connect(div.shearXZ)

            skewZSum = mathOps.addAngles(startAngle.euler.eulerZ, endAngle.euler.eulerZ,
                                         name=self.getName('skew_z_sum'))
            skewZSum.weightA.set(1.0-param)
            skewZSum.weightB.set(param)

            skewZMult = mathOps.convert(skewZSum.output, .75,
                                        name=self.getName('%s_skew_z_mult' % num))
            skewZMult.output.connect(div.shearXY)

        skewXMult = mathOps.convert(skewXSum.output, -.75,
                                        name=self.getName('end_skew_x_mult'))
        skewXMult.output.connect(self.end_shear_srt.shearXZ)

        skewZMult = mathOps.convert(skewZSum.output, -.75,
                                        name=self.getName('end_skew_z_mult'))
        skewZMult.output.connect(self.end_shear_srt.shearXY)

        # ---------------------------------
        # Head Aim
        # ---------------------------------
        headAimMtx = transform.createNonRollMatrix(self.ik_end_ctrl.getParent().worldMatrix[0],
                                                   self.aim_ctrl.worldMatrix[0], axis='z',
                                                   name=[self.getName('head_aim_lock_mtx'),
                                                         self.getName('head_aim_mtx')])
        headAimBlend = transform.blendMatrices(self.ik_end_ctrl.getParent().worldMatrix[0], headAimMtx[1].outputMatrix,
                                               name=self.getName('head_aim_blend_mtx'))
        self.params.aim.connect(headAimBlend.target[0].weight)
        headAimBlend.outputMatrix.connect(self.ik_end_ctrl.offsetParentMatrix)
        self.ik_end_ctrl.inheritsTransform.set(0)

        # ---------------------------------
        # Internal spaces switching setup
        # ---------------------------------
        self.spaces['%s' % (self.ik_end_ctrl.getParent().name())] =\
            'neck_base: %s.worldMatrix[0], neck_tip: %s.worldMatrix[0]' % (self.fk_ctrls[0].name(), self.fk_tip.name())

        self.spaces['%s' % (self.aim_ctrl.name())] = 'neck_tip: %s.worldMatrix[0]' % self.fk_tip.name()

    def finish(self):
        self.setColours(self.guide)
        
        spaceAttrs = [attr for attr in ['ik_end_buffer_srt_parent_space', 'ik_end_buffer_srt_translate_space',
                                        'ik_end_buffer_srt_rotate_space', 'aim'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.ik_end_ctrl)
            
        spaceAttrs = [attr for attr in ['aim_ctrl_parent_space', 'aim_ctrl_translate_space',
                                        'aim_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.aim_ctrl)

        # Lock non-keyable attrs
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList.remove(self.ik_end_ctrl)
        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['rx', 'rz']
        attribute.channelControl(nodeList=[self.ik_mid_ctrl], attrList=attrList)

        '''
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


    '''






def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TNeck01(guide)
'''
TODO:
Joints
Volume preservation
shearing
Finishing (attrs, colours, etc)
'''
