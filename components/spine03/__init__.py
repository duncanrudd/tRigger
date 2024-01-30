
from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve, anim, icon
reload(components)
reload(mathOps)
reload(curve)
reload(anim)

import pymel.core as pm

class TSpine03(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index,
                                           'spine03')
        print
        'Created Spine03 Component: %s' % self.comp_name

    def addObjects(self, guide):
        ctrlSize = mathOps.getDistance(guide.locs[-1], guide.root) * .3

        def _getVec(start, end, invert=0):
            start, end = mathOps.getStartAndEnd(start, end)
            if invert:
                return (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
            else:
                return (pm.datatypes.Vector(start) - pm.datatypes.Vector(end)).normal()

        # FK controls
        self.fk_ctrls = []
        for index in range(guide.num_ctrls):
            num = str(index + 1).zfill(2)
            loc = guide.ctrlLocs[index]
            if index > 0:
                upVecTemp = mathOps.getMatrixAxisAsVector(self.controls_list[-1].worldMatrix[0], 'z')
                if index != guide.num_ctrls-1:
                    aimVec = _getVec(guide.ctrlLocs[index + 1], loc)
                    sideVec = aimVec.cross(upVecTemp).normal()
                    upVec = sideVec.cross(aimVec).normal()
                    startPos = pm.xform(loc, q=1, ws=1, t=1)
                    xform = pm.datatypes.Matrix(sideVec, aimVec, upVec, startPos)
                else:
                    xform = guide.locs[3].worldMatrix[0].get()
                parent = self.controls_list[-1]
            else:
                xform = self.base_srt.worldMatrix[0].get()
                parent = self.base_srt
            ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                name=self.getName('fk_%s' % num), xform=xform, parent=parent, metaParent=parent)
            self.fk_ctrls.append(ctrl)
        # fK hip
        self.fk_start_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.75,
                                          name=self.getName('fk_start'), xform=self.fk_ctrls[0].worldMatrix[0].get(),
                                          parent=self.base_srt, metaParent=self.fk_ctrls[0])

        # IK start control
        self.ik_start_ctrl = self.addCtrl(shape='box', size=ctrlSize,
                                          name=self.getName('ik_start'), xform=guide.root.worldMatrix[0].get(),
                                          parent=self.base_srt, metaParent=self.base_srt)

        # IK end control
        self.ik_end_ctrl = self.addCtrl(shape='box', size=ctrlSize,
                                        name=self.getName('ik_end'), xform=guide.locs[3].worldMatrix[0].get(),
                                        parent=self.base_srt, metaParent=self.ik_start_ctrl)

        # mid_ctrl
        self.ik_mid_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize * 1.25,
                                        name=self.getName('ik_mid'), xform=self.root.worldMatrix[0].get(),
                                        parent=self.controls, metaParent=self.base_srt)
        # mid twist srt
        self.mid_twist_srt = dag.addChild(self.rig, 'group', name=self.getName('mid_twist_srt'))
        self.ik_mid_ctrl.worldMatrix[0].connect(self.mid_twist_srt.offsetParentMatrix)

        initPoints = [(i, 0, 0) for i in range(guide.num_ctrls)]

        self.ik_crv = curve.curveThroughPoints(name=self.getName('ik_crv'), positions=initPoints, degree=2)
        self.ik_crv.setParent(self.rig)

        self.ik_railCrv = curve.curveThroughPoints(name=self.getName('ik_rail_crv'), positions=initPoints,
                                                   degree=2)
        self.ik_railCrv.setParent(self.rig)

        # End srt - subsequent components are parented to this
        self.end_srt = dag.addChild(self.rig, 'group', name=self.getName('end_out_srt'))

        self.tweak_ctrls = []
        metaParent = self.base_srt
        for i, div in enumerate(guide.divisionLocs):
            num = str(i + 1).zfill(2)
            mtx = div.worldMatrix[0].get()
            ctrl = self.addCtrl(shape='box', size=ctrlSize*.25,
                                name=self.getName('tweak_%s' % num), xform=mtx,
                                parent=self.controls, metaParent=metaParent, buffer=1)
            metaParent = ctrl
            self.mapToGuideLocs(ctrl, div)
            self.tweak_ctrls.append(ctrl)

        # MAP TO GUIDE LOCS
        mappingPairs = [[self.fk_start_ctrl, guide.root], [self.end_srt, guide.locs[3]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

        # Joints
        if guide.root.add_joint.get():
            for i, ctrl in enumerate(self.tweak_ctrls):
                num = str(i + 1).zfill(2)
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                if i > 0:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': ctrl})
            self.mapJointToGuideLocs(self.joints_list[-1]['joint'], self.guide.locs[3])
            # Add extra joint for FK hips
            j = pm.createNode('joint', name=self.getName('fk_start_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.fk_start_ctrl})

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'ik_fk_blend', minValue=0.0, maxValue=1.0, value=0.0)
        attribute.addFloatAttr(self.params, 'bulge_amount')
        attribute.addFloatAttr(self.params, 'bulge_position', value=0.5)
        attribute.addFloatAttr(self.params, 'bulge_falloff', value=0.5)
        attribute.addFloatAttr(self.params, 'auto_bulge')
        attribute.addBoolAttr(self.params, 'show_tweak_ctrls', value=False)
        attribute.channelControl(nodeList=[self.params], attrList=['show_tweak_ctrls'], lock=0, channelBox=1)

    def addSystems(self):
        def _getOffsetPoint(node, refMtx, targetMtx, name):
            point = (pm.datatypes.Vector(pm.xform(node, q=1, t=1, ws=1))) - pm.datatypes.Vector(
                refMtx.get().translate.get())
            refPoint = mathOps.createTransformedPoint(point, refMtx,
                                                      name=self.getName('ik_%s_ref_point' % name))
            targPoint = mathOps.createTransformedPoint(point, targetMtx,
                                                       name=self.getName('ik_%s_target_point' % name))
            offset = mathOps.subtractVector([targPoint.output, refPoint.output],
                                            name=self.getName('ik_%s_offset' % name))
            return offset

        # Default length
        dist = mathOps.getDistance(self.ik_start_ctrl, self.ik_end_ctrl) * .25

        # mid position mtx
        ikStartMtx2Srt = mathOps.decomposeMatrix(self.ik_start_ctrl.worldMatrix[0],
                                                 name=self.getName('ik_start_mtx2Srt'))
        ikEndMtx2Srt = mathOps.decomposeMatrix(self.ik_end_ctrl.worldMatrix[0],
                                               name=self.getName('ik_end_mtx2Srt'))
        midStartPoint = mathOps.createTransformedPoint((0, dist, 0), self.ik_start_ctrl.worldMatrix[0],
                                                       name=self.getName('ik_mid_start_point'))
        midEndPoint = mathOps.createTransformedPoint((0, -dist, 0), self.ik_end_ctrl.worldMatrix[0],
                                                     name=self.getName('ik_mid_end_point'))
        midPoint = mathOps.averageVector([midStartPoint.output, midEndPoint.output],
                                         name=self.getName('ik_mid_point'))
        ikMidPosMtx = mathOps.createComposeMatrix(inputTranslate=midPoint.output3D,
                                                  name=self.getName('ik_mid_pos_mtx'))

        # mid rotate mtx
        ikStartPosInvert = mathOps.multiplyVector(ikStartMtx2Srt.outputTranslate, (-1, -1, -1),
                                                  name=self.getName('ik_start_pos_neg'))
        aimWorldVec = mathOps.subtractVector(
            [midEndPoint.output, midStartPoint.output, ikStartPosInvert.output],
            name=self.getName('ik_mid_aim_world_vec'))
        aimLocalVec = mathOps.createTransformedPoint(aimWorldVec.output3D,
                                                     self.ik_start_ctrl.worldInverseMatrix[0],
                                                     name=self.getName('ik_mid_aim_local_vec'))
        ikAimAngle = mathOps.angleBetween((0, 1, 0), aimLocalVec.output, name=self.getName('ik_aim_angle'))
        ikAngleMtx = mathOps.createComposeMatrix(inputRotate=ikAimAngle.euler,
                                                 name=self.getName('ik_angle_mtx'))
        ikAimMtx = mathOps.multiplyMatrices([ikAngleMtx.outputMatrix, self.ik_start_ctrl.worldMatrix[0]],
                                            name=self.getName('ik_aim_mtx'))

        blend = transform.blendMatrices(ikMidPosMtx.outputMatrix, ikAimMtx.matrixSum,
                                        name=self.getName('ik_mid_mtx'))
        blend.target[0].weight.set(1)
        blend.target[0].useTranslate.set(0)

        # curve
        startBlend = transform.blendMatrices(self.ik_start_ctrl.worldMatrix[0], self.fk_start_ctrl.worldMatrix[0],
                                             name=self.getName('ik_fk_start_blend_mtx'))
        self.params.ik_fk_blend.connect(startBlend.target[0].weight)
        startMtx2Srt = mathOps.decomposeMatrix(startBlend.outputMatrix, name=self.getName('ik_fk_start_blend_mtx2Srt'))
        points = [startMtx2Srt.outputTranslate]
        midRefMtx2Srt = mathOps.decomposeMatrix(self.ik_mid_ctrl.offsetParentMatrix,
                                                name=self.getName('ik_mid_ref_mtx2Srt'))
        midMtx2Srt = mathOps.decomposeMatrix(self.ik_mid_ctrl.worldMatrix[0],
                                             name=self.getName('ik_mid_mtx2Srt'))
        midOffsetVec = mathOps.subtractVector([midMtx2Srt.outputTranslate, midRefMtx2Srt.outputTranslate],
                                              name=self.getName('ik_mid_offset_vec'))
        for index, ctrl in enumerate(self.fk_ctrls[1:-1]):
            num = str(index + 2).zfill(2)
            weight = (1.0 / (len(self.fk_ctrls) - 1)) * (index+1)
            lowWeight = weight * weight
            highWeight = 1 - ((1-weight)*(1-weight))
            endMult = (lowWeight * (1 - lowWeight)) + (highWeight * lowWeight)
            if weight == 0.5:
                endMult = weight
            elif weight > 0.5:
                endMult = (lowWeight * (1 - highWeight)) + (highWeight * highWeight)

            midWeight = 1.0 - mathOps.getDistance((0, 0, 0), ((weight - 0.5), 0, 0))
            midMult = midWeight * midWeight
            startBasePoint = (ctrl.worldMatrix[0].get() * self.ik_start_ctrl.worldInverseMatrix[0].get()).translate.get()
            startOffsetPoint = mathOps.createTransformedPoint(startBasePoint, self.ik_start_ctrl.worldMatrix[0],
                                                              name=self.getName('ik_%s_start_offsetPoint' % num))
            endBasePoint = (ctrl.worldMatrix[0].get() * self.ik_end_ctrl.worldInverseMatrix[0].get()).translate.get()
            endOffsetPoint = mathOps.createTransformedPoint(endBasePoint, self.ik_end_ctrl.worldMatrix[0],
                                                            name=self.getName('ik_%s_end_offsetPoint' % num))
            ikPoint = mathOps.pairBlend(translateA=startOffsetPoint.output, translateB=endOffsetPoint.output,
                                        name=self.getName('ik_%s_point' % num), weight=endMult)

            midWeightedOffset = mathOps.multiplyVector(midOffsetVec.output3D, (midMult, midMult, midMult),
                                                       name=self.getName('ik_%s_mid_weighted_offset' % num))
            fkPoint = mathOps.decomposeMatrix(ctrl.worldMatrix[0], name=self.getName('fk_%s_ctrl_mtx2Srt' % num))
            resultPoint = mathOps.addVector([midWeightedOffset.output, ikPoint.outTranslate],
                                            name=self.getName('ik_%s_result_point' % num))
            blendPoint = mathOps.pairBlend(translateA=resultPoint.output3D, translateB=fkPoint.outputTranslate,
                                           name=self.getName('crv_%s_point' % num), weight=self.params.ik_fk_blend)
            points.append(blendPoint.outTranslate)

        endBlend = transform.blendMatrices(self.ik_end_ctrl.worldMatrix[0], self.fk_ctrls[-1].worldMatrix[0],
                                           name=self.getName('ik_fk_end_blend_mtx'))
        self.params.ik_fk_blend.connect(endBlend.target[0].weight)
        endMtx2Srt = mathOps.decomposeMatrix(endBlend.outputMatrix, name=self.getName('ik_fk_end_blend_mtx2Srt'))
        points.append(endMtx2Srt.outputTranslate)

        startXAxis = mathOps.createMatrixAxisVector(self.ik_start_ctrl.worldMatrix[0], (-1, 0, 0),
                                                    name=self.getName('start_ctrl_z_axis'))
        midOffsetVec = mathOps.createMatrixAxisVector(self.mid_twist_srt.worldMatrix[0], (-1, 0, 0),
                                                      name=self.getName('mid_rail_offset_vec'))
        endXAxis = mathOps.createMatrixAxisVector(self.ik_end_ctrl.worldMatrix[0], (-1, 0, 0),
                                                  name=self.getName('end_ctrl_z_axis'))

        # Mid twist
        blendInverse = mathOps.inverseMatrix(blend.outputMatrix, name=self.getName('ik_mid_baseInverse_mtx'))
        midTwistRefMtx = mathOps.multiplyMatrices([self.ik_end_ctrl.worldMatrix[0], blendInverse.outputMatrix],
                                                  name=self.getName('mid_twist_ref_mtx'))
        midTwistMtx2Srt = mathOps.decomposeMatrix(midTwistRefMtx.matrixSum,
                                                  name=self.getName('mid_twist_mtx2Srt'))
        midTwist = mathOps.isolateRotationOnAxis(midTwistMtx2Srt.outputRotate, 'y',
                                                 name=self.getName('mid_twist'))

        twistInvert = mathOps.multiplyAngleByScalar(midTwist[1].outputRotateY, .5,
                                                    name=self.getName('ik_mid_twist_invert'))
        twistInvert.output.connect(self.mid_twist_srt.ry)

        blend.outputMatrix.connect(self.ik_mid_ctrl.offsetParentMatrix)

        # Rail
        ikRailPoints = []
        for index in range(self.guide.num_ctrls):
            num = str(index + 1).zfill(2)
            mult = (1.0 / (len(self.fk_ctrls)-1)) * index
            mult = 1.0 - (mathOps.getDistance((0, 0, 0), ((mult - 0.5), 0, 0)) * 2)
            targ = startXAxis.output
            if index >= self.guide.num_ctrls / 2:
                targ = endXAxis.output
            offsetVec = mathOps.pairBlend(translateA=targ, translateB=midOffsetVec.output, weight=mult,
                                          name=self.getName('rail_%s_offset_vec' % num))
            railPoint = mathOps.addVector([points[index], offsetVec.outTranslate],
                                          name=self.getName('rail_%s_pos' % num))

            if index > 0:
                fkCtrl = self.fk_ctrls[index]
            else:
                fkCtrl = self.fk_start_ctrl
            fkRailPoint = mathOps.createTransformedPoint((-1, 0, 0), fkCtrl.worldMatrix[0],
                                                         name=self.getName('rail_%s_fk_point' % num))
            railBlendPoint = mathOps.pairBlend(translateA=railPoint.output3D, translateB=fkRailPoint.output,
                                           name=self.getName('rail_%s_point' % num), weight=self.params.ik_fk_blend)
            ikRailPoints.append(railBlendPoint.outTranslate)

        for index, point in enumerate(points):
            point.connect(self.ik_crv.controlPoints[index])
        for index, point in enumerate(ikRailPoints):
            point.connect(self.ik_railCrv.controlPoints[index])

        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
        crvInfo = curve.createCurveInfo(self.ik_crv, name=self.getName('crv_info'))
        defaultLen = crvInfo.arcLength.get()
        defaultLenScaled = mathOps.multiply(defaultLen, baseMtx2Srt.outputScaleX,
                                            name=self.getName('default_len_scaled'))
        stretchFactor = mathOps.divide(crvInfo.arcLength, defaultLenScaled.output,
                                       name=self.getName('ik_stretch_factor'))
        stretchRev = mathOps.reverse(stretchFactor.outputX, name=self.getName('stretch_rev'))
        stretchAuto = mathOps.multiply(stretchRev.outputX, self.params.auto_bulge,
                                       name=self.getName('bulge_auto_mult'))

        # ---------------------------------
        # Motion path stuff
        # ---------------------------------
        for i, ctrl in enumerate(self.tweak_ctrls):
            buffer = ctrl.getParent()
            if i == -1:
                startMtx2Srt.outputTranslate.connect(buffer.t)
                startMtx2Srt.outputRotate.connect(buffer.r)
                param = 0.0
            elif i == len(self.tweak_ctrls) - 1:
                param = 1.0
                endMtx2Srt.outputTranslate.connect(buffer.t)
                endMtx2Srt.outputRotate.connect(buffer.r)
            else:
                num = str(i + 1).zfill(2)
                param = pm.listConnections(self.guide.divisionLocs[i], type='motionPath')[0].uValue.get()
                mp = curve.createMotionPathNode(self.ik_crv, uValue=param, frontAxis='x', upAxis='y',
                                                name=self.getName('%s_mp' % num))
                railMp = curve.createMotionPathNode(self.ik_railCrv, uValue=param, follow=0,
                                                    name=self.getName('%s_rail_mp' % num))
                upVec = mathOps.subtractVector([railMp.allCoordinates, mp.allCoordinates],
                                               name=self.getName('%s_upVec' % num))
                upVec.output3D.connect(mp.worldUpVector)

                mp.allCoordinates.connect(buffer.t)
                mp.rotate.connect(buffer.r)

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
            bulgeSum = mathOps.addScalar([1, bulgeAmount.output, bulgeAuto.output],
                                         name=self.getName('%s_bulge_sum' % num))
            bulgeResult = mathOps.multiply(bulgeSum.output1D, baseMtx2Srt.outputScaleX,
                                           name=self.getName('%s_bulge_Result' % num))

            bulgeResult.output.connect(buffer.sy)
            bulgeResult.output.connect(buffer.sz)
            baseMtx2Srt.outputScaleX.connect(buffer.sx)

            buffer.offsetParentMatrix.set(pm.datatypes.Matrix())

        # Drive fk start ctrl when in ik mode:
        restMtx = mathOps.createComposeMatrix(name=self.getName('fk_start_restMtx'))
        fkStartMtx = transform.blendMatrices(self.ik_start_ctrl.matrix, restMtx.outputMatrix, weight=self.params.ik_fk_blend,
                                             name=self.getName('fk_start_mtx'))
        fkStartMtx.outputMatrix.connect(self.fk_start_ctrl.offsetParentMatrix)

        # Drive end srt
        endMtx = transform.blendMatrices(self.ik_end_ctrl.worldMatrix[0], self.fk_ctrls[-1].worldMatrix[0], weight=self.params.ik_fk_blend,
                                         name=self.getName('end_out_mtx'))
        endMtx.outputMatrix.connect(self.end_srt.offsetParentMatrix)

        # For some reason this control does not end up in self.controls_list and therefore doesn't get tagged!?!
        # Manually add it
        self.controls_list.append(self.ik_end_ctrl)

        # Attach params shape to base srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.base_srt, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):

        self.setColours(self.guide)

        nodes = [node for node in self.controls_list if node not in ([self.params, self.ik_mid_ctrl] + self.tweak_ctrls)]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        attrList = ['bulge_amount', 'bulge_position', 'bulge_falloff', 'auto_bulge']
        for attr in attrList:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.ik_end_ctrl)

        # Lock non-keyable attrs
        nodeList = [node for node in self.controls_list if not node in self.tweak_ctrls]
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList.remove(self.ik_end_ctrl)
        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['rx', 'rz']
        attribute.channelControl(nodeList=[self.ik_mid_ctrl], attrList=attrList)

        # --------------------------------------------------
        # Set auto vis switch on ik / fk controls
        # --------------------------------------------------
        isIk = pm.createNode('condition', name=self.getName('is_ikfk_1'))
        self.params.ik_fk_blend.connect(isIk.firstTerm)
        isIk.secondTerm.set(1)
        isIk.colorIfTrueR.set(0)
        isIk.colorIfFalseR.set(1)
        for node in [self.ik_start_ctrl, self.ik_mid_ctrl, self.ik_end_ctrl]:
            pm.setAttr(node.visibility, lock=0)
            isIk.outColorR.connect(node.visibility)

        isFk = pm.createNode('condition', name=self.getName('is_ikfk_0'))
        isFk.secondTerm.set(0)
        self.params.ik_fk_blend.connect(isFk.firstTerm)
        isFk.colorIfTrueR.set(0)
        isFk.colorIfFalseR.set(1)
        for node in self.fk_ctrls:
            pm.setAttr(node.visibility, lock=0)
            isFk.outColorR.connect(node.visibility)

        for node in self.tweak_ctrls:

            self.params.show_tweak_ctrls.connect(node.visibility)

    def exposeCurvePointAsOutput(self, requestedFrom, name, live=0):
        '''
        Adds a transform to the component's rig group which is driven along the component's crv. An attribute is
        added on the component's params ctrl to drive the parameter along the curve
        Args:
            requestedFrom: (pm.PyNode) The node that is requesting the output. The initial param value is based
                           on this node's nearest point to the curve.
            name: (string) the name of the new output
            live: (bool) whether or not to expose an animateable attr to control the path parameter of the output
        Returns: (pm.general.Attribute) The newly created matrix attr
        '''
        initialParam = curve.getNearestPointOnCurve(self.ik_crv, requestedFrom)
        mp = curve.createMotionPathNode(self.ik_crv, uValue=initialParam, frontAxis='x', upAxis='y', wut=1,
                                        name=self.getName('%s_crvOutput_mp' % name))
        railMp = curve.createMotionPathNode(self.ik_railCrv, uValue=initialParam, follow=0,
                                            name=self.getName('%s_crvOutput_railMp' % name))
        railMtx = mathOps.createComposeMatrix(inputTranslate=railMp.allCoordinates,
                                              name=self.getName('%s_crvOutput_rail_mtx' % name))
        railMtx.outputMatrix.connect(mp.worldUpMatrix)
        out_srt = dag.addChild(self.rig, 'group', name=self.getName('%s_crvOutput_srt' % name))
        if live:
            if not self.params.hasAttr('CURVE_OUTPUTS___________'):
                attribute.addDividerAttr(self.params, 'CURVE_OUTPUTS')
            paramAttr = attribute.addFloatAttr(self.params, ('%s_path_offset' % name),
                                               minValue=(0.0 - initialParam), maxValue=(1.0 - initialParam))
            paramSum = mathOps.addScalar([paramAttr, initialParam],
                                         name=self.getName('%s_crvOutput_sum' % name))
            paramSum.output1D.connect(mp.uValue)
            paramSum.output1D.connect(railMp.uValue)
        mp.allCoordinates.connect(out_srt.t)
        mp.rotate.connect(out_srt.r)
        d = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0])
        d.outputScale.connect(out_srt.s)
        return out_srt

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSpine03(guide)
