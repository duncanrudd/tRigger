from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve, anim, icon
reload(components)
reload(mathOps)
reload(curve)
reload(anim)

import pymel.core as pm

class TNeck02(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'neck02')
        print 'Created Neck02 Component: %s' % self.comp_name

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
                                name=self.getName('fk_%s' % num), xform=xform, parent=parent, metaParent=parent)
            self.fk_ctrls.append(ctrl)

        # FK tip
        self.fk_tip = dag.addChild(self.rig, 'group', self.getName('fk_tip'))
        self.fk_ctrls[-1].worldMatrix[0].connect(self.fk_tip.offsetParentMatrix)
        self.fk_tip.ty.set(mathOps.getDistance(self.fk_ctrls[-1], guide.locs[3]))

        if self.guide.aimer:
            # Aim ctrl
            xform = transform.getMatrixFromPos(guide.locs[4].worldMatrix.get().translate.get())
            self.aim_ctrl = self.addCtrl(shape='cross', size=ctrlSize,
                                         name=self.getName('aim'), xform=xform, parent=self.controls,
                                         metaParent=self.base_srt)

        parent = self.base_srt
        curveStartCtrl = self.fk_ctrls[0]
        if self.guide.ik_start_ctrl:
            # IK start ctrl

            xform = self.fk_ctrls[0].worldMatrix[0].get()
            self.ik_start_ctrl = self.addCtrl(shape='box', size=ctrlSize,
                                         name=self.getName('ik_start'), xform=xform, parent=self.fk_ctrls[0],
                                         metaParent=parent)
            parent = self.ik_start_ctrl
            curveStartCtrl = self.ik_start_ctrl

        # mid_ctrl
        self.ik_mid_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize * 1.25,
                                        name=self.getName('ik_mid'), xform=self.root.worldMatrix[0].get(),
                                        parent=self.controls, metaParent=parent)

        # IK control
        self.ik_end_ctrl = self.addCtrl(shape='box', size=ctrlSize,
                                        name=self.getName('ik_end'), xform=guide.locs[3].worldMatrix[0].get(),
                                        parent=self.controls, metaParent=self.ik_mid_ctrl, buffer=1)

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
        tipMtx2Srt = mathOps.decomposeMatrix(self.fk_tip.worldMatrix[0], name=self.getName('tip_mtx2Srt'))
        tipOffset = mathOps.subtractVector([d.outputTranslate, tipMtx2Srt.outputTranslate],
                                           name=self.getName('tip_offset_vec'))
        dist1 = mathOps.getDistance(baseMtx, self.ik_end_ctrl)
        dist2 = mathOps.getDistance(baseMtx, curveStartCtrl)
        midRatio = float(dist2)/(dist1+dist2)
        tipOffsetMult = mathOps.multiplyVector(tipOffset.output3D, (midRatio, midRatio, midRatio),
                                               name=self.getName('ik_mid_tip_offset_mult'))
        baseMtx2Srt = mathOps.decomposeMatrix(baseMtx, name=self.getName('ik_mid_base_pos_mtx2Srt'))
        ik_mid_pos_sum = mathOps.addVector([baseMtx2Srt.outputTranslate, tipOffsetMult.output],
                                           name=self.getName('ik_mid_pos_sum'))
        ikMidPosMtx = mathOps.createComposeMatrix(inputTranslate=ik_mid_pos_sum.output3D,
                                                  name=self.getName('ik_mid_pos_mtx'))

        # mid rotate mtx
        endLocalPos = mathOps.createTransformedPoint(d.outputTranslate, curveStartCtrl.worldInverseMatrix[0],
                                                     name=self.getName('ik_end_2_base_space_pos'))
        ikAimAngle = mathOps.angleBetween((0, 1, 0), endLocalPos.output, name=self.getName('ik_aim_angle'))
        ikAngleMtx = mathOps.createComposeMatrix(inputRotate=ikAimAngle.euler, name=self.getName('ik_angle_mtx'))
        ikAimMtx = mathOps.multiplyMatrices([ikAngleMtx.outputMatrix, curveStartCtrl.worldMatrix[0]],
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
        rootMtx2Srt = mathOps.decomposeMatrix(curveStartCtrl.worldMatrix[0], name=self.getName('fk_01_mtx2Srt'))
        points = [rootMtx2Srt.outputTranslate]
        for index, ctrl in enumerate(self.fk_ctrls[1:]):
            num = str(index+2).zfill(2)
            mult = (1.0 / len(self.fk_ctrls))*(index+1)
            dm = mathOps.decomposeMatrix(ctrl.worldMatrix[0], name=self.getName('fk_%s_mtx2Srt' % num))
            tipOffsetMult = mathOps.multiplyVector(tipOffset.output3D, (mult, mult, mult),
                                                   name=self.getName('ik_%s_tip_offset_mult' % num))
            mult = 1.0 - (mathOps.getDistance((0, 0, 0), ((mult-0.5), 0, 0))*2)
            mult = mathOps.ease(mult)
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

        baseSideVec = mathOps.createMatrixAxisVector(curveStartCtrl.worldMatrix[0], (-1, 0, 0),
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

        twistInvert = mathOps.multiplyAngleByScalar(midTwist[1].outputRotateY, midRatio,
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

        self.divs = []
        for i, div in enumerate(guide.divisionLocs):
            num = str(i+1).zfill(2)
            mtx = pm.xform(div, q=1, ws=1, m=1)
            node = dag.addChild(self.rig, 'group', name=self.getName('div_%s_srt' % num))
            pm.xform(node, ws=1, m=mtx)
            self.mapToGuideLocs(node, div)
            self.divs.append(node)

        # MAP TO GUIDE LOCS
        baseNode = self.base_srt
        if self.guide.ik_start_ctrl:
            baseNode = self.ik_start_ctrl
        mappingPairs = [[self.ik_end_ctrl.getParent(), guide.locs[3]], [baseNode, guide.locs[0]],
                        [self.ik_end_ctrl, guide.locs[5]], [self.ik_mid_ctrl, guide.ctrlLocs[len(guide.ctrlLocs)/2]],
                        [curveStartCtrl, guide.ctrlLocs[0]]]
        for div, loc in zip(self.divs, guide.divisionLocs):
            mappingPairs.append([div, loc])
        if self.guide.aimer:
            mappingPairs.append([self.aim_ctrl, guide.locs[4]])
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

        # Joints
        if guide.root.add_joint.get():
            for i, div in enumerate(self.divs):
                num = str(i+1).zfill(2)
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                if i > 0:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': div})
                self.mapJointToGuideLocs(j, self.guide.divisionLocs[i])

        # Head joint
        j = pm.createNode('joint', name=self.getName('end_jnt'))
        j.setParent(self.joints_list[-1]['joint'])
        self.mapJointToGuideLocs(j, self.guide.locs[3])
        self.joints_list.append({'joint': j, 'driver': self.ik_end_ctrl})



    def addAttributes(self):
        if self.guide.aimer:
            attribute.addFloatAttr(self.params, 'aim', minValue=0, maxValue=1)

    def addSystems(self):
        # ---------------------------------
        # Motion path stuff
        # ---------------------------------
        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
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
            baseMtx2Srt.outputScale.connect(div.s)

        # ---------------------------------
        # Head Aim
        # ---------------------------------
        if self.guide.aimer:
            headAimMtx = transform.createNonRollMatrix(self.ik_end_ctrl.getParent().worldMatrix[0],
                                                       self.aim_ctrl.worldMatrix[0], axis='z',
                                                       name=[self.getName('head_aim_lock_mtx'),
                                                             self.getName('head_aim_mtx')])
            headAimBlend = transform.blendMatrices(self.ik_end_ctrl.getParent().worldMatrix[0], headAimMtx[1].outputMatrix,
                                                   name=self.getName('head_aim_blend_mtx'))
            self.params.aim.connect(headAimBlend.target[0].weight)
            headAimBlend.outputMatrix.connect(self.ik_end_ctrl.offsetParentMatrix)

            self.ik_end_ctrl.inheritsTransform.set(0)

            self.spaces['%s' % (self.aim_ctrl.name())] = 'neck_tip: %s.worldMatrix[0]' % self.fk_tip.name()
        else:
            self.fk_tip.worldMatrix[0].connect(self.ik_end_ctrl.getParent().offsetParentMatrix)

        # ---------------------------------
        # Internal spaces switching setup
        # ---------------------------------
        self.spaces['%s' % (self.ik_end_ctrl.getParent().name())] =\
            'neck_base: %s.worldMatrix[0], neck_tip: %s.worldMatrix[0]' % (self.fk_ctrls[0].name(), self.fk_tip.name())

        # Attach params shape to ik end ctrl
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.ik_end_ctrl, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):
        self.setColours(self.guide)

        nodeList = [node for node in self.controls_list if node not in [self.params, self.ik_mid_ctrl]]
        attribute.channelControl(nodeList=nodeList, attrList=['rotateOrder'], keyable=1, lock=0)
        
        spaceAttrs = [attr for attr in ['ik_end_buffer_srt_parent_space', 'ik_end_buffer_srt_translate_space',
                                        'ik_end_buffer_srt_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.ik_end_ctrl)
            
        spaceAttrs = [attr for attr in ['aim_ctrl_parent_space', 'aim_ctrl_translate_space',
                                        'aim_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.aim_ctrl)

        spaceAttrs = [attr for attr in ['fk_01_ctrl_parent_space', 'fk_01_ctrl_translate_space',
                                        'fk_01_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.fk_ctrls[0])

        attrList = [attr for attr in ['shear_amount', 'aim'] if pm.hasAttr(self.params, attr)]
        for attr in attrList:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.ik_end_ctrl)

        # Lock non-keyable attrs
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList.remove(self.ik_end_ctrl)
        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['rx', 'rz']
        attribute.channelControl(nodeList=[self.ik_mid_ctrl], attrList=attrList)

        if self.ik_end_ctrl not in self.controls_list:
            self.controls_list.append(self.ik_end_ctrl)



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
        initialParam = curve.getNearestPointOnCurve(self.crv, requestedFrom)
        mp = curve.createMotionPathNode(self.crv, uValue=initialParam, frontAxis='x', upAxis='y', wut=1,
                                        name=self.getName('%s_crvOutput_mp' % name))
        railMp = curve.createMotionPathNode(self.railCrv, uValue=initialParam, follow=0,
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
            paramSum = mathOps.addScalar([paramAttr, initialParam], name=self.getName('%s_crvOutput_sum' % name))
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
    return TNeck02(guide)
'''
TODO:
Joints
Volume preservation
shearing
Finishing (attrs, colours, etc)
'''
