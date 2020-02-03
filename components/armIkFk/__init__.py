from tRigger import components
from tRigger.core import attribute, transform, dag, mathOps, icon, curve, anim
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)
reload(attribute)
reload(curve)

import pymel.core as pm

#[(root_guide'),(02_guide'), (04_guide'), (03_guide'), (05_guide'), (01_guide')]

class TArmIkFk(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'armIkFk')
        print 'Created armIkFk Component: %s' % self.comp_name

    def addObjects(self, guide):
        ctrlSize = mathOps.getDistance(guide.locs[5], guide.locs[3])

        def _getVec(start, end, invert=0):
            start, end = mathOps.getStartAndEnd(start, end)
            if invert:
                return (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
            else:
                return (pm.datatypes.Vector(start) - pm.datatypes.Vector(end)).normal()

        # Build start matrix
        self.invert = (self.comp_side == 'R')
        aimVec = _getVec(guide.locs[1], guide.locs[5], self.invert)
        upVecTemp = _getVec(guide.locs[4], guide.locs[1])
        self.reversePole = (upVecTemp[2] > 0)
        if self.reversePole:
            sideVec = upVecTemp.cross(aimVec).normal()
        else:
            sideVec = aimVec.cross(upVecTemp).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(guide.locs[5], q=1, ws=1, t=1)

        startXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)

        # Build mid matrix
        aimVec = _getVec(guide.locs[3], guide.locs[1])
        sideVec = upVec.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        if self.invert:
            sideVec = aimVec.cross(upVec).normal()
            upVec = sideVec.cross(aimVec).normal()
        startPos = pm.xform(guide.locs[1], q=1, ws=1, t=1)

        midXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)

        # Build end matrix
        endXform = guide.locs[3].worldMatrix[0].get()
        if self.invert:
            endXform = mathOps.invertHandedness(endXform)

        # FK controls
        self.fk_start_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.3,
                                          name=self.getName('fk_start'), xform=startXform,
                                          parent=self.base_srt, buffer=1)
        if self.invert:
            self.fk_start_ctrl.getParent().sx.set(-1)
        self.fk_mid_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.3,
                                        name=self.getName('fk_mid'), xform=midXform, parent=self.fk_start_ctrl)
        self.fk_end_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.3,
                                        name=self.getName('fk_end'), xform=endXform, parent=self.fk_mid_ctrl)

        # --------------------------------
        # IK
        # --------------------------------
        # IK ctrl
        ik_ctrl_mtx = None
        startPos = pm.xform(guide.locs[3], q=1, ws=1, t=1)
        if guide.root.world_aligned_ik_ctrl.get():
            ik_ctrl_mtx = transform.getMatrixFromPos(startPos)
        else:
            aimVec = _getVec(guide.locs[2], guide.locs[3], self.invert)
            upVecTemp = mathOps.getMatrixAxisAsVector(guide.locs[3].worldMatrix[0].get(), 'z')
            sideVec = upVecTemp.cross(aimVec).normal()
            upVec = aimVec.cross(sideVec).normal()
            ik_ctrl_mtx = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        self.ik_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize*.3,
                                    name=self.getName('ik'), xform=ik_ctrl_mtx, parent=self.controls)
        self.ik_displaced = dag.addChild(self.rig, 'group', name=self.getName('ik_displaced_srt'))
        self.ik_ctrl.worldMatrix[0].connect(self.ik_displaced.offsetParentMatrix)

        # Pole Vector ctrl
        pole_ctrl_mtx = transform.getMatrixFromPos(pm.xform(guide.locs[4], q=1, ws=1, t=1))
        self.pole_ctrl = self.addCtrl(shape='ball', size=ctrlSize*.05,
                                      name=self.getName('ik_pole'), xform=pole_ctrl_mtx, parent=self.controls)

        # --------------------------------
        # RESULT
        # --------------------------------
        offsetMult = 1
        if self.invert:
            offsetMult = -1

        # Start
        aimVec = _getVec(guide.locs[1], guide.locs[5], self.invert)
        upVecTemp = _getVec(guide.locs[4], guide.locs[1])
        if self.reversePole:
            sideVec = upVecTemp.cross(aimVec).normal()
        else:
            sideVec = aimVec.cross(upVecTemp).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(guide.locs[5], q=1, ws=1, t=1)
        startXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        self.result_start = dag.addChild(self.rig, 'group', name=self.getName('result_start_srt'))
        self.result_start.offsetParentMatrix.set(startXform)
        tipPos = pm.xform(guide.locs[1], q=1, ws=1, t=1)
        tipXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, tipPos)
        self.result_startTip = dag.addChild(self.rig, 'group', name=self.getName('result_startTip_srt'))
        self.result_startTip.offsetParentMatrix.set(tipXform)

        # BendyControls
        self.upper_startBend_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.15,
                                                 name=self.getName('upper_startBend'),
                                                 xform=startXform, parent=self.controls)
        self.upper_endBend_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.15,
                                               name=self.getName('upper_endBend'),
                                               xform=tipXform, parent=self.controls)

        # Build mid matrix
        aimVec = _getVec(guide.locs[3], guide.locs[1], self.invert)
        sideVec = upVec.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        midXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, tipPos)
        self.result_mid = dag.addChild(self.rig, 'group', name=self.getName('result_mid_srt'))
        self.result_mid.offsetParentMatrix.set(midXform)
        tipPos = pm.xform(guide.locs[3], q=1, ws=1, t=1)
        tipXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, tipPos)
        self.result_midTip = dag.addChild(self.rig, 'group', name=self.getName('result_midTip_srt'))
        self.result_midTip.offsetParentMatrix.set(tipXform)

        # BendyControls
        self.lower_startBend_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.15,
                                                 name=self.getName('lower_startBend'),
                                                 xform=midXform, parent=self.controls)
        self.lower_endBend_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.15,
                                               name=self.getName('lower_endBend'),
                                               xform=tipXform, parent=self.controls)

        # Build end matrix
        aimVec = _getVec(guide.locs[2], guide.locs[3], self.invert)
        upVecTemp = mathOps.getMatrixAxisAsVector(guide.locs[3].worldMatrix[0].get(), 'z')
        sideVec = upVecTemp.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(self.fk_end_ctrl, q=1, ws=1, t=1)
        endXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        self.result_end = dag.addChild(self.rig, 'group', name=self.getName('result_end_srt'))
        self.result_end.offsetParentMatrix.set(endXform)

        # Mid Ctrl
        m = transform.getBlendedMatrix([self.fk_start_ctrl, self.fk_mid_ctrl])
        midCtrl_xform = pm.datatypes.Matrix(m[0], m[1], m[2], list(midXform.translate.get()) + [0.0])
        self.mid_ctrl = self.addCtrl(shape='ball', size=ctrlSize*.1,
                                     name=self.getName('mid'),
                                     xform=midCtrl_xform,
                                     parent=self.controls)

        # AVG srts
        self.avgSrts = []
        self.result_start_avg = dag.addChild(self.rig, 'group', name=self.getName('result_start_avg_srt'))
        self.result_start_avg.offsetParentMatrix.set(transform.getBlendedMatrix([self.base_srt, self.result_start]))
        self.avgSrts.append(self.result_start_avg)

        self.result_mid_avg = dag.addChild(self.rig, 'group', name=self.getName('result_mid_avg_srt'))
        self.result_mid_avg.offsetParentMatrix.set(transform.getBlendedMatrix([self.result_startTip, self.result_mid]))
        self.avgSrts.append(self.result_mid_avg)

        self.result_end_avg = dag.addChild(self.rig, 'group', name=self.getName('result_end_avg_srt'))
        self.result_end_avg.offsetParentMatrix.set(transform.getBlendedMatrix([self.result_midTip, self.result_end]))
        self.avgSrts.append(self.result_end_avg)

        # MAP TO GUIDE LOCS
        mappingPairs = [[self.pole_ctrl, guide.locs[4]], [self.ik_ctrl, guide.locs[3]], [self.base_srt, guide.root],
                        [self.result_start, guide.locs[5]], [self.result_mid, guide.locs[1]],
                        [self.result_end, guide.locs[2]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

        # -----------------------------------------
        # Twist segments
        # -----------------------------------------
        self.upper_crv = curve.curveBetweenPoints(self.result_start_avg, self.mid_ctrl, numPoints=4,
                                                  degree=3, name=self.getName('upperTwist_crv'))
        self.upper_crv.setParent(self.rig)
        self.upper_mps = curve.nodesAlongCurve(self.upper_startBend_ctrl.worldMatrix[0],
                                               self.upper_endBend_ctrl.worldMatrix[0],
                                               guide.num_divisions, self.upper_crv,
                                               self.getName('upperTwist'))

        self.lower_crv = curve.curveBetweenPoints(self.mid_ctrl, self.result_end_avg, numPoints=4,
                                                  degree=3, name=self.getName('lowerTwist_crv'))
        self.lower_crv.setParent(self.rig)
        self.lower_mps = curve.nodesAlongCurve(self.lower_startBend_ctrl.worldMatrix[0],
                                               self.lower_endBend_ctrl.worldMatrix[0], guide.num_divisions,
                                               self.lower_crv, self.getName('lowerTwist'))

        # Divs and Joints
        def _makeDiv(name, mp):
            node = dag.addChild(self.rig, 'group', name=name)
            mp.allCoordinates.connect(node.t)
            mp.rotate.connect(node.r)
            self.base_srt.s.connect(node.s)
            return node

        self.upperDivs = []
        self.lowerDivs = []
        for i in range(guide.num_divisions):
            num = str(i+1).zfill(2)
            self.upperDivs.append(_makeDiv(self.getName('upperDiv_%s_srt' % num), self.upper_mps[i]))
            self.lowerDivs.append(_makeDiv(self.getName('lowerDiv_%s_srt' % num), self.lower_mps[i]))

        # ----------
        # SLEEVES
        # ----------
        self.upperSleeve_mps = []
        self.lowerSleeve_mps = []
        self.upperSleeveDivs = []
        self.lowerSleeveDivs = []

        if self.guide.sleeve in [1, 2, 4]:
            self.upper_sleeve_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.25,
                                                  name=self.getName('upper_sleeve'),
                                                  xform=self.result_start.worldMatrix[0].get(),
                                                  parent=self.controls)
            self.result_start.worldMatrix[0].connect(self.upper_sleeve_ctrl.offsetParentMatrix)
            self.upperSleeve_mps = curve.nodesAlongCurve(self.upper_startBend_ctrl.worldMatrix[0],
                                                         self.upper_endBend_ctrl.worldMatrix[0],
                                                         guide.num_divisions, self.upper_crv,
                                                         self.getName('upperSleeve_twist'))
            for i in range(guide.num_divisions):
                num = str(i+1).zfill(2)
                mp = self.upperSleeve_mps[i]
                self.upperSleeveDivs.append(dag.addChild(self.rig, 'group',
                                                         name=self.getName('upperSleeveDiv_%s_srt' % num)))
                worldMtx = mathOps.createComposeMatrix(inputTranslate=mp.allCoordinates, inputRotate=mp.rotate,
                                                       name=self.getName('upperSleeve_%s_world_mtx' % num))
                localMtx = mathOps.multiplyMatrices([worldMtx.outputMatrix, self.result_start.worldInverseMatrix[0],
                                                     self.upper_sleeve_ctrl.worldMatrix[0]],
                                                    name=self.getName('upperSleeve_%s_mtx' % num))
                localMtx.matrixSum.connect(self.upperSleeveDivs[-1].offsetParentMatrix)

            if self.guide.sleeve == 2:
                self.lower_sleeve_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.25,
                                                      name=self.getName('lower_sleeve'),
                                                      xform=self.mid_ctrl.worldMatrix[0].get(),
                                                      parent=self.upper_sleeve_ctrl)
                self.lowerSleeve_mps = curve.nodesAlongCurve(self.upperSleeveDivs[-1].worldMatrix[0],
                                                             self.lower_endBend_ctrl.worldMatrix[0],
                                                             guide.num_divisions, self.lower_crv,
                                                             self.getName('lowerSleeve_twist'))
                self.lowerSleeveBaseMtx = transform.blend_T_R_matrices(self.mid_ctrl.worldMatrix[0],
                                                                       self.result_start.worldMatrix[0],
                                                                       name=self.getName('lowerSleeve_base_mtx'))
                lowerSleeveAimMtx = transform.createAimMatrix(self.lowerSleeveBaseMtx.outputMatrix,
                                                              self.result_end.worldMatrix[0],
                                                              name=self.getName('lowerSleeve_aim_mtx'))
                lowerSleeveInverse = mathOps.inverseMatrix(lowerSleeveAimMtx.outputMatrix,
                                                           name=self.getName('lowerSleeve_aim_inverse_mtx'))
                lowerSleeveMtx = mathOps.multiplyMatrices([lowerSleeveAimMtx.outputMatrix,
                                                           self.result_start.worldInverseMatrix[0]],
                                                          name=self.getName('lowerSleeve_mtx'))
                lowerSleeveMtx.matrixSum.connect(self.lower_sleeve_ctrl.offsetParentMatrix)


                for i in range(guide.num_divisions):
                    num = str(i+1).zfill(2)
                    mp = self.lowerSleeve_mps[i]
                    self.lowerSleeveDivs.append(dag.addChild(self.rig, 'group',
                                                             name=self.getName('lowerSleeveDiv_%s_srt' % num)))
                    worldMtx = mathOps.createComposeMatrix(inputTranslate=mp.allCoordinates, inputRotate=mp.rotate,
                                                           name=self.getName('lowerSleeve_%s_world_mtx' % num))
                    localMtx = mathOps.multiplyMatrices([worldMtx.outputMatrix, lowerSleeveInverse.outputMatrix,
                                                         self.lower_sleeve_ctrl.worldMatrix[0]],
                                                        name=self.getName('lowerSleeve_%s_mtx' % num))
                    localMtx.matrixSum.connect(self.lowerSleeveDivs[-1].offsetParentMatrix)

                self.sleeve_mid_avg = dag.addChild(self.rig, 'group', name=self.getName('sleeve_mid_avg_srt'))

        if self.guide.sleeve in [3, 4]:
            self.lower_sleeve_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.25,
                                                  name=self.getName('lower_sleeve'),
                                                  xform=self.result_midTip.worldMatrix[0].get(),
                                                  parent=self.controls)
            self.result_midTip.worldMatrix[0].connect(self.lower_sleeve_ctrl.offsetParentMatrix)
            self.lowerSleeve_mps = curve.nodesAlongCurve(self.lower_endBend_ctrl.worldMatrix[0],
                                                         self.lower_startBend_ctrl.worldMatrix[0],
                                                         guide.num_divisions, self.lower_crv,
                                                         self.getName('lowerSleeve_twist'))
            for i in range(guide.num_divisions):
                num = str(i+1).zfill(2)
                mp = self.lowerSleeve_mps[i]
                self.lowerSleeveDivs.append(dag.addChild(self.rig, 'group',
                                                         name=self.getName('lowerSleeveDiv_%s_srt' % num)))
                worldMtx = mathOps.createComposeMatrix(inputTranslate=mp.allCoordinates, inputRotate=mp.rotate,
                                                       name=self.getName('lowerSleeve_%s_world_mtx' % num))
                localMtx = mathOps.multiplyMatrices([worldMtx.outputMatrix, self.result_midTip.worldInverseMatrix[0],
                                                     self.lower_sleeve_ctrl.worldMatrix[0]],
                                                    name=self.getName('lowerSleeve_%s_mtx' % num))
                localMtx.matrixSum.connect(self.lowerSleeveDivs[-1].offsetParentMatrix)

        # Joints
        if self.guide.add_joint:
            j = pm.createNode('joint', name=self.getName('start_avg_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.result_start_avg})

            j = pm.createNode('joint', name=self.getName('mid_avg_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.result_mid_avg})

            j = pm.createNode('joint', name=self.getName('end_avg_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.result_end_avg})

            for i, div in enumerate(self.upperDivs + self.lowerDivs + self.upperSleeveDivs + self.lowerSleeveDivs):
                j = pm.createNode('joint', name=div.name().replace('srt', 'jnt'))
                self.joints_list.append({'joint': j, 'driver': div})
            if self.guide.sleeve == 2:
                j = pm.createNode('joint', name=self.getName('sleeve_mid_avg_jnt'))
                self.joints_list.append({'joint': j, 'driver': self.sleeve_mid_avg})

        # Call overloaded method of parent class
        components.TBaseComponent.addObjects(self, guide)

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'ikfk_blend', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'mid_twist', minValue=-1, maxValue=1)
        attribute.addFloatAttr(self.params, 'upper_twist', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'lower_twist', minValue=-1, maxValue=0)
        if self.guide.sleeve != 0:
            attribute.addFloatAttr(self.params, 'sleeve_twist', minValue=0, maxValue=1)
            if self.guide.sleeve == 4:
                attribute.addFloatAttr(self.params, 'lower_sleeve_pull', minValue=0, maxValue=1)
                attribute.addFloatAttr(self.params, 'upper_sleeve_pull', minValue=0, maxValue=1)
            else:
                attribute.addFloatAttr(self.params, 'sleeve_pull', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'mid_slide', minValue=-1, maxValue=1)
        attribute.addFloatAttr(self.params, 'stretch', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'pin_to_pole', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'softness', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'extend', minValue=0, value=1)
        attribute.addFloatAttr(self.params, 'roundness', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'round_radius', minValue=.001, maxValue=1, value=.001)
        attribute.addFloatAttr(self.params, 'chamfer', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'start_tangent', minValue=.001, maxValue=1, value=.001)
        attribute.addFloatAttr(self.params, 'start_follow', minValue=.001, maxValue=1, value=1)
        attribute.addFloatAttr(self.params, 'end_tangent', minValue=.001, maxValue=1, value=.001)
        attribute.addFloatAttr(self.params, 'end_follow', minValue=.001, maxValue=1, value=1)
        attribute.addFloatAttr(self.params, 'volume_preserve', minValue=0.0)
        attribute.addFloatAttr(self.params, 'volume_falloff', minValue=0.0, maxValue=2.0)

        # Call overloaded method of parent class
        #components.TBaseComponent.addAttributes(self)

    def addSystems(self):
        # --------------------------------
        # IK
        # --------------------------------
        # Non roll base - starting point for basis and stable parent space for pole ctrl
        axis = 'x'
        angle=(1, 0, 0)
        if self.invert:
            axis = '-x'
            angle=(-1, 0, 0)
        d = mathOps.decomposeMatrix(self.ik_displaced.worldMatrix[0])
        targToBaseVec = mathOps.createTransformedPoint(d.outputTranslate, self.base_srt.worldInverseMatrix[0],
                                                       name=self.getName('ik_targ_to_base_vec'))
        base_angle = mathOps.angleBetween(angle, targToBaseVec.output, name=self.getName('ik_base_angle'))
        base_angle_mtx = mathOps.createComposeMatrix(inputRotate=base_angle.euler,
                                                     name=self.getName('ik_base_angle_mtx'))
        base_mtx = mathOps.multiplyMatrices([base_angle_mtx.outputMatrix, self.base_srt.worldMatrix[0]],
                                            name=self.getName('ik_base_mtx'))
        ik_basis_mtx = pm.createNode('aimMatrix', name=self.getName('base_nonRoll_aim_mtx'))
        base_mtx.matrixSum.connect(ik_basis_mtx.inputMatrix)
        ik_basis_mtx.primaryMode.set(0)
        ik_basis_mtx.primaryInputAxis.set((1, 0, 0))
        ik_basis_mtx.secondaryMode.set(1)
        if self.reversePole:
            ik_basis_mtx.secondaryInputAxis.set((0, 0, 1))
        else:
            ik_basis_mtx.secondaryInputAxis.set((0, 0, -1))
        self.pole_ctrl.worldMatrix[0].connect(ik_basis_mtx.secondaryTargetMatrix)

        # --IK SOLVER--
        #   Get main distances and ratios
        d = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
        targetDist = mathOps.distance(self.base_srt, self.ik_displaced, name=self.getName('ik_target_dist'))
        targetDistScaled = mathOps.divide(targetDist.distance, d.outputScaleX,
                                          name=self.getName('ik_target_dist_normal'))

        chainLen = mathOps.addScalar([mathOps.getDistance(self.fk_start_ctrl, self.fk_mid_ctrl),
                                      mathOps.getDistance(self.fk_mid_ctrl, self.fk_end_ctrl)],
                                     name=self.getName('ik_chain_len'))
        extendLen = mathOps.multiply(chainLen.output1D, self.params.extend, name=self.getName('ik_extend_len'))

        softLen = mathOps.multiply(extendLen.output, self.params.softness, name=self.getName('softLen'))

        softStartLen = mathOps.subtractScalar([extendLen.output, softLen.output], name=self.getName('soft_start_len'))

        softDist = mathOps.subtractScalar([targetDistScaled.outputX, softStartLen.output1D],
                                          name=self.getName('soft_dist'))

        softDistInvert = mathOps.multiply(softDist.output1D, -1, name=self.getName('soft_dist_invert'))

        isSoftZero = pm.createNode('condition', name=self.getName('is_soft_zero'))
        self.params.softness.connect(isSoftZero.firstTerm)
        isSoftZero.colorIfTrueR.set(1)
        isSoftZero.colorIfFalseR.set(2)

        softOverSoftLen = mathOps.divide(softDistInvert.output, softLen.output,
                                         name=self.getName('softDistInvert_over_softLen'))
        isSoftZero.outColorR.connect(softOverSoftLen.operation)

        baseLogPow = mathOps.power(2.718, softOverSoftLen.outputX, name=self.getName('base_log_pow'))

        softLenByPow = mathOps.multiply(baseLogPow.outputX, softLen.output, name=self.getName('baseLogPow_by_softLen'))

        subFromExtendLen = mathOps.subtractScalar([extendLen.output, softLenByPow.output],
                                                  name=self.getName('subtract_from_extendLen'))

        isSoft = pm.createNode('condition', name=self.getName('is_soft'))
        isSoft.operation.set(2)
        softStartLen.output1D.connect(isSoft.firstTerm)
        targetDistScaled.outputX.connect(isSoft.secondTerm)
        subFromExtendLen.output1D.connect(isSoft.colorIfFalseR)
        targetDistScaled.outputX.connect(isSoft.colorIfTrueR)

        softToTargDist = mathOps.subtractScalar([targetDistScaled.outputX, isSoft.outColorR],
                                                name=self.getName('soft_to_targ_dist'))

        stretchBlend = mathOps.blendScalarAttrs(isSoft.outColorR, targetDistScaled.outputX, self.params.stretch,
                                                name=self.getName('stretch_blend'))
        startPoleDist = mathOps.distance(self.base_srt, self.pole_ctrl, name=self.getName('ik_start2Pole_dist'))
        startPoleDistScaled = mathOps.divide(startPoleDist.distance, d.outputScaleX,
                                             name=self.getName('ik_start2Pole_dist_normal'))
        endPoleDist = mathOps.distance(self.ik_displaced, self.pole_ctrl, name=self.getName('ik_end2Pole_dist'))
        endPoleDistScaled = mathOps.divide(endPoleDist.distance, d.outputScaleX,
                                           name=self.getName('ik_end2Pole_dist_normal'))

        upperRatio = mathOps.addScalar([mathOps.getDistance(self.fk_start_ctrl,self.fk_mid_ctrl)/chainLen.output1D.get(),
                                        self.params.mid_slide], name=self.getName('upper_ratio'))
        upperRatioClamp = mathOps.clamp(upperRatio.output1D, 0.1, 0.9, name=self.getName('upper_ratio_clamp'))
        lowerRatio = mathOps.subtractScalar([1.0, upperRatioClamp.outputR], name=self.getName('lower_ratio'))

        upperSoftLen = mathOps.multiply(softToTargDist.output1D, upperRatioClamp.outputR,
                                        name=self.getName('upper_soft_len'))
        lowerSoftLen = mathOps.multiply(softToTargDist.output1D, lowerRatio.output1D,
                                        name=self.getName('lower_soft_len'))

        upper_softStretchy_len = pm.createNode('animBlendNodeAdditive', name=self.getName('upper_softStretchy_len'))
        upperSoftLen.output.connect(upper_softStretchy_len.inputA)
        extendLen.output.connect(upper_softStretchy_len.inputB)
        self.params.stretch.connect(upper_softStretchy_len.weightA)
        upperRatioClamp.outputR.connect(upper_softStretchy_len.weightB)
        
        lower_softStretchy_len = pm.createNode('animBlendNodeAdditive', name=self.getName('lower_softStretchy_len'))
        lowerSoftLen.output.connect(lower_softStretchy_len.inputA)
        extendLen.output.connect(lower_softStretchy_len.inputB)
        self.params.stretch.connect(lower_softStretchy_len.weightA)
        lowerRatio.output1D.connect(lower_softStretchy_len.weightB)

        lowerPinStretchBlend = mathOps.blendScalarAttrs(stretchBlend.output, targetDistScaled.outputX,
                                                        self.params.pin_to_pole,
                                                        name=self.getName('lower_pin_stretch_blend'))

        upperResult = mathOps.blendScalarAttrs(upper_softStretchy_len.output, startPoleDistScaled.outputX,
                                               self.params.pin_to_pole, name=self.getName('upper_result_len'))

        lowerSolverLen = mathOps.blendScalarAttrs(lower_softStretchy_len.output, endPoleDistScaled.outputX,
                                                  self.params.pin_to_pole, name=self.getName('lower_solver_len'))

        lowerResult = mathOps.blendScalarAttrs(lower_softStretchy_len.output, lowerSolverLen.output,
                                               self.params.stretch, name=self.getName('lower_result_len'))

        ik_start_angle = pm.createNode('animBlendNodeAdditiveDA', name=self.getName('ik_start_angle'))
        ik_mid_angle = pm.createNode('animBlendNodeAdditiveDA', name=self.getName('ik_mid_angle'))
        if self.invert:
            ik_start_angle.weightA.set(-1)
            ik_mid_angle.weightA.set(-1)
        if self.reversePole:
            ik_start_angle.weightA.set(ik_start_angle.weightA.get()*-1)
            ik_mid_angle.weightA.set(ik_mid_angle.weightA.get()*-1)

        # Expression
        exprString = 'float $upLen = %s.output;\n' % upperResult.name()
        exprString += 'float $lowLen = %s.output;\n' % lowerSolverLen.name()
        exprString += 'float $restLen = $upLen+$lowLen;\n'
        exprString += 'float $targLen = %s.output;\n\n' % lowerPinStretchBlend.name()
        exprString += 'float $upSq = $upLen*$upLen;\n'
        exprString += 'float $lowSq = $lowLen*$lowLen;\n'
        exprString += 'float $targSq = $targLen*$targLen;\n\n'
        exprString += 'float $startAngle = 0.0;\n'
        exprString += 'float $midAngle = 0.0;\n\n'
        exprString += 'if ($targLen < $restLen){;\n\t'
        exprString += '$startAngle = acos(($upSq + $targSq - $lowSq)/(2 * $upLen * $targLen));\n\t'
        exprString += '$startAngle = acos(($upSq + $targSq - $lowSq)/(2 * $upLen * $targLen));\n\t'
        exprString += '$midAngle = acos(($upSq + $lowSq - $targSq)/(2 * $upLen * $lowLen)) - deg_to_rad(180);\n}\n\n'
        exprString += '%s.inputA = $startAngle;\n' % ik_start_angle.name()
        exprString += '%s.inputA = $midAngle;' % ik_mid_angle.name()

        expr = pm.expression(s=exprString, name=self.getName('ik_expr'), alwaysEvaluate=0, unitConversion='none')

        # --------------------------------
        # RESULT
        # --------------------------------
        # FK Matrices
        fkAttrs = [self.fk_start_ctrl.worldMatrix[0], self.fk_mid_ctrl.worldMatrix[0], self.fk_end_ctrl.worldMatrix[0]]
        if self.invert:
            def _offsetMtx(source, dest, name, index):
                offset = dest.worldMatrix[0].get() * source.worldInverseMatrix[0].get()
                offset_mtx = mathOps.vectors2Mtx44(offset[0], offset[1], offset[2], offset[3],
                                                   name=self.getName('fk_%s_offset_mtx' % name))
                mtx = mathOps.multiplyMatrices([offset_mtx.output, source.worldMatrix[0]],
                                               name=self.getName('fk_%s_mtx') % name)
                fkAttrs[index] = mtx.matrixSum

            _offsetMtx(self.fk_start_ctrl, self.result_start, 'start', 0)
            _offsetMtx(self.fk_mid_ctrl, self.result_mid, 'mid', 1)
            _offsetMtx(self.fk_end_ctrl, self.result_end, 'end', 2)

        # IK Matrices
        # -- Start
        ik_start_rotate_mtx = pm.createNode('composeMatrix', name=self.getName('ik_start_rotate_mtx'))
        ik_start_angle.output.connect(ik_start_rotate_mtx.inputRotateY)
        ik_start_mtx = mathOps.multiplyMatrices([ik_start_rotate_mtx.outputMatrix, ik_basis_mtx.outputMatrix],
                                                name=self.getName('ik_start_mtx'))
        # -- Mid
        ik_mid_rotate_mtx = pm.createNode('composeMatrix', name=self.getName('ik_mid_rotate_mtx'))
        ik_mid_angle.output.connect(ik_mid_rotate_mtx.inputRotateY)
        if self.invert:
            inv = mathOps.multiply(upperResult.output, -1, name=self.getName('ik_upperLen_invert'))
            inv.output.connect(ik_mid_rotate_mtx.inputTranslateX)
        else:
            upperResult.output.connect(ik_mid_rotate_mtx.inputTranslateX)
        ik_mid_mtx = mathOps.multiplyMatrices([ik_mid_rotate_mtx.outputMatrix, ik_start_mtx.matrixSum],
                                              name=self.getName('ik_mid_mtx'))

        # -- End
        offset = self.result_end.worldMatrix[0].get() * self.ik_displaced.worldInverseMatrix[0].get()
        ik_end_rotate_mtx = mathOps.vectors2Mtx44(offset[0], offset[1], offset[2],
                                                  name=self.getName('ik_end_rotate_mtx'))
        ik_end_translate_mtx = pm.createNode('composeMatrix', name=self.getName('ik_end_translate_mtx'))
        if self.invert:
            inv = mathOps.multiply(lowerResult.output, -1, name=self.getName('ik_lowerLen_invert'))
            inv.output.connect(ik_end_translate_mtx.inputTranslateX)
        else:
            lowerResult.output.connect(ik_end_translate_mtx.inputTranslateX)
        ik_end_base_mtx = mathOps.multiplyMatrices([ik_end_rotate_mtx.output,
                                                    self.ik_displaced.worldMatrix[0]],
                                                   name=self.getName('ik_end_base_mtx'))
        ik_end_place_mtx = mathOps.multiplyMatrices([ik_end_translate_mtx.outputMatrix,
                                                    ik_mid_mtx.matrixSum],
                                                    name=self.getName('ik_end_place_mtx'))
        ik_end_mtx = pm.createNode('blendMatrix', name=self.getName('ik_end_mtx'))
        ik_end_base_mtx.matrixSum.connect(ik_end_mtx.inputMatrix)
        ik_end_place_mtx.matrixSum.connect(ik_end_mtx.target[0].targetMatrix)
        ik_end_mtx.target[0].useRotate.set(0)
        ik_end_mtx.target[0].useShear.set(0)
        ik_end_mtx.target[0].weight.set(1)

        # Blend Matrices
        result_start_mtx = pm.createNode('blendMatrix', name=self.getName('result_start_mtx'))
        ik_start_mtx.matrixSum.connect(result_start_mtx.inputMatrix)
        fkAttrs[0].connect(result_start_mtx.target[0].targetMatrix)
        self.params.ikfk_blend.connect(result_start_mtx.target[0].weight)
        
        result_mid_mtx = pm.createNode('blendMatrix', name=self.getName('result_mid_mtx'))
        ik_mid_mtx.matrixSum.connect(result_mid_mtx.inputMatrix)
        fkAttrs[1].connect(result_mid_mtx.target[0].targetMatrix)
        self.params.ikfk_blend.connect(result_mid_mtx.target[0].weight)
        result_mid_mtx.outputMatrix.connect(self.result_mid.offsetParentMatrix)
        
        result_end_mtx = pm.createNode('blendMatrix', name=self.getName('result_end_mtx'))
        ik_end_mtx.outputMatrix.connect(result_end_mtx.inputMatrix)
        fkAttrs[2].connect(result_end_mtx.target[0].targetMatrix)
        self.params.ikfk_blend.connect(result_end_mtx.target[0].weight)
        result_end_mtx.outputMatrix.connect(self.result_end.offsetParentMatrix)

        # Additional matrices - (non-rolls, averages)
        axis = 'x'
        angle = (1, 0, 0)
        if self.invert:
            axis = '-x'
            #angle = (-1, 0, 0)
            
        startLocalMtx = mathOps.multiplyMatrices([result_start_mtx.outputMatrix, self.base_srt.worldInverseMatrix[0]],
                                                 name=self.getName('startMtx_to_baseMtx'))

        startVec = mathOps.createMatrixAxisVector(startLocalMtx.matrixSum, (1, 0, 0),
                                                  name=self.getName('result_start_vec'))

        start_angle = mathOps.angleBetween(angle, startVec.output, name=self.getName('start_angle'))
        start_angle_mtx = mathOps.createComposeMatrix(inputRotate=start_angle.euler,
                                                      name=self.getName('start_angle_mtx'))
        start_nonRoll_mtx = mathOps.multiplyMatrices([start_angle_mtx.outputMatrix, self.base_srt.worldMatrix[0]],
                                                     name=self.getName('start_nonRoll_mtx'))
        start_nonRoll_mtx.matrixSum.connect(self.result_start.offsetParentMatrix)

        startTip_mtx = transform.blend_T_R_matrices(result_mid_mtx.outputMatrix, result_start_mtx.outputMatrix,
                                                    name=self.getName('result_startTip_mtx'))
        startTip_mtx.outputMatrix.connect(self.result_startTip.offsetParentMatrix)
        
        startTipInverse = mathOps.inverseMatrix(startTip_mtx.outputMatrix, name=self.getName('startTipMtx_inverse'))
        midLocalMtx = mathOps.multiplyMatrices([self.result_mid.worldMatrix[0], startTipInverse.outputMatrix],
                                               name=self.getName('midMtx_to_startMtx'))
        d = mathOps.decomposeMatrix(midLocalMtx.matrixSum, name=self.getName('mid_local_mtx2Srt'))
        midRotX = mathOps.isolateRotationOnAxis(d.outputRotate, 'x', name=self.getName('mid'))
        midRotX[1].outputRotateX.connect(self.result_startTip.rx)

        startTwistMtx = mathOps.multiplyMatrices([start_nonRoll_mtx.matrixSum,
                                                  self.result_startTip.worldInverseMatrix[0]],
                                                 name=self.getName('start_twist_mtx'))
        startTwistMtx2Srt = mathOps.decomposeMatrix(startTwistMtx.matrixSum, name=self.getName('start_twist_mtx2Srt'))

        midTip_mtx = transform.blend_T_R_matrices(result_end_mtx.outputMatrix, result_mid_mtx.outputMatrix,
                                                  name=self.getName('result_midTip_mtx'))
        midTip_mtx.outputMatrix.connect(self.result_midTip.offsetParentMatrix)
        midTipInverse = mathOps.inverseMatrix(midTip_mtx.outputMatrix, name=self.getName('midTipMtx_inverse'))
        endLocalMtx = mathOps.multiplyMatrices([self.result_end.worldMatrix[0], midTipInverse.outputMatrix],
                                               name=self.getName('endMtx_to_midMtx'))
        d = mathOps.decomposeMatrix(endLocalMtx.matrixSum, name=self.getName('end_local_mtx2Srt'))
        rotX = mathOps.isolateRotationOnAxis(d.outputRotate, 'x', name=self.getName('end'))
        rotX[1].outputRotateX.connect(self.result_midTip.rx)

        startAvgMtx = transform.blendMatrices(self.base_srt.worldMatrix[0], self.result_start.worldMatrix[0],
                                              name=self.getName('result_start_avg_mtx'))
        startAvgMtx.outputMatrix.connect(self.result_start_avg.offsetParentMatrix)

        midAvgMtx = transform.blendMatrices(self.result_startTip.worldMatrix[0], result_mid_mtx.outputMatrix,
                                            name=self.getName('result_mid_avg_mtx'))
        midAvgPickMtx = pm.createNode('pickMatrix', name=self.getName('result_mid_avg_noScale_mtx'))
        midAvgMtx.outputMatrix.connect(midAvgPickMtx.inputMatrix)
        midAvgPickMtx.useScale.set(0)
        chamferMtx = transform.blendMatrices(result_start_mtx.outputMatrix, result_end_mtx.outputMatrix,
                                             name=self.getName('mid_chamfer_mtx'))
        upperRatioClamp.outputR.connect(chamferMtx.target[0].weight)
        midCtrlMtx = transform.blendMatrices(midAvgMtx.outputMatrix, chamferMtx.outputMatrix,
                                             name=self.getName('result_mid_ctrl_mtx'))
        midCtrlMtx.target[0].useRotate.set(0)
        midCtrlMtx.target[0].useShear.set(0)
        midCtrlMtx.target[0].useScale.set(0)
        self.params.chamfer.connect(midCtrlMtx.target[0].weight)
        midTwistMtx = mathOps.createComposeMatrix(name=self.getName('mid_twist_mtx'))
        midUpperTwist = mathOps.remap(self.params.mid_twist, -1, 0, 1, 0, name=self.getName('mid_upper_twist'))
        midLowerTwist = mathOps.remap(self.params.mid_twist, 0, 1, 0, 1, name=self.getName('mid_lower_twist'))
        lowerTwistMult = mathOps.multiplyAngleByScalar(rotX[1].outputRotateX, midLowerTwist.outValueX,
                                                       name=self.getName('lower_twist_mult'))
        upperTwistMult = mathOps.multiplyAngleByScalar(startTwistMtx2Srt.outputRotateX, midUpperTwist.outValueX,
                                                       name=self.getName('upper_twist_mult'))
        midTwistSum = mathOps.addAngles(lowerTwistMult.output, upperTwistMult.output,
                                        name=self.getName('mid_twist_sum'))
        midTwistSum.output.connect(midTwistMtx.inputRotateX)
        midCtrlTwistMtx = mathOps.multiplyMatrices([midTwistMtx.outputMatrix, midCtrlMtx.outputMatrix],
                                                   name=self.getName('mid_ctrl_twist_mtx'))
        negScaleMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('negScale_mtx'))
        if self.invert:
            midCtrlNegScaleMtx = mathOps.createInverseHandedMatrix(midCtrlTwistMtx.matrixSum,
                                                                   composeMtx=negScaleMtx,
                                                                   name=self.getName('midCtrl'))
            midCtrlNegScaleMtx.matrixSum.connect(self.mid_ctrl.offsetParentMatrix)
            midAvgNegScaleMtx = mathOps.createInverseHandedMatrix(self.mid_ctrl.worldMatrix[0],
                                                                  composeMtx=negScaleMtx,
                                                                  name=self.getName('midAvg'))
            midAvgNegScaleMtx.matrixSum.connect(self.result_mid_avg.offsetParentMatrix)
        else:
            midCtrlTwistMtx.matrixSum.connect(self.mid_ctrl.offsetParentMatrix)
            self.mid_ctrl.worldMatrix[0].connect(self.result_mid_avg.offsetParentMatrix)

        endAvgMtx = transform.blendMatrices(self.result_midTip.worldMatrix[0], result_end_mtx.outputMatrix,
                                            name=self.getName('result_end_avg_mtx'))
        endAvgMtx.outputMatrix.connect(self.result_end_avg.offsetParentMatrix)

        # ---------------------------------
        # Twist Segment systems
        # ---------------------------------
        # Upper
        dm = mathOps.decomposeMatrix(result_start_mtx.outputMatrix, name=self.getName('result_start_mtx2Srt'))
        dm.outputTranslate.connect(self.upper_crv.controlPoints[0])
        dm = mathOps.decomposeMatrix(self.mid_ctrl.worldMatrix[0], name=self.getName('mid_ctrl_mtx2Srt'))
        dm.outputTranslate.connect(self.upper_crv.controlPoints[3])

        upperDist = mathOps.distance(self.result_start, self.mid_ctrl, name=self.getName('result_upper_dist'))
        upperDistScaled = mathOps.divide(upperDist.distance, self.base_srt.sx,
                                         name=self.getName('result_upperDist_scaled'))
        upperDistInverse = mathOps.multiply(upperDistScaled.outputX, -1, name=self.getName('upper_dist_inverse'))

        # Upper start tangent point
        startLocalMtx = pm.createNode('composeMatrix', name=self.getName('upper_start_tangent_local_mtx'))
        upperStartTangentLen = pm.createNode('animBlendNodeAdditive', name=self.getName('upper_start_tangent_len'))
        upperStartTangentLen.output.connect(startLocalMtx.inputTranslateX)
        self.params.start_tangent.connect(upperStartTangentLen.weightA)
        upperDistScaled.outputX.connect(upperStartTangentLen.inputA)
        startTargMtx = self.base_srt.worldMatrix[0]
        if self.invert:
            startTargMtx = mathOps.createInverseHandedMatrix(startTargMtx,
                                                             composeMtx=negScaleMtx,
                                                             name=self.getName('base_neg_mtx')).matrixSum
        startAimMtx = transform.createAimMatrix(startTargMtx, self.mid_ctrl.worldMatrix[0],
                                                name=self.getName('upper_start_aim_mtx'))
        startBlendMtx = transform.blendMatrices(startTargMtx, startAimMtx.outputMatrix,
                                                name=self.getName('upper_start_blend_mtx'))
        self.params.start_follow.connect(startBlendMtx.target[0].weight)

        upperTwistMtx = mathOps.createComposeMatrix(name=self.getName('start_twist_mtx'))
        upperTwistMult = mathOps.multiplyAngleByScalar(rotX[1].outputRotateX, self.params.upper_twist,
                                                       name=self.getName('upper_twist_mult'))
        upperTwistMult.output.connect(upperTwistMtx.inputRotateX)

        startTangentMtx = mathOps.multiplyMatrices([upperTwistMtx.outputMatrix,
                                                    startLocalMtx.outputMatrix, startBlendMtx.outputMatrix],
                                                   name=self.getName('upper_start_tangent_mtx'))
        startTangentMtx.matrixSum.connect(self.upper_startBend_ctrl.offsetParentMatrix)
        dm = mathOps.decomposeMatrix(self.upper_startBend_ctrl.worldMatrix[0],
                                     name=self.getName('upper_start_tangent_mtx2Srt'))
        dm.outputTranslate.connect(self.upper_crv.controlPoints[1])
        
        # upper end tangent point
        upperEndLocalMtx = pm.createNode('composeMatrix', name=self.getName('upper_end_tangent_local_mtx'))
        upperEndTangentLen = pm.createNode('animBlendNodeAdditive', name=self.getName('upper_end_tangent_len'))
        upperEndTangentLen.output.connect(upperEndLocalMtx.inputTranslateX)
        self.params.round_radius.connect(upperEndTangentLen.weightA)
        upperDistInverse.output.connect(upperEndTangentLen.inputA)

        endAimMtx = transform.createAimMatrix(self.mid_ctrl.worldMatrix[0], self.base_srt.worldMatrix[0],
                                                name=self.getName('upper_end_aim_mtx'))
        endAimMtx.primaryInputAxis.set(-1, 0, 0)
        endBlendMtx = transform.blendMatrices(endAimMtx.outputMatrix, self.mid_ctrl.worldMatrix[0],
                                                name=self.getName('upper_end_blend_mtx'))
        self.params.roundness.connect(endBlendMtx.target[0].weight)
        endTangentMtx = mathOps.multiplyMatrices([upperEndLocalMtx.outputMatrix, endBlendMtx.outputMatrix],
                                                 name=self.getName('upper_end_tangent_mtx'))
        endTangentMtx.matrixSum.connect(self.upper_endBend_ctrl.offsetParentMatrix)
        dm = mathOps.decomposeMatrix(self.upper_endBend_ctrl.worldMatrix[0],
                                     name=self.getName('upper_end_tangent_mtx2Srt'))
        dm.outputTranslate.connect(self.upper_crv.controlPoints[2])
        
        # LOWER
        dm = mathOps.decomposeMatrix(self.mid_ctrl.worldMatrix[0])
        dm.outputTranslate.connect(self.lower_crv.controlPoints[0])
        dm = mathOps.decomposeMatrix(result_end_mtx.outputMatrix, name=self.getName('result_end_mtx2Srt'))
        dm.outputTranslate.connect(self.lower_crv.controlPoints[3])

        # lower start tangent point
        lowerDist = mathOps.distance(self.mid_ctrl, self.result_end, name=self.getName('result_lower_dist'))
        lowerDistScaled = mathOps.divide(lowerDist.distance, self.base_srt.sx,
                                         name=self.getName('result_lowerDist_scaled'))
        lowerDistInverse = mathOps.multiply(lowerDistScaled.outputX, -1, name=self.getName('lower_dist_inverse'))

        lowerEndLocalMtx = pm.createNode('composeMatrix', name=self.getName('lower_end_tangent_local_mtx'))
        lowerEndTangentLen = pm.createNode('animBlendNodeAdditive', name=self.getName('lower_end_tangent_len'))
        lowerEndTangentLen.output.connect(lowerEndLocalMtx.inputTranslateX)
        self.params.round_radius.connect(lowerEndTangentLen.weightA)
        lowerDistScaled.outputX.connect(lowerEndTangentLen.inputA)

        lowerStartAimMtx = transform.createAimMatrix(self.mid_ctrl.worldMatrix[0], result_end_mtx.outputMatrix,
                                                     name=self.getName('lower_start_aim_mtx'))
        lowerStartBlendMtx = transform.blendMatrices(lowerStartAimMtx.outputMatrix, self.mid_ctrl.worldMatrix[0],
                                                     name=self.getName('lower_start_blend_mtx'))
        self.params.roundness.connect(lowerStartBlendMtx.target[0].weight)
        lowerStartTangentMtx = mathOps.multiplyMatrices([lowerEndLocalMtx.outputMatrix,
                                                         lowerStartBlendMtx.outputMatrix],
                                                        name=self.getName('lower_start_tangent_mtx'))
        lowerStartTangentMtx.matrixSum.connect(self.lower_startBend_ctrl.offsetParentMatrix)
        dm = mathOps.decomposeMatrix(self.lower_startBend_ctrl.worldMatrix[0],
                                     name=self.getName('lower_start_tangent_mtx2Srt'))
        dm.outputTranslate.connect(self.lower_crv.controlPoints[1])

        # Lower end tangent point
        lowerEndLocalMtx = pm.createNode('composeMatrix', name=self.getName('lower_end_tangent_local_mtx'))
        lowerEndTangentLen = pm.createNode('animBlendNodeAdditive', name=self.getName('lower_end_tangent_len'))
        lowerEndTangentLen.output.connect(lowerEndLocalMtx.inputTranslateX)
        self.params.end_tangent.connect(lowerEndTangentLen.weightA)
        lowerDistInverse.output.connect(lowerEndTangentLen.inputA)
        endTargMtx = self.result_end.worldMatrix[0]
        if self.invert:
            endTargMtx = mathOps.createInverseHandedMatrix(endTargMtx,
                                                           composeMtx=negScaleMtx,
                                                           name=self.getName('end_neg_mtx')).matrixSum
        lowerEndAimMtx = transform.createAimMatrix(endTargMtx, self.mid_ctrl.worldMatrix[0],
                                                   name=self.getName('lower_end_aim_mtx'))
        lowerEndAimMtx.primaryInputAxis.set((-1, 0, 0))
        lowerEndBlendMtx = transform.blendMatrices(endTargMtx, lowerEndAimMtx.outputMatrix,
                                                   name=self.getName('lower_end_blend_mtx'))
        self.params.end_follow.connect(lowerEndBlendMtx.target[0].weight)

        lowerTwistSum = mathOps.addAngles(startTwistMtx2Srt.outputRotateX, rotX[1].outputRotateX,
                                          name=self.getName('lower_twist_sum'))
        lowerTwistSum.weightA.set(-1)
        lowerTwistMult = mathOps.multiplyAngleByScalar(lowerTwistSum.output, self.params.lower_twist,
                                                       name=self.getName('lower_twist_mult'))
        lowerTwistMtx = mathOps.createComposeMatrix(name=self.getName('lower_twist_mtx'))
        lowerTwistMult.output.connect(lowerTwistMtx.inputRotateX)

        lowerEndTangentMtx = mathOps.multiplyMatrices([lowerTwistMtx.outputMatrix,
                                                       lowerEndLocalMtx.outputMatrix, lowerEndBlendMtx.outputMatrix],
                                                      name=self.getName('lower_end_tangent_mtx'))

        lowerEndTangentMtx.matrixSum.connect(self.lower_endBend_ctrl.offsetParentMatrix)
        dm = mathOps.decomposeMatrix(self.lower_endBend_ctrl.worldMatrix[0],
                                     name=self.getName('lower_end_tangent_mtx2Srt'))
        dm.outputTranslate.connect(self.lower_crv.controlPoints[2])

        # -------------------------------------------
        # SLEEVES
        # -------------------------------------------
        if self.guide.sleeve in [1, 2, 4]:
            for index, mp in enumerate(self.upperSleeve_mps):
                num = str(index+1).zfill(2)
                mtx = pm.listConnections(mp.worldUpMatrix)[0]
                param = mp.uValue.get()
                try:
                    pullAttr = self.params.sleeve_pull
                except:
                    pullAttr = self.params.upper_sleeve_pull
                pullReverse = mathOps.reverse(pullAttr, name=self.getName('sleeve_pull_reverse'))
                twistMult = mathOps.multiply(param, pullReverse.outputX,
                                             name=self.getName('upperSleeve_%s_twist_mult' % num))
                if self.guide.sleeve != 2:
                    paramMult = mathOps.multiply(param, pullReverse.outputX,
                                                 name=self.getName('upperSleeve_%s_param_mult' % num))
                    paramMult.output.connect(mp.uValue)
                twistResult = mathOps.multiply(self.params.sleeve_twist, twistMult.output,
                                                   name=self.getName('upperSleeve_%s_twist_mult' % num))
                twistResult.output.connect(mtx.target[0].weight)

        if self.guide.sleeve == 2:
            result_start_mtx.outputMatrix.connect(self.lowerSleeveBaseMtx.target[0].targetMatrix, f=1)
            self.lowerSleeveBaseMtx.target[0].useTranslate.set(0)
            pullReverse = mathOps.reverse(self.params.sleeve_pull, name=self.getName('sleeve_pull_reverse'))

            sleeveLocalMtx = mathOps.multiplyMatrices([result_mid_mtx.outputMatrix,
                                                      self.upperSleeveDivs[-1].worldInverseMatrix[0]],
                                                      name=self.getName('lower_sleeve_local_mtx'))
            tempVec = (1, 0, 0)
            if self.invert:
                tempVec = (-1, 0, 0)
            sleeveVec = mathOps.createMatrixAxisVector(sleeveLocalMtx.matrixSum, tempVec,
                                                       name=self.getName('lower_sleeve_start_vec'))
            sleeve_angle = mathOps.angleBetween(angle, sleeveVec.output, name=self.getName('lower_sleeve_angle'))
            sleeve_angle_mtx = mathOps.createComposeMatrix(inputRotate=sleeve_angle.euler,
                                                           name=self.getName('lower_sleeve_angle_mtx'))
            sleeve_nonRoll_mtx = mathOps.multiplyMatrices([sleeve_angle_mtx.outputMatrix,
                                                           self.upperSleeveDivs[-1].worldMatrix[0]],
                                                          name=self.getName('lower_sleeve_nonRoll_mtx'))

            for index, mp in enumerate(self.lowerSleeve_mps):
                num = str(index+1).zfill(2)
                mtx = pm.listConnections(mp.worldUpMatrix)[0]
                sleeve_nonRoll_mtx.matrixSum.connect(mtx.inputMatrix, f=1)
                param = mp.uValue.get()

                paramMult = mathOps.multiply(param, pullReverse.outputX,
                                             name=self.getName('lowerSleeve_%s_param_mult' % num))
                paramMult.output.connect(mp.uValue)
                twistMult = mathOps.multiply(param, self.params.sleeve_pull,
                                             name=self.getName('lowerSleeve_%s_twist_mult' % num))
                twistResult = mathOps.multiply(self.params.sleeve_twist, twistMult.output,
                                               name=self.getName('lowerSleeve_%s_twist_mult' % num))
                twistResult.output.connect(mtx.target[0].weight)

            sleeveMidMtx = transform.blendMatrices(self.upperSleeveDivs[-1].worldMatrix[0],
                                                   self.lowerSleeveDivs[0].worldMatrix[0],
                                                   name=self.getName('sleeve_mid_avg_mtx'))
            sleeveMidMtx.outputMatrix.connect(self.sleeve_mid_avg.offsetParentMatrix)
        elif self.guide.sleeve in [3, 4]:
            pullReverse = mathOps.reverse(self.params.lower_sleeve_pull, name=self.getName('lower_sleeve_pull_reverse'))
            for index, mp in enumerate(self.lowerSleeve_mps):
                num = str(index+1).zfill(2)
                mtx = pm.listConnections(mp.worldUpMatrix)[0]
                param = 1.0 - mp.uValue.get()
                paramBlend = mathOps.remap(self.params.lower_sleeve_pull, 0, 1, param, 1,
                                           name=self.getName('lowerSleeve_%s_param_blend' % num))
                paramBlend.outValueX.connect(mp.uValue)
                paramReverse = mathOps.reverse(paramBlend.outValueX,
                                               name=self.getName('lowerSleeve_%s_param_reverse' % num))
                twistMult = mathOps.multiply(pullReverse.outputX, paramReverse.outputX,
                                             name=self.getName('lowerSleeve_%s_twist_mult' % num))
                twistResult = mathOps.multiply(self.params.sleeve_twist, twistMult.output,
                                               name=self.getName('lowerSleeve_%s_twist_mult' % num))
                twistResult.output.connect(mtx.target[0].weight)

        # -------------------------------------------
        # Squash n Stretch
        # -------------------------------------------
        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
        upperCrvLen = curve.createCurveLength(self.upper_crv, name=self.getName('upper_crv_info'))
        upperCrvLenScaled = mathOps.multiply(baseMtx2Srt.outputScaleX, upperCrvLen.arcLength.get(),
                                             name=self.getName('upper_crv_len_scaled'))
        upperStretch = mathOps.divide(upperCrvLenScaled.output, upperCrvLen.arcLength,
                                      name=self.getName('upper_stretch_mult'))
        
        lowerCrvLen = curve.createCurveLength(self.lower_crv, name=self.getName('lower_crv_info'))
        lowerCrvLenScaled = mathOps.multiply(baseMtx2Srt.outputScaleX, lowerCrvLen.arcLength.get(),
                                             name=self.getName('lower_crv_len_scaled'))
        lowerStretch = mathOps.divide(lowerCrvLenScaled.output, lowerCrvLen.arcLength,
                                      name=self.getName('lower_stretch_mult'))

        stretchSum = mathOps.blendScalarAttrs(upperStretch.outputX, lowerStretch.outputX, 0.5,
                                              name=self.getName('stretch_mult'))
        stretchScale = mathOps.multiply(stretchSum.output, baseMtx2Srt.outputScaleX, name=self.getName('stretch_scaled'))
        stretchBlend = mathOps.blendScalarAttrs(self.base_srt.sx, stretchSum.output, self.params.volume_preserve,
                                                name=self.getName('mid_avg_volume_mult'))

        def _setupVolume(node, name, upper=1, param=None):
            if not param:
                param = pm.listConnections(node, type='motionPath')[0].uValue.get()
            if upper:
                param = 1.0 - param
            volumeRemap = mathOps.remap(param, 0, 1, self.params.volume_falloff, 0,
                                        name=self.getName('%s_volume_falloff' % name))
            volumeEase = anim.easeCurve(input=volumeRemap.outValueX,
                                        name=self.getName('%s_volume_ease' % name))
            volumeMult = mathOps.blendScalarAttrs(baseMtx2Srt.outputScaleX, stretchScale.output, self.params.volume_preserve,
                                          name=self.getName('%s_volume_mult' % name))
            volumeAmount = mathOps.blendScalarAttrs(baseMtx2Srt.outputScaleX, volumeMult.output, volumeEase.output,
                                                    name=self.getName('%s_volume_amount' % name))

            volumeAmount.output.connect(node.sy)
            volumeAmount.output.connect(node.sz)
            baseMtx2Srt.outputScaleX.connect(node.sx)

        for node in self.upperDivs:
            _setupVolume(node, 'upper')
        for node in self.lowerDivs:
            _setupVolume(node, 'lower', upper=0)
        stretchBlend.output.connect(self.result_mid_avg.sy)
        stretchBlend.output.connect(self.result_mid_avg.sz)

        for index, node in enumerate(self.upperSleeveDivs):
            conn = pm.listConnections(self.upperDivs[index].sy, plugs=1)[0]
            conn.connect(node.sy)
            conn.connect(node.sz)
            baseMtx2Srt.outputScaleX.connect(node.sx)

        for index, node in enumerate(self.lowerSleeveDivs):
            conn = pm.listConnections(self.lowerDivs[index].sy, plugs=1)[0]
            conn.connect(node.sy)
            conn.connect(node.sz)
            baseMtx2Srt.outputScaleX.connect(node.sx)

        # ---------------------------------
        # Internal spaces switching setup
        # ---------------------------------
        self.spaces['%s' % (self.pole_ctrl.name())] = 'limb_average: %s.matrixSum' % base_mtx.name()

    def finish(self):
        # --------------------------------------------------
        # Proxy animation attrs on to relevant controls
        # --------------------------------------------------
        attrList = [self.params.mid_slide, self.params.pin_to_pole, self.params.softness, self.params.stretch]
        for attr in attrList:
            attribute.proxyAttribute(attr, self.ik_ctrl)

        spaceAttrs = [attr for attr in ['ik_ctrl_parent_space', 'ik_ctrl_translate_space', 'ik_ctrl_rotate_space']
                      if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.ik_ctrl)

        spaceAttrs = [attr for attr in ['ik_pole_ctrl_parent_space', 'ik_pole_ctrl_translate_space',
                                        'ik_pole_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.pole_ctrl)

        attrList = [self.params.ikfk_blend, self.params.volume_preserve, self.params.volume_falloff,
                    self.params.roundness, self.params.round_radius, self.params.start_tangent,
                    self.params.start_follow, self.params.end_tangent, self.params.end_follow, self.params.chamfer,
                    self.params.upper_twist, self.params.lower_twist, self.params.mid_twist]
        if not self.guide.sleeve == 0:
            attrList.append(self.params.sleeve_twist)
            attrList.append(self.params.sleeve_pull)

        for attr in attrList:
            attribute.proxyAttribute(attr, self.mid_ctrl)

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodeList = self.controls_list
        attrList = ['sx', 'sy', 'sz', 'visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = [node for node in self.controls_list if not node == self.pole_ctrl]
        attribute.channelControl(nodeList=nodeList, attrList=['rotateOrder'], keyable=1, lock=0)

        nodeList = [self.upper_startBend_ctrl, self.lower_endBend_ctrl]
        attrList = ['ry', 'rz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = [self.upper_endBend_ctrl, self.lower_startBend_ctrl]
        attrList = ['ry', 'rz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['tx', 'ty', 'tz']
        attribute.channelControl(nodeList=[self.fk_start_ctrl], attrList=attrList)

        self.setColours(self.guide)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TArmIkFk(guide)

