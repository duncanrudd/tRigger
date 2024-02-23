from tRigger import components
from tRigger.core import attribute, transform, dag, mathOps, icon, curve, anim
import pymel.core as pm
import math
reload(components)
reload(transform)
reload(mathOps)
reload(attribute)
reload(curve)

import pymel.core as pm

#[(root_guide'),(02_guide'), (04_guide'), (03_guide'), (05_guide'), (01_guide')]

class TLimbIkFk(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'limbIkFk')
        print 'Created limbIkFk Component: %s' % self.comp_name

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
        flip = aimVec[0] < 0
        if self.invert:
            flip = aimVec[0] > 0

        sideVec = upVec.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        if self.invert:
            sideVec = aimVec.cross(upVec).normal()
            upVec = sideVec.cross(aimVec).normal()
        startPos = pm.xform(guide.locs[1], q=1, ws=1, t=1)

        if flip and not self.reversePole:
            sideVec *= -1
            upVec *= -1

        midXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)

        # Build end matrix
        endXform = guide.locs[3].worldMatrix[0].get()
        if self.invert:
            endXform = mathOps.invertHandedness(endXform)

        # FK controls
        self.fk_start_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.3,
                                          name=self.getName('fk_start'), xform=startXform,
                                          parent=self.base_srt, metaParent=self.base_srt, buffer=1)
        if self.invert:
            self.fk_start_ctrl.getParent().sx.set(-1)
        self.fk_mid_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.3,
                                        name=self.getName('fk_mid'), xform=midXform, parent=self.fk_start_ctrl,
                                        metaParent=self.fk_start_ctrl)
        self.fk_end_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.3,
                                        name=self.getName('fk_end'), xform=endXform, parent=self.fk_mid_ctrl,
                                        metaParent=self.fk_mid_ctrl)

        # --------------------------------
        # IK
        # --------------------------------
        # IK ctrl
        ik_ctrl_mtx = None
        startPos = pm.xform(guide.locs[3], q=1, ws=1, t=1)
        ik_ctrl_mtx = transform.getMatrixFromPos(startPos)
        self.ik_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize * .3,
                                    name=self.getName('ik'), xform=ik_ctrl_mtx,
                                    parent=self.controls, metaParent=self.base_srt)
        aimVec = _getVec(guide.locs[2], guide.locs[3], self.invert)
        if aimVec[0] >= math.fabs(aimVec[1]):
            self.twist = 'x'
        else:
            self.twist = 'y'
        if not guide.root.world_aligned_ik_ctrl.get():
            upVecTemp = mathOps.getMatrixAxisAsVector(guide.locs[3].worldMatrix[0].get(), 'z')
            sideVec = upVecTemp.cross(aimVec).normal()
            upVec = aimVec.cross(sideVec).normal()
            if self.twist == 'x':
                ik_offset_mtx = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
            else:
                if self.invert:
                    ik_offset_mtx = pm.datatypes.Matrix(-sideVec, aimVec, upVec, startPos)
                else:
                    ik_offset_mtx = pm.datatypes.Matrix(sideVec, -aimVec, upVec, startPos)
            mtx = ik_offset_mtx*self.ik_ctrl.worldInverseMatrix[0].get()
            rot = pm.datatypes.EulerRotation()
            rVal = pm.datatypes.degrees(rot.decompose(mtx, 'XYZ'))
            self.ik_ctrl.r.set(rVal)
        self.ik_displaced = dag.addChild(self.rig, 'group', name=self.getName('ik_displaced_srt'))
        self.ik_ctrl.worldMatrix[0].connect(self.ik_displaced.offsetParentMatrix)
        transform.align(self.ik_displaced, self.base_srt, translate=0)

        # Pole Vector ctrl
        pole_ctrl_mtx = transform.getMatrixFromPos(pm.xform(guide.locs[4], q=1, ws=1, t=1))
        self.pole_ctrl = self.addCtrl(shape='ball', size=ctrlSize*.05,
                                      name=self.getName('ik_pole'), xform=pole_ctrl_mtx, parent=self.controls,
                                      metaParent=self.ik_ctrl)
        self.pole_guide = curve.curveBetweenPoints((0, 0, 0), (1, 0, 0), numPoints=2,
                                                   name=self.getName('pole_guide_crv'), degree=1)
        self.pole_guide.setParent(self.controls)

        # Pole spin srt
        self.pole_follow_srt = dag.addChild(self.rig, 'group', name=self.getName('ik_pole_follow_srt'))

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

        # Build end matrix
        aimVec = _getVec(guide.locs[2], guide.locs[3], self.invert)
        upVecTemp = mathOps.getMatrixAxisAsVector(guide.locs[3].worldMatrix[0].get(), 'z')
        sideVec = upVecTemp.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(self.fk_end_ctrl, q=1, ws=1, t=1)
        endXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        self.result_end = dag.addChild(self.rig, 'group', name=self.getName('result_end_srt'))
        self.result_end.offsetParentMatrix.set(endXform)
        self.result_tip = dag.addChild(self.result_end, 'group', name=self.getName('result_tip_srt'))
        self.result_tip.tx.set(ctrlSize*.2)
        if self.invert:
            self.result_tip.tx.set(ctrlSize*-.2)

        # Mid Ctrl
        m = transform.getBlendedMatrix([self.fk_start_ctrl, self.fk_mid_ctrl])
        midCtrl_xform = pm.datatypes.Matrix(m[0], m[1], m[2], list(midXform.translate.get()) + [0.0])
        self.mid_ctrl = self.addCtrl(shape='ball', size=ctrlSize*.1,
                                     name=self.getName('mid'),
                                     xform=midCtrl_xform,
                                     parent=self.controls, metaParent=self.base_srt)

        # BendyControls
        self.upper_bend_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.15,
                                            name=self.getName('upper_bend'),
                                            xform=startXform, parent=self.controls, metaParent=self.mid_ctrl)
        self.lower_bend_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.15,
                                            name=self.getName('lower_startBend'),
                                            xform=midXform, parent=self.controls, metaParent=self.mid_ctrl)

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
        # Add mapping connection to control - used when mapping controller tags
        self.mapToControl(self.mid_ctrl, self.result_end)

        # Divs and Joints
        self.upperDivs = []
        self.lowerDivs = []
        for i in range(guide.num_divisions):
            num = str(i+1).zfill(2)
            self.upperDivs.append(dag.addChild(self.rig, 'group', name=self.getName('upperDiv_%s_srt' % num)))
            self.lowerDivs.append(dag.addChild(self.rig, 'group', name=self.getName('lowerDiv_%s_srt' % num)))

        # Joints
        if self.guide.add_joint:
            j = pm.createNode('joint', name=self.getName('start_avg_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.result_start_avg})
            parent = j

            for i, div in enumerate(self.upperDivs):
                j = pm.createNode('joint', name=div.name().replace('srt', 'jnt'))
                j.setParent(parent)
                self.joints_list.append({'joint': j, 'driver': div})
                parent = j

            j = pm.createNode('joint', name=self.getName('mid_avg_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.result_mid_avg})
            j.setParent(parent)
            parent = j

            for i, div in enumerate(self.lowerDivs):
                j = pm.createNode('joint', name=div.name().replace('srt', 'jnt'))
                j.setParent(parent)
                self.joints_list.append({'joint': j, 'driver': div})
                parent = j

            j = pm.createNode('joint', name=self.getName('end_avg_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.result_end_avg})
            j.setParent(parent)
            parent = j

            self.mapJointToGuideLocs(j, self.guide.locs[3])

            j = pm.createNode('joint', name=self.getName('end_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.result_tip})
            j.setParent(parent)

            self.mapJointToGuideLocs(j, self.guide.locs[4])

        # Call overloaded method of parent class
        components.TBaseComponent.addObjects(self, guide)

    def addAttributes(self):
        attribute.addDividerAttr(self.params, 'GENERAL')
        attribute.addFloatAttr(self.params, 'mid_separation', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'ikfk_blend', minValue=0, maxValue=1)

        attribute.addDividerAttr(self.params, 'IK')
        attribute.addFloatAttr(self.params, 'stretch', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'softness', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'length_mult', minValue=0, value=1)
        attribute.addFloatAttr(self.params, 'mid_slide', minValue=-1, maxValue=1)
        attribute.addFloatAttr(self.params, 'pole_follows_ik', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'pin_to_pole', minValue=0, maxValue=1)

        attribute.addDividerAttr(self.params, 'BENDY')
        attribute.addBoolAttr(self.params, 'show_bendy_ctrls')
        pm.setAttr(self.params.show_bendy_ctrls, k=0, cb=1)

    def addSystems(self):
        # --------------------------------
        # IK
        # --------------------------------
        # Non roll base - starting point for basis and stable parent space for pole ctrl
        axis = 'x'
        angle=(1, 0, 0)
        revAngle=(-1, 0, 0)
        if self.invert:
            axis = '-x'
            angle=(-1, 0, 0)
            revAngle=(1, 0, 0)
        d = mathOps.decomposeMatrix(self.ik_displaced.worldMatrix[0], name=self.getName('ik_displaced_mtx2Srt'))
        targToBaseVec = mathOps.createTransformedPoint(d.outputTranslate, self.base_srt.worldInverseMatrix[0],
                                                       name=self.getName('ik_targ_to_base_vec'))
        base_angle = mathOps.angleBetween(angle, targToBaseVec.output, name=self.getName('ik_base_angle'))
        base_angle_mtx = mathOps.createComposeMatrix(inputRotate=base_angle.euler,
                                                     name=self.getName('ik_base_angle_mtx'))

        base_mtx = mathOps.multiplyMatrices([base_angle_mtx.outputMatrix, self.base_srt.worldMatrix[0]],
                                            name=self.getName('ik_base_mtx'))
        base_mtx.matrixSum.connect(self.pole_follow_srt.offsetParentMatrix)
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

        baseInverse = mathOps.inverseMatrix(base_mtx.matrixSum, name=self.getName('base_mtx_inverse'))
        targLocalMtx = mathOps.multiplyMatrices([self.ik_displaced.worldMatrix[0], baseInverse.outputMatrix],
                                                name=self.getName('ik_targ_local_mtx'))
        localMtx2Srt = mathOps.decomposeMatrix(targLocalMtx.matrixSum, name=self.getName('ik_targ_local_mtx2Srt'))
        poleSpin = mathOps.isolateRotationOnAxis(localMtx2Srt.outputRotate, 'x', name=self.getName('ik_pole_spin'))
        #spinAttr = pm.general.Attribute('%s.outputRotate%s' % (poleSpin[1].name(), self.twist.upper()))
        poleSpinOffset = mathOps.addAngles(poleSpin[1].outputRotateX, poleSpin[1].outputRotateX.get() * -1,
                                           name=self.getName('ik_pole_spin_offset'))
        driver = self.params.pole_follows_ik
        driver.connect(poleSpinOffset.weightA)
        driver.connect(poleSpinOffset.weightB)
        poleSpinOffset.output.connect(self.pole_follow_srt.rx)

        # --IK SOLVER--
        #   Get main distances and ratios

        targetDist = mathOps.distance(self.base_srt, self.ik_displaced, name=self.getName('ik_target_dist'))
        targetDistScaled = mathOps.divide(targetDist.distance, d.outputScaleX,
                                          name=self.getName('ik_target_dist_normal'))

        chainLen = mathOps.addScalar([mathOps.getDistance(self.fk_start_ctrl, self.fk_mid_ctrl),
                                      mathOps.getDistance(self.fk_mid_ctrl, self.fk_end_ctrl)],
                                     name=self.getName('ik_chain_len'))
        extendLen = mathOps.multiply(chainLen.output1D, self.params.length_mult, name=self.getName('ik_extend_len'))

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
        exprString += 'float $targLen = %s.output;\n' % lowerPinStretchBlend.name()
        exprString += 'float $minLen = abs($upLen - $lowLen)*1.01;\n'
        exprString += 'if ($targLen < $minLen){\n\t'
        exprString += '$targLen = $minLen;\n}\n\n'
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

        def _offsetMtx(source, dest, name, index):
            offset = dest.worldMatrix[0].get() * source.worldInverseMatrix[0].get()
            offset_mtx = mathOps.vectors2Mtx44(offset[0], offset[1], offset[2], offset[3],
                                               name=self.getName('fk_%s_offset_mtx' % name))
            mtx = mathOps.multiplyMatrices([offset_mtx.output, source.worldMatrix[0]],
                                           name=self.getName('fk_%s_mtx') % name)
            fkAttrs[index] = mtx.matrixSum
        if self.invert:
            _offsetMtx(self.fk_start_ctrl, self.result_start, 'start', 0)
            _offsetMtx(self.fk_mid_ctrl, self.result_mid, 'mid', 1)
            _offsetMtx(self.fk_end_ctrl, self.result_end, 'end', 2)
        elif self.guide.root.world_aligned_ik_ctrl.get():
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
                                                  name=self.getName('local_start_vec'))

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
        mid_dm = mathOps.decomposeMatrix(midLocalMtx.matrixSum, name=self.getName('mid_local_mtx2Srt'))
        midRotX = mathOps.isolateRotationOnAxis(mid_dm.outputRotate, 'x', name=self.getName('mid'))
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
        mtxAttr = midAvgMtx.outputMatrix
        avgAttr = self.mid_ctrl.worldMatrix[0]
        
        if self.invert:
            self.negScaleMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('negScale_mtx'))
            midCtrlNegScaleMtx = mathOps.multiplyMatrices([self.negScaleMtx.outputMatrix, mtxAttr],
                                                          name=self.getName('midCtrl_negScale_mtx'))
            mtxAttr = midCtrlNegScaleMtx.matrixSum

            midAvgNegScaleMtx = mathOps.multiplyMatrices([self.negScaleMtx.outputMatrix, avgAttr],
                                                          name=self.getName('midCtrl_negScale_mtx'))
            avgAttr = midAvgNegScaleMtx.matrixSum

        mtxAttr.connect(self.mid_ctrl.offsetParentMatrix)
        avgAttr.connect(self.result_mid_avg.offsetParentMatrix)

        endAvgMtx = transform.blendMatrices(self.result_midTip.worldMatrix[0], result_end_mtx.outputMatrix,
                                            name=self.getName('result_end_avg_mtx'))
        endAvgMtx.outputMatrix.connect(self.result_end_avg.offsetParentMatrix)

        # -----------------------------------------
        # Separation
        # -----------------------------------------
        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
        endMtx2Srt = mathOps.decomposeMatrix(self.result_end.worldMatrix[0], name=self.getName('end_mtx2Srt'))
        midMtx2Srt = mathOps.decomposeMatrix(self.mid_ctrl.worldMatrix[0], name=self.getName('mid_mtx2Srt'))
        upper_sep = mathOps.createTransformedPoint((0, 0, 0), self.mid_ctrl.worldMatrix[0],
                                                   name=self.getName('upper_separation_point'))
        lower_sep = mathOps.createTransformedPoint((0, 0, 0), self.mid_ctrl.worldMatrix[0],
                                                   name=self.getName('lower_separation_point'))
        upperVec = mathOps.subtractVector([midMtx2Srt.outputTranslate, baseMtx2Srt.outputTranslate],
                                          name=self.getName('start_vec'))
        lowerVec = mathOps.subtractVector([endMtx2Srt.outputTranslate, midMtx2Srt.outputTranslate],
                                          name=self.getName('mid_vec'))
        midAngle = mathOps.angleBetween(upperVec.output3D, lowerVec.output3D, name=self.getName('mid_angle'))
        angleMult = mathOps.convert(midAngle.angle, 0.3183, name=self.getName('mid_angle_mult'))
        separationMult = mathOps.multiply(angleMult.output, self.params.mid_separation,
                                          name=self.getName('separation_mult'))
        separationResult = mathOps.multiply(mathOps.getDistance(self.base_srt, self.result_mid)*.2,
                                            separationMult.output,
                                            name=self.getName('separation_result'))
        separationNeg = mathOps.multiply(separationResult.output, -1, name=self.getName('separation_neg'))

        separationResult.output.connect(lower_sep.input1X)
        separationNeg.output.connect(upper_sep.input1X)
        # -----------------------------------------
        # Bendy ctrls
        # -----------------------------------------
        # UPPER
        upperMidPos = mathOps.averageVector([baseMtx2Srt.outputTranslate, upper_sep.output],
                                            name=self.getName('upper_mid_pos'))
        upperMidStartAimMtx = pm.createNode('aimMatrix', name=self.getName('upper_mid_startAim_mtx'))
        upperVec.output3D.connect(upperMidStartAimMtx.primary.primaryTargetVector)
        start_nonRoll_mtx.matrixSum.connect(upperMidStartAimMtx.secondary.secondaryTargetMatrix)
        upperMidStartAimMtx.secondaryMode.set(2)
        upperMidStartAimMtx.secondaryTargetVectorY.set(1)

        upperMidEndAimMtx = pm.createNode('aimMatrix', name=self.getName('upper_mid_endAim_mtx'))
        upperVec.output3D.connect(upperMidEndAimMtx.primary.primaryTargetVector)
        self.mid_ctrl.worldMatrix[0].connect(upperMidEndAimMtx.secondary.secondaryTargetMatrix)
        upperMidEndAimMtx.secondaryMode.set(2)
        upperMidEndAimMtx.secondaryTargetVectorY.set(1)

        if self.invert:
            upperMidStartAimMtx.primaryInputAxisX.set(-1)
            upperMidEndAimMtx.primaryInputAxisX.set(-1)

        upperMidAimMtx = transform.blendMatrices(upperMidStartAimMtx.outputMatrix, upperMidEndAimMtx.outputMatrix,
                                                 name=self.getName('upper_mid_aim_mtx'))
        upperMidPosMtx = mathOps.createComposeMatrix(inputTranslate=upperMidPos.output3D,
                                                     name=self.getName('upper_mid_pos_mtx'))
        upperMidMtx = transform.blend_T_R_matrices(upperMidPosMtx.outputMatrix, upperMidAimMtx.outputMatrix,
                                                   name=self.getName('upper_mid_mtx'))
        
        mtxAttr = upperMidMtx.outputMatrix
        if self.invert:
            upperMidNegMtx = mathOps.multiplyMatrices([self.negScaleMtx.outputMatrix, mtxAttr],
                                                      name=self.getName('upper_mid_neg_mtx'))
            mtxAttr = upperMidNegMtx.matrixSum
        mtxAttr.connect(self.upper_bend_ctrl.offsetParentMatrix)

        # LOWER
        lowerMidPos = mathOps.averageVector([endMtx2Srt.outputTranslate, lower_sep.output],
                                            name=self.getName('lower_mid_pos'))
        lowerMidStartAimMtx = pm.createNode('aimMatrix', name=self.getName('lower_mid_startAim_mtx'))
        lowerVec.output3D.connect(lowerMidStartAimMtx.primary.primaryTargetVector)
        self.mid_ctrl.worldMatrix[0].connect(lowerMidStartAimMtx.secondary.secondaryTargetMatrix)
        lowerMidStartAimMtx.secondaryMode.set(2)
        lowerMidStartAimMtx.secondaryTargetVectorY.set(1)

        lowerMidEndAimMtx = pm.createNode('aimMatrix', name=self.getName('lower_mid_endAim_mtx'))
        lowerVec.output3D.connect(lowerMidEndAimMtx.primary.primaryTargetVector)
        self.result_end_avg.worldMatrix[0].connect(lowerMidEndAimMtx.secondary.secondaryTargetMatrix)
        lowerMidEndAimMtx.secondaryMode.set(2)
        lowerMidEndAimMtx.secondaryTargetVectorY.set(1)

        lowerMidAimMtx = transform.blendMatrices(lowerMidStartAimMtx.outputMatrix, lowerMidEndAimMtx.outputMatrix,
                                                 name=self.getName('lower_mid_aim_mtx'))
        lowerMidPosMtx = mathOps.createComposeMatrix(inputTranslate=lowerMidPos.output3D,
                                                     name=self.getName('lower_mid_pos_mtx'))
        lowerMidMtx = transform.blend_T_R_matrices(lowerMidPosMtx.outputMatrix, lowerMidAimMtx.outputMatrix,
                                                   name=self.getName('lower_mid_mtx'))

        if self.invert:
            lowerMidStartAimMtx.primaryInputAxisX.set(-1)
            lowerMidEndAimMtx.primaryInputAxisX.set(-1)

        mtxAttr = lowerMidMtx.outputMatrix
        if self.invert:
            lowerMidNegMtx = mathOps.multiplyMatrices([self.negScaleMtx.outputMatrix, mtxAttr],
                                                      name=self.getName('lower_mid_neg_mtx'))
            mtxAttr = lowerMidNegMtx.matrixSum
        mtxAttr.connect(self.lower_bend_ctrl.offsetParentMatrix)
        
        # -----------------------------------------
        # Twist segments
        # -----------------------------------------
        # UPPER
        midUpVec = mathOps.createMatrixAxisVector(self.mid_ctrl.worldMatrix[0], (0, 1, 0),
                                                  name=self.getName('mid_up_vec'))
        upperMidMtx2Srt = mathOps.decomposeMatrix(self.upper_bend_ctrl.worldMatrix[0],
                                                  name=self.getName('upperBendCtrl_mtx2Srt'))
        midMtx2Srt = mathOps.decomposeMatrix(self.mid_ctrl.worldMatrix[0], name=self.getName('midCtrl_mtx2Srt'))
        self.upper_crv = curve.curveBetweenPoints(self.result_start_avg, self.mid_ctrl, numPoints=3,
                                                  degree=2, name=self.getName('upper_crv'))
        self.upper_rail = curve.curveBetweenPoints(self.result_start_avg, self.mid_ctrl, numPoints=3,
                                                  degree=2, name=self.getName('upper_rail'))
        self.upper_crv.setParent(self.rig)
        self.upper_rail.setParent(self.rig)

        baseMtx2Srt.outputTranslate.connect(self.upper_crv.controlPoints[0])
        upperMidMtx2Srt.outputTranslate.connect(self.upper_crv.controlPoints[1])
        upper_sep.output.connect(self.upper_crv.controlPoints[2])
        
        upperRailStartPoint = mathOps.createTransformedPoint((0, 1, 0), start_nonRoll_mtx.matrixSum,
                                                             name=self.getName('upper_rail_start_point'))
        upperRailMidPoint = mathOps.createTransformedPoint((0, 1, 0), self.upper_bend_ctrl.worldMatrix[0],
                                                             name=self.getName('upper_rail_mid_point'))
        upperRailEndPoint = mathOps.addVector([midUpVec.output, upper_sep.output],
                                              name=self.getName('upper_rail_end_point'))
        
        upperRailStartPoint.output.connect(self.upper_rail.controlPoints[0])
        upperRailMidPoint.output.connect(self.upper_rail.controlPoints[1])
        upperRailEndPoint.output3D.connect(self.upper_rail.controlPoints[2])

        for i, div in enumerate(self.upperDivs):
            num = str(i + 1).zfill(2)
            param = (0.9 / (self.guide.num_divisions -1)) * i + .05
            mp = curve.createMotionPathNode(self.upper_crv, uValue=param, frontAxis='x', upAxis='y',
                                            name=self.getName('upper_%s_mp' % num))
            railMp = curve.createMotionPathNode(self.upper_rail, uValue=param, follow=0,
                                                name=self.getName('upper_%s_rail_mp' % num))
            upVec = mathOps.subtractVector([railMp.allCoordinates, mp.allCoordinates],
                                           name=self.getName('upper_%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            mp.allCoordinates.connect(div.t)
            mp.rotate.connect(div.r)
            baseMtx2Srt.outputScale.connect(div.s)
        
        # LOWER
        lowerMidMtx2Srt = mathOps.decomposeMatrix(self.lower_bend_ctrl.worldMatrix[0],
                                                  name=self.getName('lowerBendCtrl_mtx2Srt'))
        midMtx2Srt = mathOps.decomposeMatrix(self.mid_ctrl.worldMatrix[0], name=self.getName('midCtrl_mtx2Srt'))
        self.lower_crv = curve.curveBetweenPoints(self.mid_ctrl, self.result_end, numPoints=3,
                                                  degree=2, name=self.getName('lower_crv'))
        self.lower_rail = curve.curveBetweenPoints(self.mid_ctrl, self.result_end, numPoints=3,
                                                   degree=2, name=self.getName('lower_rail'))
        self.lower_crv.setParent(self.rig)
        self.lower_rail.setParent(self.rig)

        lower_sep.output.connect(self.lower_crv.controlPoints[0])
        lowerMidMtx2Srt.outputTranslate.connect(self.lower_crv.controlPoints[1])
        endMtx2Srt.outputTranslate.connect(self.lower_crv.controlPoints[2])

        lowerRailStartPoint = mathOps.addVector([midUpVec.output, lower_sep.output],
                                                name=self.getName('lower_rail_start_point'))
        lowerRailMidPoint = mathOps.createTransformedPoint((0, 1, 0), self.lower_bend_ctrl.worldMatrix[0],
                                                           name=self.getName('lower_rail_mid_point'))
        lowerRailEndPoint = mathOps.createTransformedPoint((0, 1, 0), self.result_midTip.worldMatrix[0],
                                                           name=self.getName('lower_rail_end_point'))

        lowerRailStartPoint.output3D.connect(self.lower_rail.controlPoints[0])
        lowerRailMidPoint.output.connect(self.lower_rail.controlPoints[1])
        lowerRailEndPoint.output.connect(self.lower_rail.controlPoints[2])

        for i, div in enumerate(self.lowerDivs):
            num = str(i + 1).zfill(2)
            param = (0.9 / (self.guide.num_divisions - 1)) * i + .05
            mp = curve.createMotionPathNode(self.lower_crv, uValue=param, frontAxis='x', upAxis='y',
                                            name=self.getName('lower_%s_mp' % num))
            railMp = curve.createMotionPathNode(self.lower_rail, uValue=param, follow=0,
                                                name=self.getName('lower_%s_rail_mp' % num))
            upVec = mathOps.subtractVector([railMp.allCoordinates, mp.allCoordinates],
                                           name=self.getName('lower_%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            mp.allCoordinates.connect(div.t)
            mp.rotate.connect(div.r)
            baseMtx2Srt.outputScale.connect(div.s)

        # ---------------------------------
        # Internal spaces switching setup
        # ---------------------------------
        self.spaces['%s' % (self.pole_ctrl.name())] = 'limb_average: %s.worldMatrix[0]' % self.pole_follow_srt.name()

        # Attach params shape to tip srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.result_tip, lw=1, wt=1)
        pm.delete(tempJoint)

        # Connect pole guide
        ikMidMtx2Srt = mathOps.decomposeMatrix(ik_mid_mtx.matrixSum, name=self.getName('ik_pole_guide_start_mtx2Srt'))
        poleMtx2Srt = mathOps.decomposeMatrix(self.pole_ctrl.worldMatrix[0], name=self.getName('ik_pole_guide_end_mtx2Srt'))
        ikMidMtx2Srt.outputTranslate.connect(self.pole_guide.controlPoints[0])
        poleMtx2Srt.outputTranslate.connect(self.pole_guide.controlPoints[1])
        self.pole_guide.overrideEnabled.set(1)
        self.pole_guide.overrideDisplayType.set(1)

    def finish(self):
        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodeList = self.controls_list
        attrList = ['sx', 'sy', 'sz', 'visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = [self.fk_start_ctrl, self.fk_end_ctrl, self.fk_mid_ctrl, self.ik_ctrl]
        attribute.channelControl(nodeList=nodeList, attrList=['rotateOrder'], keyable=1, lock=0)

        nodeList = [self.upper_bend_ctrl, self.lower_bend_ctrl, self.mid_ctrl]
        attrList = ['ry', 'rz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['tx', 'ty', 'tz']
        attribute.channelControl(nodeList=[self.fk_start_ctrl], attrList=attrList)

        self.setColours(self.guide)

        # --------------------------------------------------
        # Set auto vis switch on ik / fk controls
        # --------------------------------------------------
        isIk = pm.createNode('condition', name=self.getName('is_ikfk_1'))
        self.params.ikfk_blend.connect(isIk.firstTerm)
        isIk.secondTerm.set(1)
        isIk.colorIfTrueR.set(0)
        isIk.colorIfFalseR.set(1)
        for node in [self.ik_ctrl, self.pole_ctrl, self.pole_guide]:
            pm.setAttr(node.visibility, lock=0)
            isIk.outColorR.connect(node.visibility)

        isFk = pm.createNode('condition', name=self.getName('is_ikfk_0'))
        isFk.secondTerm.set(0)
        self.params.ikfk_blend.connect(isFk.firstTerm)
        isFk.colorIfTrueR.set(0)
        isFk.colorIfFalseR.set(1)
        for node in [self.fk_start_ctrl]:
            pm.setAttr(node.visibility, lock=0)
            isFk.outColorR.connect(node.visibility)

        # Bendy ctrls vis
        for ctrl in [self.upper_bend_ctrl, self.lower_bend_ctrl, self.mid_ctrl]:
            pm.setAttr(ctrl.visibility, lock=0)
            self.params.show_bendy_ctrls.connect(ctrl.visibility)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TLimbIkFk(guide)


'''
TO DO
-- IK FK snapping (serialize offset onto nodes like with defaults, message connect target)
-- Gimbal/Offset ctrls
'''

