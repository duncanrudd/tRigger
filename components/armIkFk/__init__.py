from tRigger import components
from tRigger.core import attribute, transform, dag, mathOps
import pymel.core as pm
reload(components)
reload(transform)

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
        pole_ctrl_mtx = ik_ctrl_mtx = transform.getMatrixFromPos(pm.xform(guide.locs[4], q=1, ws=1, t=1))
        self.pole_ctrl = self.addCtrl(shape='ball', size=ctrlSize*.05,
                                      name=self.getName('pole'), xform=pole_ctrl_mtx, parent=self.controls)

        # --------------------------------
        # RESULT
        # --------------------------------
        offsetMult = 1
        if self.invert:
            offsetMult = -1

        # Start
        aimVec = _getVec(guide.locs[1], guide.locs[5], self.invert)
        upVecTemp = _getVec(guide.locs[4], guide.locs[1])
        sideVec = aimVec.cross(upVecTemp).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(guide.locs[5], q=1, ws=1, t=1)
        startXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        self.result_start = dag.addChild(self.rig, 'group', name=self.getName('result_start_srt'))
        self.result_start.offsetParentMatrix.set(startXform)
        self.result_startTip = dag.addChild(self.result_start, 'group', name=self.getName('result_startTip_srt'))
        self.result_startTip.tx.set(mathOps.getDistance(self.fk_start_ctrl, self.fk_mid_ctrl)*offsetMult)

        # Build mid matrix
        aimVec = _getVec(guide.locs[3], guide.locs[1], self.invert)
        sideVec = upVec.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(guide.locs[1], q=1, ws=1, t=1)
        midXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        self.result_mid = dag.addChild(self.result_startTip, 'group', name=self.getName('result_mid_srt'))
        self.result_mid.offsetParentMatrix.set(midXform*(self.result_startTip.worldInverseMatrix[0].get()))
        self.result_midTip = dag.addChild(self.result_mid, 'group', name=self.getName('result_midTip_srt'))
        self.result_midTip.tx.set(mathOps.getDistance(self.fk_mid_ctrl, self.fk_end_ctrl)*offsetMult)

        # Build end matrix
        aimVec = _getVec(guide.locs[2], guide.locs[3], self.invert)
        upVecTemp = mathOps.getMatrixAxisAsVector(guide.locs[3].worldMatrix[0].get(), 'z')
        sideVec = upVecTemp.cross(aimVec).normal()
        upVec = aimVec.cross(sideVec).normal()
        startPos = pm.xform(self.fk_end_ctrl, q=1, ws=1, t=1)
        endXform = pm.datatypes.Matrix(aimVec, sideVec, upVec, startPos)
        self.result_end = dag.addChild(self.result_midTip, 'group', name=self.getName('result_end_srt'))
        self.result_end.offsetParentMatrix.set(endXform*(self.result_midTip.worldInverseMatrix[0].get()))

        # Mid Ctrl
        self.mid_ctrl = self.addCtrl(shape='ball', size=ctrlSize*.075,
                                     name=self.getName('mid'),
                                     xform=self.fk_start_ctrl.getParent().worldMatrix[0].get(),
                                     parent=self.fk_start_ctrl.getParent())
        ori = pm.orientConstraint(self.fk_start_ctrl, self.fk_mid_ctrl, self.mid_ctrl, mo=0)
        sc = pm.scaleConstraint(self.fk_start_ctrl, self.mid_ctrl, mo=0)
        p = pm.pointConstraint(self.fk_mid_ctrl, self.mid_ctrl, mo=0)
        pm.delete([ori, sc, p])
        self.mid_ctrl.setParent(self.controls)
        transform.bakeSrtToOffsetParentMtx(self.mid_ctrl)


        # Call overloaded method of parent class
        components.TBaseComponent.addObjects(self, guide)

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'ikfk_blend', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'mid_slide', minValue=-1, maxValue=1)
        attribute.addFloatAttr(self.params, 'stretch', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'pin_to_pole', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'softness', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'extend', minValue=0, value=1)

        # Call overloaded method of parent class
        #components.TBaseComponent.addAttributes(self)

    def addSystems(self):
        # --------------------------------
        # IK
        # --------------------------------
        # Non roll base - starting point for basis and stable parent space for pole ctrl
        axis = 'x'
        if self.invert:
            axis = '-x'
        base_nonRoll_mtx = transform.createNonRollMatrix(self.base_srt, self.ik_displaced, axis=axis,
                                                         name=[self.getName('base_nonRoll_orbit_mtx'),
                                                               self.getName('base_nonRoll_aim_mtx')])[1]

        # IK Basis
        ik_basis_mtx = transform.createAimMatrix(base_nonRoll_mtx.outputMatrix, self.pole_ctrl,
                                                 name=self.getName('ik_basis_mtx'))
        ik_basis_mtx.primaryMode.set(0)
        ik_basis_mtx.primaryInputAxis.set((1, 0, 0))
        ik_basis_mtx.secondaryMode.set(1)
        ik_basis_mtx.secondaryInputAxis.set((0, 0, -1))
        conn = pm.listConnections(ik_basis_mtx.primaryTargetMatrix, p=1, d=0)[0]
        conn.connect(ik_basis_mtx.secondaryTargetMatrix)
        conn.disconnect(ik_basis_mtx.primaryTargetMatrix)

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
        stretchLen = mathOps.clamp(targetDistScaled.outputX, extendLen.output, 1000000,
                                   name=self.getName('stretch_len'))
        stretchBlend = mathOps.blendScalarAttrs(extendLen.output, stretchLen.outputR, self.params.stretch,
                                                name=self.getName('stretch_blend'))
        startPoleDist = mathOps.distance(self.base_srt, self.pole_ctrl, name=self.getName('ik_start2Pole_dist'))
        startPoleDistScaled = mathOps.divide(startPoleDist.distance, d.outputScaleX,
                                             name=self.getName('ik_start2Pole_dist_normal'))
        endPoleDist = mathOps.distance(self.ik_displaced, self.pole_ctrl, name=self.getName('ik_end2Pole_dist'))
        endPoleDistScaled = mathOps.divide(endPoleDist.distance, d.outputScaleX,
                                           name=self.getName('ik_end2Pole_dist_normal'))

        upperRatio = mathOps.addScalar([mathOps.getDistance(self.fk_start_ctrl,self.fk_mid_ctrl)/chainLen.output1D.get(),
                                        self.params.mid_slide], name=self.getName('upper_ratio'))
        upperRatioClamp = mathOps.clamp(upperRatio.output1D, 0, 1, name=self.getName('upper_ratio_clamp'))
        lowerRatio = mathOps.subtractScalar([1.0, upperRatioClamp.outputR], name=self.getName('lower_ratio'))


        upperLen = mathOps.multiply(stretchBlend.output, upperRatioClamp.outputR, name=self.getName('upper_len'))
        upperResult = mathOps.blendScalarAttrs(upperLen.output, startPoleDistScaled.outputX, self.params.pin_to_pole,
                                               name=self.getName('upperResult_len'))

        lowerLen = mathOps.multiply(stretchBlend.output, lowerRatio.output1D, name=self.getName('lower_len'))
        lowerResult = mathOps.blendScalarAttrs(lowerLen.output, endPoleDistScaled.outputX, self.params.pin_to_pole,
                                               name=self.getName('lowerResult_len'))

        tempAngles = pm.createNode('animBlendNodeAdditiveDA')

        # Expression
        exprString = 'float $upLen = %s.output;\n' % upperResult.name()
        exprString += 'float $lowLen = %s.output;\n' % lowerResult.name()
        exprString += 'float $restLen = $upLen+$lowLen;\n'
        exprString += 'float $targLen = %s.outputX;\n\n' % targetDistScaled.name()
        exprString += 'float $upSq = $upLen*$upLen;\n'
        exprString += 'float $lowSq = $lowLen*$lowLen;\n'
        exprString += 'float $targSq = $targLen*$targLen;\n\n'
        exprString += 'float $startAngle = 0.0;\n'
        exprString += 'float $midAngle = 0.0;\n\n'
        exprString += 'if ($targLen < $restLen){;\n\t'
        exprString += '$startAngle = acos(($upSq + $targSq - $lowSq)/(2 * $upLen * $targLen));\n\t'
        exprString += '$startAngle = acos(($upSq + $targSq - $lowSq)/(2 * $upLen * $targLen));\n\t'
        exprString += '$midAngle = acos(($upSq + $lowSq - $targSq)/(2 * $upLen * $lowLen)) - deg_to_rad(180);\n}\n\n'
        if self.invert:
            exprString += '%s.inputA = -$startAngle;\n' % tempAngles
            exprString += '%s.inputB = -$midAngle;' % tempAngles
        else:
            exprString += '%s.inputA = $startAngle;\n' % tempAngles
            exprString += '%s.inputB = $midAngle;' % tempAngles

        expr = pm.expression(s=exprString, name=self.getName('ik_expr'), alwaysEvaluate=0, unitConversion='none')






def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TArmIkFk(guide)

