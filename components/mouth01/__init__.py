from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve, anim
reload(components)
reload(mathOps)
reload(curve)
reload(anim)

import pymel.core as pm

class TMouth01(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'mouth01')
        print 'Created Mouth01 Component: %s' % self.comp_name

    def addObjects(self, guide):
        ctrlSize = mathOps.getDistance(guide.locs[1], guide.locs[-1])*.1

        self.jaw = guide.root.jaw.get() != ''

        crvPoints = [pm.xform(loc, q=1, ws=1, t=1) for loc in guide.locs[1:]]
        self.base_crv = curve.curveThroughPoints(self.getName('base_crv'), crvPoints, degree=3)
        self.base_crv.setParent(self.rig)
        if self.jaw:
            self.jawCrv = curve.curveThroughPoints(self.getName('jaw_crv'), crvPoints, degree=3)
            self.jawCrv.setParent(self.rig)

        # MAIN CONTROLS

        upperLocs = guide.getGuideLocs(guide.root, locType='upperCtrl')
        lowerLocs = guide.getGuideLocs(guide.root, locType='lowerCtrl')
        startXform = mathOps.getInverseHandedMatrix(upperLocs[0].worldMatrix[0].get())
        self.start_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize,
                                       name=self.getName('start'), xform=startXform, parent=self.controls)

        upperMidXform = upperLocs[len(upperLocs)/2].worldMatrix[0].get()
        self.upper_mid_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize,
                                           name=self.getName('upper_mid'), xform=upperMidXform, parent=self.controls)
        lowerMidXform = lowerLocs[len(lowerLocs)/2].worldMatrix[0].get()
        self.lower_mid_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize,
                                           name=self.getName('lower_mid'), xform=lowerMidXform, parent=self.controls)

        endXform = upperLocs[-1].worldMatrix[0].get()
        self.end_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize,
                                       name=self.getName('end'), xform=endXform, parent=self.controls)

        # CURVES
        points = [loc.worldMatrix[0].get().translate.get() for loc in upperLocs]
        lowerPoints = [loc.worldMatrix[0].get().translate.get() for loc in lowerLocs]

        lowerPoints.reverse()
        circlePoints = points + lowerPoints + points[:2]
        circleKnots = range(len(circlePoints)+1)
        self.circle_crv = pm.curve(p=circlePoints, k=circleKnots, d=2, name=self.getName('circle_crv'))
        self.circle_crv.setParent(self.rig)

        points = [points[0]] + points + [points[-1]]
        self.upper_crv = curve.curveThroughPoints(self.getName('upper_crv'), points, degree=2)
        self.upper_crv.setParent(self.rig)

        lowerPoints.reverse()
        lowerPoints = points[:2] + lowerPoints + [points[-1], points[-1]]
        self.lower_crv = curve.curveThroughPoints(self.getName('lower_crv'), lowerPoints, degree=2)
        self.lower_crv.setParent(self.rig)

        # SUB CONTROLS
        self.upper_ctrls = []
        self.lower_ctrls = []

        for index, loc in enumerate(upperLocs):
            num = str(index+1).zfill(2)
            xform = loc.worldMatrix[0].get()
            if index < len(upperLocs)/2:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='ball', size=ctrlSize*.25, name=self.getName('upper_%s' % num),
                                xform=xform, parent=self.controls)
            self.upper_ctrls.append(ctrl)

        for index, loc in enumerate(lowerLocs):
            num = str(index+1).zfill(2)
            xform = loc.worldMatrix[0].get()
            if index < len(lowerLocs)/2:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='ball', size=ctrlSize*.25, name=self.getName('lower_%s' % num),
                                xform=xform, parent=self.controls)
            self.lower_ctrls.append(ctrl)

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'seal', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'seal_height', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'seal_falloff', minValue=0, maxValue=1)

    def addSystems(self):
        base_crv_offset_mtx = mathOps.composeMatrixFromMatrix(self.base_srt.worldInverseMatrix[0],
                                                              name=self.getName('base_crv_offset_mtx'))
        base_crv_mtx = mathOps.multiplyMatrices([base_crv_offset_mtx.outputMatrix, self.base_srt.worldMatrix[0]],
                                                name=self.getName('base_crv_mtx'))
        base_crv_mtx.matrixSum.connect(self.base_crv.offsetParentMatrix)

        # Drive sub ctrls along base crv (or blend of base and jaw crvs if jaw exists)
        crvLen = curve.getCurveLength(self.base_crv)
        startParamMult = mathOps.divide(self.start_ctrl.tx, crvLen*-1, name=self.getName('start_param_mult'))
        startParamSum = mathOps.addScalar([startParamMult.outputX, self.guide.root.corner_start.get()],
                                          name=self.getName('start_param_sum'))
        
        upperMidParamMult = mathOps.divide(self.upper_mid_ctrl.tx, crvLen, name=self.getName('upper_mid_param_mult'))
        upperMidParamSum = mathOps.addScalar([upperMidParamMult.outputX, 0.5], name=self.getName('upper_mid_param_sum'))
        
        endParamMult = mathOps.divide(self.end_ctrl.tx, crvLen, name=self.getName('end_param_mult'))
        endParamSum = mathOps.addScalar([endParamMult.outputX, 1.0-self.guide.root.corner_start.get()],
                                        name=self.getName('end_param_sum'))
        
        lowerMidParamMult = mathOps.divide(self.lower_mid_ctrl.tx, crvLen, name=self.getName('lower_mid_param_mult'))
        lowerMidParamSum = mathOps.addScalar([lowerMidParamMult.outputX, 0.5], name=self.getName('lower_mid_param_sum'))
        upperPoints = []
        lowerPoints = []
        for index, ctrl in enumerate(self.upper_ctrls):
            num = str(index+1).zfill(2)
            param = (1.0 / (len(self.upper_ctrls)/2))*index
            blend = mathOps.blendScalarAttrs(startParamSum.output1D, upperMidParamSum.output1D, param,
                                             name=self.getName('upper_%s_param' % num))
            if index >= len(self.upper_ctrls)/2:
                param = (1.0 / (len(self.upper_ctrls)/2))*(index-(len(self.upper_ctrls)/2))
                blend = mathOps.blendScalarAttrs(upperMidParamSum.output1D, endParamSum.output1D, param,
                                                 name=self.getName('upper_%s_param' % num))
            mp = curve.createMotionPathNode(self.base_crv, uValue=blend.output, wut=2, wuo=self.base_srt,
                                            upAxis='z', name=self.getName('upper_%s_base_mp' % num))
            mp.worldUpVector.set(0, 0, 1)
            crvMtx = mathOps.createComposeMatrix(mp.allCoordinates, mp.rotate, name=self.getName('upper_%s_crv_mtx' % num))
            offset = pm.xform(ctrl, q=1, ws=1, t=1) - mp.allCoordinates.get()
            offsetMtx = mathOps.createComposeMatrix(offset, name=self.getName('upper_%s_offset_mtx' % num))
            offsetSum = mathOps.addVector([offset, (0, 0, 0)], name=self.getName('upper_%s_offset_sum' % num))
            if 0 < index < len(self.upper_ctrls)-1:
                self.upper_mid_ctrl.ty.connect(offsetSum.input3D[1].input3Dy)
                self.upper_mid_ctrl.tz.connect(offsetSum.input3D[1].input3Dz)
            elif index == 0:
                self.start_ctrl.ty.connect(offsetSum.input3D[1].input3Dy)
                self.start_ctrl.tz.connect(offsetSum.input3D[1].input3Dz)
            else:
                self.end_ctrl.ty.connect(offsetSum.input3D[1].input3Dy)
                self.end_ctrl.tz.connect(offsetSum.input3D[1].input3Dz)
            offsetSum.output3D.connect(offsetMtx.inputTranslate)

            if index < len(self.upper_ctrls)/2:
                offsetMtx.inputScaleX.set(-1)
            ctrlMtx = mathOps.multiplyMatrices([offsetMtx.outputMatrix, crvMtx.outputMatrix],
                                               name=self.getName('upper_%s_ctrl_mtx' % num))
            ctrlMtx.matrixSum.connect(ctrl.offsetParentMatrix)
            ctrl.t.set((0, 0, 0))
            ctrl.r.set((0, 0, 0))

            d = mathOps.decomposeMatrix(ctrl.worldMatrix[0], name=self.getName('upper_%s_mtx2Srt' % num))
            # d.outputTranslate.connect(self.upper_crv.controlPoints[index+1])
            if index == 0:
                d.outputTranslate.connect(self.lower_crv.controlPoints[index])
                d.outputTranslate.connect(self.lower_crv.controlPoints[index+1])
                d.outputTranslate.connect(self.upper_crv.controlPoints[index])
                d.outputTranslate.connect(self.upper_crv.controlPoints[index+1])
                d.outputTranslate.connect(self.circle_crv.controlPoints[index])
            elif index == len(self.upper_ctrls)-1:
                d.outputTranslate.connect(self.circle_crv.controlPoints[index])
                d.outputTranslate.connect(self.upper_crv.controlPoints[index+1])
                d.outputTranslate.connect(self.upper_crv.controlPoints[index+2])
                d.outputTranslate.connect(self.lower_crv.controlPoints[index+1])
                d.outputTranslate.connect(self.lower_crv.controlPoints[index+2])
            else:
                upperPoints.append(d)

        for index, ctrl in enumerate(self.lower_ctrls):
            num = str(index+1).zfill(2)
            param = (1.0 / (len(self.upper_ctrls)/2))*(index+1)
            blend = mathOps.blendScalarAttrs(startParamSum.output1D, lowerMidParamSum.output1D, param,
                                                 name=self.getName('upper_%s_param' % num))
            if index >= len(self.upper_ctrls)/2:
                param = (1.0 / (len(self.upper_ctrls)/2))*(index-((len(self.upper_ctrls)/2)-1))
                blend = mathOps.blendScalarAttrs(lowerMidParamSum.output1D, endParamSum.output1D, param,
                                                 name=self.getName('upper_%s_param' % num))
            mp = curve.createMotionPathNode(self.base_crv, uValue=blend.output, wut=2, wuo=self.base_srt,
                                            upAxis='z', name=self.getName('lower_%s_base_mp' % num))
            mp.worldUpVector.set(0, 0, 1)
            crvMtx = mathOps.createComposeMatrix(mp.allCoordinates, mp.rotate, name=self.getName('lower_%s_crv_mtx' % num))
            offset = pm.xform(ctrl, q=1, ws=1, t=1) - mp.allCoordinates.get()
            offsetMtx = mathOps.createComposeMatrix(offset, name=self.getName('lower_%s_offset_mtx' % num))
            offsetSum = mathOps.addVector([offset, (0, 0, 0)], name=self.getName('upper_%s_offset_sum' % num))
            self.lower_mid_ctrl.ty.connect(offsetSum.input3D[1].input3Dy)
            self.lower_mid_ctrl.tz.connect(offsetSum.input3D[1].input3Dz)
            offsetSum.output3D.connect(offsetMtx.inputTranslate)
            if index < len(self.upper_ctrls)/2:
                offsetMtx.inputScaleX.set(-1)
            ctrlMtx = mathOps.multiplyMatrices([offsetMtx.outputMatrix, crvMtx.outputMatrix],
                                               name=self.getName('lower_%s_ctrl_mtx' % num))
            ctrlMtx.matrixSum.connect(ctrl.offsetParentMatrix)
            ctrl.t.set((0, 0, 0))
            ctrl.r.set((0, 0, 0))

            d = mathOps.decomposeMatrix(ctrl.worldMatrix[0], name=self.getName('lower_%s_mtx2Srt' % num))
            # d.outputTranslate.connect(self.lower_crv.controlPoints[index+2])
            lowerPoints.append(d)

        # blending between upper and lower points for lip sealing
        numPoints = len(upperPoints) + len(lowerPoints) + 2
        for index, point in enumerate(upperPoints):
            num = str(index+1).zfill(2)

            # Find distance from corner
            param = (1.0 / (len(upperPoints)+1))*(index+1)
            cornerDist = min(param, 1 - param)
            blendStart = mathOps.subtractScalar([cornerDist, self.params.seal_falloff],
                                                name=self.getName('seal_%s_start' % num))
            blendStartClamp = mathOps.clamp(blendStart.output1D, 0, 1, name=self.getName('seal_%s_start_clamp' % num))
            blendEnd = mathOps.addScalar([cornerDist, self.params.seal_falloff],
                                                name=self.getName('seal_%s_end' % num))
            blendEndClamp = mathOps.clamp(blendEnd.output1D, 0, 1, name=self.getName('seal_%s_end_clamp' % num))
            blendRemap = mathOps.remap(self.params.seal, blendStartClamp.outputR, blendEndClamp.outputR, 0, 1,
                                       name=self.getName('seal_%s_remap' % num))
            blendEase = anim.easeCurve(blendRemap.outValueX, name=self.getName('seal_%s_ease' % num))

            sealBlend = mathOps.pairBlend(translateA=lowerPoints[index].outputTranslate,
                                          translateB=point.outputTranslate, weight=self.params.seal_height,
                                          name=self.getName('seal_%s_pos' % num))

            upperBlend = mathOps.pairBlend(translateA=point.outputTranslate,
                                           translateB=sealBlend.outTranslate, weight=blendEase.output,
                                           name=self.getName('upper_seal_%s_pos' % num))
            upperBlend.outTranslate.connect(self.upper_crv.controlPoints[index+2])
            upperBlend.outTranslate.connect(self.circle_crv.controlPoints[index+1])

            lowerBlend = mathOps.pairBlend(translateA=lowerPoints[index].outputTranslate,
                                           translateB=sealBlend.outTranslate, weight=blendEase.output,
                                           name=self.getName('lower_seal_%s_pos' % num))
            lowerBlend.outTranslate.connect(self.lower_crv.controlPoints[index+2])
            lowerBlend.outTranslate.connect(self.circle_crv.controlPoints[numPoints-(index+1)])

        conn = pm.listConnections(self.circle_crv.controlPoints[0], d=0, p=1)[0]
        conn.connect(self.circle_crv.controlPoints[numPoints], f=1)
        conn = pm.listConnections(self.circle_crv.controlPoints[1], d=0, p=1)[0]
        conn.connect(self.circle_crv.controlPoints[numPoints+1], f=1)

    def finish(self):
        self.setColours(self.guide)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TMouth01(guide)
