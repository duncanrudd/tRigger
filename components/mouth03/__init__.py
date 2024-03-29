from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve, anim, icon
import math
reload(components)
reload(mathOps)
reload(curve)
reload(anim)

import pymel.core as pm

class TMouth03(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'mouth03')
        print 'Created Mouth03 Component: %s' % self.comp_name

    def addObjects(self, guide):
        ctrlSize = mathOps.getDistance(guide.locs[1], guide.locs[-1])*.1

        self.jaw = guide.jawLoc.spaces.get() != ''

        crvPoints = [pm.xform(loc, q=1, ws=1, t=1) for loc in guide.locs[1:-1]]
        self.base_crv = curve.curveThroughPoints(self.getName('base_crv'), crvPoints, degree=3)
        self.base_crv.setParent(self.rig)
        if self.jaw:
            self.jaw_srt = dag.addChild(self.rig, 'group', self.getName('jaw_srt'))
            transform.align(self.jaw_srt, self.base_srt)
            transform.bakeSrtToOffsetParentMtx(self.jaw_srt)
            self.jaw_crv = curve.curveThroughPoints(self.getName('jaw_crv'), crvPoints, degree=3)
            self.jaw_crv.setParent(self.jaw_srt)
            self.jaw_crv = curve.curveThroughPoints(self.getName('jaw_crv'), crvPoints, degree=3)
            self.jaw_crv.setParent(self.jaw_srt)


        upperLocs = guide.getGuideLocs(guide.root, locType='upperCtrl')
        lowerLocs = guide.getGuideLocs(guide.root, locType='lowerCtrl')

        # CURVES
        points = [loc.worldMatrix[0].get().translate.get() for loc in upperLocs]
        lowerPoints = [loc.worldMatrix[0].get().translate.get() for loc in lowerLocs]

        self.upper_crv = curve.curveThroughPoints(self.getName('upper_crv'), points, degree=2)
        self.upper_crv.setParent(self.rig)

        lowerPointsList = [points[0]] + lowerPoints + [points[-1]]
        self.lower_crv = curve.curveThroughPoints(self.getName('lower_crv'), lowerPointsList, degree=2)
        self.lower_crv.setParent(self.rig)

        # CONTROLS
        self.upper_ctrls = []
        self.lower_ctrls = []

        for index, loc in enumerate(upperLocs):
            num = str(index+1).zfill(2)
            xform = loc.worldMatrix[0].get()
            if index < len(upperLocs)/2:
                xform = mathOps.getInverseHandedMatrix(xform)
            shape = 'triNorth'
            if loc == upperLocs[0]:
                shape = 'triWest'
            elif loc == upperLocs[-1]:
                shape = 'triWest'
            ctrl = self.addCtrl(shape=shape, size=ctrlSize, name=self.getName('upper_%s' % num),
                                xform=xform, parent=self.controls, metaParent=self.base_srt, buffer=1)
            self.upper_ctrls.append(ctrl)

        for index, loc in enumerate(lowerLocs):
            num = str(index+1).zfill(2)
            xform = loc.worldMatrix[0].get()
            if index < len(lowerLocs)/2:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='triSouth', size=ctrlSize, name=self.getName('lower_%s' % num),
                                xform=xform, parent=self.controls, metaParent=self.base_srt, buffer=1)
            self.lower_ctrls.append(ctrl)

        # DIVS and JOINTS
        self.upperDivs = []
        self.lowerDivs = []
        self.local_srt = dag.addChild(self.rig, 'group', name=self.getName('local_srt'))
        for i in range(guide.num_divisions):
            num = str(i+1).zfill(2)
            srt = dag.addChild(self.local_srt, 'group', name=self.getName('upper_%s_srt' % num))
            self.upperDivs.append(srt)
            if 0 < i < guide.num_divisions-1:
                srt = dag.addChild(self.local_srt, 'group', name=self.getName('lower_%s_srt' % num))
                self.lowerDivs.append(srt)

        if guide.root.add_joint.get():
            for srt in self.upperDivs + self.lowerDivs:
                j = pm.createNode('joint', name=srt.name().replace('_srt', '_jnt'))
                self.joints_list.append({'joint': j, 'driver': srt})

        # MAP TO GUIDE LOCS
        mappingPairs = []
        if self.jaw:
            mappingPairs.append([self.jaw_srt, guide.jawLoc])
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'start_seal', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'end_seal', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'seal_height', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'seal_falloff', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'anchor_mid', minValue=0, maxValue=1)
        if self.jaw:
            attribute.addFloatAttr(self.params, 'start_jaw_follow', minValue=0, maxValue=1)
            attribute.addFloatAttr(self.params, 'end_jaw_follow', minValue=0, maxValue=1)

    def addSystems(self):
        # List of biased param values for use throughout
        params = [(1.0 / (len(self.upper_ctrls)-1)) * i for i in range(len(self.upper_ctrls))]
        for index, param in enumerate(params):
            if param <= 0.5:
                params[index] = 0.5*(1.0-(math.cos((math.pi / 2.0)*(param*2))))
            else:
                params[index] = (0.5*(math.sin((math.pi / 2.0)*((param-0.5)*2))))+0.5

        midIndex = len(self.upper_ctrls)/2

        upperLocs = self.guide.getGuideLocs(self.guide.root, locType='upperCtrl')
        lowerLocs = self.guide.getGuideLocs(self.guide.root, locType='lowerCtrl')

        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))

        if self.jaw:
            startFollowRev = mathOps.reverse(self.params.start_jaw_follow, name=self.getName('start_jawFollow_rev'))
            endFollowRev = mathOps.reverse(self.params.end_jaw_follow, name=self.getName('end_jawFollow_rev'))

        crvLen = curve.getCurveLength(self.base_crv)
        crvLenScaled = mathOps.multiply(crvLen, baseMtx2Srt.outputScaleX, self.getName('base_crv_scaled_len'))
        startParamMult = mathOps.divide(self.upper_ctrls[0].getParent().tx, crvLen,
                                        name=self.getName('start_param_mult'))
        startParamSum = mathOps.addScalar([startParamMult.outputX, self.guide.root.corner_start.get()],
                                          name=self.getName('start_param_sum'))

        endParamMult = mathOps.divide(self.upper_ctrls[-1].tx, crvLen, name=self.getName('end_param_mult'))
        endParamSum = mathOps.addScalar([endParamMult.outputX, 1.0-self.guide.root.corner_start.get()],
                                        name=self.getName('end_param_sum'))

        startParamHalf = mathOps.multiply(startParamMult.outputX, 0.5, name=self.getName('half_start_param_mult'))
        endParamHalf = mathOps.multiply(endParamMult.outputX, 0.5, name=self.getName('half_end_param_mult'))

        startParamAffectsMid = mathOps.blendScalarAttrs(startParamHalf.output, 0, self.params.anchor_mid,
                                                        name=self.getName('start_affects_mid_weight'))
        endParamAffectsMid = mathOps.blendScalarAttrs(endParamHalf.output, 0, self.params.anchor_mid,
                                                      name=self.getName('end_affects_mid_weight'))

        upperMidParamMult = mathOps.divide(self.upper_ctrls[midIndex].tx, crvLen, name=self.getName('upper_mid_param_mult'))
        upperMidParamSum = mathOps.addScalar([upperMidParamMult.outputX, startParamAffectsMid.output,
                                              endParamAffectsMid.output, 0.5], name=self.getName('upper_mid_param_sum'))

        lowerMidParamMult = mathOps.divide(self.lower_ctrls[midIndex-1].tx, crvLen, name=self.getName('lower_mid_param_mult'))
        lowerMidParamSum = mathOps.addScalar([lowerMidParamMult.outputX, startParamAffectsMid.output,
                                              endParamAffectsMid.output, 0.5], name=self.getName('lower_mid_param_sum'))

        # Drive base and jaw curves
        base_crv_offset = mathOps.composeMatrixFromMatrix(self.base_srt.worldInverseMatrix,
                                                     name=self.getName('jaw_offset_mtx'))
        base_crv_mtx = mathOps.multiplyMatrices([base_crv_offset.outputMatrix, self.base_srt.worldMatrix[0]],
                                            name=self.getName('base_crv_mtx'))
        base_crv_mtx.matrixSum.connect(self.base_crv.offsetParentMatrix)

        # Drive controls
        def _driveCtrl(index, ctrl, prefix='upper'):
            num = str(index+1).zfill(2)
            negX = mathOps.multiply(ctrl.tx, -1, name=self.getName('%s_%s_negX' % (prefix, num)))
            negX.output.connect(ctrl.getParent().tx)
            param = params[index]
            if index < midIndex:
                paramBlend = param*2
                start = startParamSum.output1D
                if prefix == 'upper':
                    end = upperMidParamSum.output1D
                else:
                    end = lowerMidParamSum.output1D
            else:
                paramBlend = (param-0.5)*2
                if prefix == 'upper':
                    start = upperMidParamSum.output1D
                else:
                    start = lowerMidParamSum.output1D
                end = endParamSum.output1D
            paramBase = mathOps.blendScalarAttrs(start, end, paramBlend,
                                                 name=self.getName('%s_%s_base_param' % (prefix, num)))
            paramAttr = paramBase.output
            if 0 < index < len(self.upper_ctrls)-1 and index != midIndex:
                txAttr = ctrl.getParent().tx
                if param > 0.5:
                    txAttr=ctrl.tx
                paramMult = mathOps.divide(txAttr, crvLen,
                                           name=self.getName('%s_%s_param_mult' % (prefix, num)))
                paramSum = mathOps.addScalar([paramBase.output, paramMult.outputX],
                                             name=self.getName('%s_%s_param_sum' % (prefix, num)))
                paramAttr = paramSum.output1D

                # Set up influence of corner and mid controls on in between controls
                if index < midIndex:
                    factor = (1-(param*2))*(1-(param*2))*.75
                    cornerMult = mathOps.multiplyVector(self.upper_ctrls[0].t,
                                                        (factor, factor, factor),
                                                        name=self.getName('%s_%s_corner_translate_mult' % (prefix, num)))
                else:
                    factor = ((param-.5)*2)*((param-.5)*2)*.75
                    cornerMult = mathOps.multiplyVector(self.upper_ctrls[-1].t,
                                                        (factor, factor, factor),
                                                        name=self.getName('%s_%s_corner_translate_mult' % (prefix, num)))
                if prefix == 'upper':
                    midCtrl = self.upper_ctrls[midIndex]
                else:
                    midCtrl = self.lower_ctrls[midIndex-1]
                midMult = mathOps.multiplyVector(midCtrl.t, (1-factor, 1-factor, 1-factor),
                                                 name=self.getName('%s_%s_mid_translate_mult' % (prefix, num)))
                translateSum = mathOps.addVector([cornerMult.output, midMult.output],
                                                 name=self.getName('%s_%s_translate_sum' % (prefix, num)))
                translateSum.output3Dy.connect(ctrl.getParent().ty)
                translateSum.output3Dz.connect(ctrl.getParent().tz)

            mp = curve.createMotionPathNode(self.base_crv, uValue=paramAttr, wut=2, wuo=self.base_srt,
                                            upAxis='z', name=self.getName('%s_%s_base_mp' % (prefix, num)))
            mp.worldUpVector.set(0, 0, 1)
            targPos = mp.allCoordinates
            targRot = mp.rotate
            if self.jaw:
                jawMp = curve.createMotionPathNode(self.jaw_crv, uValue=paramAttr, wut=2, wuo=self.jaw_srt,
                                                   upAxis='z', name=self.getName('%s_%s_jaw_mp' % (prefix, num)))
                jawMp.worldUpVector.set(0, 0, 1)
                if 0 < index < len(self.upper_ctrls)-1:
                    if index < midIndex:
                        attr = self.params.start_jaw_follow
                        if prefix == 'lower':
                            attr = startFollowRev.outputX
                        factor = (1-(param*2))*(1-(param*2))*.75
                    else:
                        attr = self.params.end_jaw_follow
                        if prefix == 'lower':
                            attr = endFollowRev.outputX
                        factor = ((param-.5)*2)*((param-.5)*2)*.75
                    mult = mathOps.multiply(attr, factor, name=ctrl.name().replace('_ctrl', '_jaw_weight'))

                    if prefix == 'upper':
                        pb = mathOps.pairBlend(translateA=targPos, translateB=jawMp.allCoordinates, quatBlend=1,
                                               rotateA=targRot, rotateB=jawMp.rotate, weight=mult.output,
                                               name=self.getName('%s_%s_jaw_blend' % (prefix, num)))
                    else:
                        pb = mathOps.pairBlend(translateA=jawMp.allCoordinates, translateB=targPos, quatBlend=1,
                                               rotateA=jawMp.rotate, rotateB=targRot, weight=mult.output,
                                               name=self.getName('%s_%s_jaw_blend' % (prefix, num)))
                elif index == 0 and prefix == 'upper':
                    pb = mathOps.pairBlend(translateA=targPos, translateB=jawMp.allCoordinates, quatBlend=1,
                                           rotateA=targRot, rotateB=jawMp.rotate, weight=self.params.start_jaw_follow,
                                           name=self.getName('%s_%s_jaw_blend' % (prefix, num)))
                elif index == len(self.upper_ctrls)-1 and prefix == 'upper':
                    pb = mathOps.pairBlend(translateA=targPos, translateB=jawMp.allCoordinates, quatBlend=1,
                                           rotateA=targRot, rotateB=jawMp.rotate, weight=self.params.end_jaw_follow,
                                           name=self.getName('%s_%s_jaw_blend' % (prefix, num)))
                targPos = pb.outTranslate
                targRot = pb.outRotate
            crvMtx = mathOps.createComposeMatrix(targPos, targRot, baseMtx2Srt.outputScale,
                                                 name=self.getName('%s_%s_crv_mtx' % (prefix, num)))
            inverseMtx = crvMtx.outputMatrix.get().inverse()
            offset = ctrl.worldMatrix[0].get() * inverseMtx
            offsetMtx = mathOps.createComposeMatrix(inputTranslate=offset.translate.get(),
                                                    inputRotate=offset.rotate.get(),
                                                    name=self.getName('%s_%s_offset_mtx' % (prefix, num)))
            if index < midIndex:
                offsetMtx.inputScaleX.set(-1)
            ctrlMtx = mathOps.multiplyMatrices([offsetMtx.outputMatrix, crvMtx.outputMatrix],
                                               name=self.getName('%s_%s_ctrl_mtx' % (prefix, num)))
            ctrlMtx.matrixSum.connect(ctrl.getParent().offsetParentMatrix)

        for index, ctrl in enumerate(self.upper_ctrls):
            _driveCtrl(index, ctrl)
            if 0 < index <= len(self.lower_ctrls):
                pass
                _driveCtrl(index, self.lower_ctrls[index-1], prefix='lower')

        # drive curves
        d = mathOps.decomposeMatrix(self.upper_ctrls[0].worldMatrix[0], name=self.getName('upper_01_ctrl_mtx2Srt'))
        d.outputTranslate.connect(self.upper_crv.controlPoints[0])
        d.outputTranslate.connect(self.lower_crv.controlPoints[0])

        d = mathOps.decomposeMatrix(self.upper_ctrls[-1].worldMatrix[0],
                                    name=self.getName('upper_%s_ctrl_mtx2Srt') % len(self.upper_ctrls))
        d.outputTranslate.connect(self.upper_crv.controlPoints[len(self.upper_ctrls)-1])
        d.outputTranslate.connect(self.lower_crv.controlPoints[len(self.upper_ctrls)-1])

        # blending between upper and lower points for lip sealing
        startSealRange = mathOps.subtractScalar([1, self.params.seal_falloff], name=self.getName('start_seal_range'))
        numPoints = len(self.upper_ctrls) + len(self.lower_ctrls)
        for index in range(len(self.upper_ctrls)-2):
            num = str(index+1).zfill(2)
            upperCtrl = self.upper_ctrls[index+1]
            lowerCtrl = self.lower_ctrls[index]
            upperD = mathOps.decomposeMatrix(upperCtrl.worldMatrix[0],
                                             name=upperCtrl.name().replace('_ctrl', '_mtx2Srt'))
            lowerD = mathOps.decomposeMatrix(lowerCtrl.worldMatrix[0],
                                             name=lowerCtrl.name().replace('_ctrl', '_mtx2Srt'))

            # Find distance from start and end
            startDist = mathOps.multiply(startSealRange.output1D, params[index+1],
                                         name=self.getName('seal_start_%s_dist' % num))
            endDist = mathOps.reverse(startDist.output, name=self.getName('seal_end_%s_dist' % num))

            startRange = mathOps.addScalar([startDist.output, self.params.seal_falloff],
                                           name=self.getName('start_seal_%s_range' % num))
            startBlendRemap = mathOps.remap(self.params.start_seal, startDist.output, startRange.output1D, 0, 1,
                                            name=self.getName('seal_start_%s_remap' % num))
            startBlendEase = anim.easeCurve(startBlendRemap.outValueX, name=self.getName('seal_start_%s_ease' % num))

            endRange = mathOps.subtractScalar([endDist.outputX, self.params.seal_falloff],
                                              name=self.getName('end_seal_%s_range' % num))
            endBlendRemap = mathOps.remap(self.params.end_seal, endRange.output1D, endDist.outputX, 0, 1,
                                          name=self.getName('seal_end_%s_remap' % num))
            endBlendEase = anim.easeCurve(endBlendRemap.outValueX, name=self.getName('seal_end_%s_ease' % num))

            blendSum = mathOps.addScalar([startBlendEase.output, endBlendEase.output],
                                         name=self.getName('seal_blend_%s_sum' % num))
            blendClamp = mathOps.clamp(blendSum.output1D, 0, 1, name=self.getName('seal_blend_%s_clamp' % num))

            sealBlend = mathOps.pairBlend(translateA=lowerD.outputTranslate,
                                          translateB=upperD.outputTranslate, weight=self.params.seal_height,
                                          name=self.getName('seal_start_%s_pos' % num))

            upperBlend = mathOps.pairBlend(translateA=upperD.outputTranslate,
                                           translateB=sealBlend.outTranslate, weight=blendClamp.outputR,
                                           name=self.getName('upper_seal_%s_pos' % num))
            upperBlend.outTranslate.connect(self.upper_crv.controlPoints[index+1])

            lowerBlend = mathOps.pairBlend(translateA=lowerD.outputTranslate,
                                           translateB=sealBlend.outTranslate, weight=blendClamp.outputR,
                                           name=self.getName('lower_seal_%s_pos' % num))
            lowerBlend.outTranslate.connect(self.lower_crv.controlPoints[index+1])


        # Calculate mid param relative to start and end for upper and lower. Used to anchor mid points of divs

        upperStartList = []
        upperEndList = []
        lowerStartList = []
        lowerEndList = []
        lowerTempCtrls = [self.upper_ctrls[0]] + self.lower_ctrls + [self.upper_ctrls[-1]]

        for index in range(len(self.upper_ctrls)/2):
            num = str(index+1).zfill(2)
            upperStartDist = mathOps.distance(self.upper_ctrls[index], self.upper_ctrls[index+1],
                                              name=self.getName('upper_start_%s_dist' % num))
            upperStartList.append(upperStartDist)

            lowerStartDist = mathOps.distance(lowerTempCtrls[index], lowerTempCtrls[index+1],
                                              name=self.getName('lower_start_%s_dist' % num))
            lowerStartList.append(lowerStartDist)

            upperEndDist = mathOps.distance(self.upper_ctrls[index+midIndex], self.upper_ctrls[index+1+midIndex],
                                            name=self.getName('upper_end_%s_dist' % num))
            upperEndList.append(upperEndDist)

            lowerEndDist = mathOps.distance(lowerTempCtrls[index+midIndex], lowerTempCtrls[index+1+midIndex],
                                            name=self.getName('lower_end_%s_dist' % num))
            lowerEndList.append(lowerEndDist)

        upperStartLen = mathOps.addScalar([node.distance for node in upperStartList],
                                          name=self.getName('upper_start_len'))
        lowerStartLen = mathOps.addScalar([node.distance for node in lowerStartList],
                                          name=self.getName('upper_start_len'))
        upperEndLen = mathOps.addScalar([node.distance for node in upperEndList],
                                        name=self.getName('upper_end_len'))
        lowerEndLen = mathOps.addScalar([node.distance for node in lowerEndList],
                                        name=self.getName('upper_end_len'))
        upperLen = mathOps.addScalar([upperStartLen.output1D, upperEndLen.output1D],
                                     name=self.getName('upper_len'))
        halfUpperLen = mathOps.multiply(upperLen.output1D, 0.5, name=self.getName('half_upper_len'))
        lowerLen = mathOps.addScalar([lowerStartLen.output1D, lowerEndLen.output1D],
                                     name=self.getName('lower_len'))
        halfLowerLen = mathOps.multiply(lowerLen.output1D, 0.5, name=self.getName('half_lower_len'))

        upperStartMult = mathOps.divide(upperStartLen.output1D, halfUpperLen.output, name=self.getName('upper_start_bias'))
        lowerStartMult = mathOps.divide(lowerStartLen.output1D, halfLowerLen.output, name=self.getName('lower_start_bias'))

        upperEndMult = mathOps.subtractScalar([2.0, upperStartMult.outputX], name=self.getName('upper_end_bias'))
        lowerEndMult = mathOps.subtractScalar([2.0, lowerStartMult.outputX], name=self.getName('lower_end_bias'))

        upperCrvLen = curve.createCurveLength(self.upper_crv, name=self.getName('upper_crv_len'))
        lowerCrvLen = curve.createCurveLength(self.lower_crv, name=self.getName('lower_crv_len'))

        upperRestLen = mathOps.multiply(upperCrvLen.arcLength.get(), baseMtx2Srt.outputScaleX,
                                        name=self.getName('upper_rest_len'))
        upperStretch = mathOps.divide(upperRestLen.output, upperCrvLen.arcLength,
                                      name=self.getName('upper_stretch'))

        lowerRestLen = mathOps.multiply(lowerCrvLen.arcLength.get(), baseMtx2Srt.outputScaleX,
                                        name=self.getName('lower_rest_len'))
        lowerStretch = mathOps.divide(lowerRestLen.output, lowerCrvLen.arcLength,
                                      name=self.getName('lower_stretch'))

        # Drive Divs
        div_mps = []
        def _driveDiv(div, index, prefix='upper'):
            num = str(index+1).zfill(2)
            param = (1.0 / (len(self.upperDivs)-1))*index
            paramMirror = 0.5-(math.fabs(param-0.5))
            crv = self.upper_crv
            startParam = upperStartMult
            endParam = upperEndMult
            if prefix != 'upper':
                crv = self.lower_crv
                startParam = lowerStartMult
                endParam = lowerEndMult
            if param <= 0.5:
                paramMult = mathOps.multiply(startParam.outputX, paramMirror,
                                             name=self.getName('%s_div_%s_param_mult' % (prefix, num)))
                paramAttr = paramMult.output
            else:
                paramMult = mathOps.multiply(endParam.output1D, paramMirror,
                                             name=self.getName('%s_div_%s_param_mult' % (prefix, num)))
                paramRev = mathOps.reverse(paramMult.output, name=self.getName('%s_div_%s_param_rev' % (prefix, num)))
                paramAttr = paramRev.outputX
            paramBlend = mathOps.blendScalarAttrs(param, paramAttr, self.params.anchor_mid,
                                                  name=self.getName('%s_div_%s_param_blend' % (prefix, num)))

            mp = curve.createMotionPathNode(crv, uValue=paramBlend.output, follow=0,
                                            name=self.getName('%s_div_%s_mp' % (prefix, num)))

            mp.allCoordinates.connect(div.t)

            basePoint = curve.createNearestPointOnCurve(self.base_crv, mp.allCoordinates,
                                                       name=self.getName('%s_div_%s_base_point' % (prefix, num)))
            baseMp = curve.createMotionPathNode(self.base_crv, uValue=basePoint.result.parameter,
                                                wut=2, wuo=self.base_srt, fractionMode=0,
                                                upAxis='y', name=self.getName('%s_div_%s_base_mp' % (prefix, num)))
            targRot = baseMp.rotate
            if self.jaw:
                jawPoint = curve.createNearestPointOnCurve(self.jaw_crv, mp.allCoordinates,
                                                           name=self.getName('%s_div_%s_jaw_point' % (prefix, num)))
                jawMp = curve.createMotionPathNode(self.jaw_crv, uValue=paramBlend.output,
                                                   wut=2, wuo=self.jaw_srt, fractionMode=0,
                                                   upAxis='y', name=self.getName('%s_div_%s_jaw_mp' % (prefix, num)))
                if param <= 0.5:
                    if prefix == 'upper':
                        weight = 1-(param*2)
                        attr = self.params.start_jaw_follow
                    else:
                        weight = param*2
                        attr = startFollowRev.outputX
                    weightMult = mathOps.blendScalarAttrs(attr, 1.0, weight,
                                                  name=self.getName('%s_div_%s_jaw_weight' % (prefix, num)))
                else:
                    if prefix == 'upper':
                        weight = (param-0.5)*2
                        attr = self.params.end_jaw_follow
                    else:
                        weight = 1-((param-0.5)*2)
                        attr = endFollowRev.outputX
                    weightMult = mathOps.blendScalarAttrs(attr, 1.0, weight,
                                                  name=self.getName('%s_div_%s_jaw_weight' % (prefix, num)))
                pb = mathOps.pairBlend(rotateA=baseMp.rotate, rotateB=jawMp.rotate, weight=weightMult.output, quatBlend=1,
                                       name=self.getName('%s_div_%s_jaw_blend' % (prefix, num)))
                targRot = pb.outRotate
            targRot.connect(div.r)

            baseMtx2Srt.outputScale.connect(div.s)

        if self.guide.local_rig:
            self.base_crv.worldInverseMatrix[0].connect(self.local_srt.offsetParentMatrix)


        for index, div in enumerate(self.upperDivs):
            _driveDiv(div, index)

        for index in range(1, len(self.lowerDivs)+1):
            _driveDiv(self.lowerDivs[index-1], index, prefix='lower')

        # Attach params shape to base srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.base_srt, lw=1, wt=1)
        pm.delete(tempJoint)

    def setColours(self):
        '''
        Sets the drawing override settings on controls and the outliner colour for the component root
        Returns:
            None
        '''
        colour = pm.Attribute('guide.centre_colour').get()
        if self.comp_side == 'R':
            colour = pm.Attribute('guide.right_colour').get()
        elif self.comp_side == 'L':
            colour = pm.Attribute('guide.left_colour').get()
        dag.setOutlinerColour(self.root, colour)
        icon.setColourRGB(self.params, colour)

        for index, node in enumerate(self.upper_ctrls):
            colour = pm.Attribute('guide.centre_colour').get()
            if index < len(self.upper_ctrls)/2:
                colour = pm.Attribute('guide.right_colour').get()
            elif index > (len(self.upper_ctrls)/2):
                colour = pm.Attribute('guide.left_colour').get()
            icon.setColourRGB(node, colour)

        for index, node in enumerate(self.lower_ctrls):
            colour = pm.Attribute('guide.centre_colour').get()
            if index < len(self.lower_ctrls)/2:
                colour = pm.Attribute('guide.right_colour').get()
            elif index > (len(self.lower_ctrls)/2):
                colour = pm.Attribute('guide.left_colour').get()
            icon.setColourRGB(node, colour)

    def finish(self):
        self.setColours()

        # Lock non-keyable attrs
        nodeList = self.controls_list
        attrList = ['visibility', 'sx', 'sy', 'sz', 'rx', 'ry', 'rz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TMouth03(guide)
