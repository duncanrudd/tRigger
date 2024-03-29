
from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform, curve, anim, icon
reload(dag)
reload(components)
reload(mathOps)
reload(curve)
reload(anim)

import pymel.core as pm

class TVariableFKBlend(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index,
                                           'variableFKBlend')
        print
        'Created VariableFKBlend Component: %s' % self.comp_name

    def addObjects(self, guide):
        ctrlSize = mathOps.getDistance(guide.locs[-2], guide.root) * .3

        def _getVec(start, end, invert=0):
            start, end = mathOps.getStartAndEnd(start, end)
            if invert:
                return (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
            else:
                return (pm.datatypes.Vector(start) - pm.datatypes.Vector(end)).normal()

        self.root_ctrl = self.addCtrl(shape='squarePoint', size=ctrlSize*2,
                                      name=self.getName('base'), xform=self.base_srt.worldMatrix[0].get(),
                                      parent=self.base_srt, metaParent=self.base_srt)

        initPoints = []
        step = 1.0 / (self.guide.num_divisions - 1)
        for i in range(self.guide.num_divisions):
            param = step * i
            initPoints.append(curve.sampleCurvePosition(self.guide.crv, param))

        self.crv = curve.curveThroughPoints(name=self.getName('crv'), positions=initPoints, degree=2)
        self.crv.setParent(self.rig)

        self.railCrv = curve.curveThroughPoints(name=self.getName('rail_crv'), positions=initPoints, degree=2)
        self.railCrv.setParent(self.rig)

        self.divs = []
        self.locs = []
        self.parentDivs = []
        self.parentLocs = []
        self.blends = []
        self.outSrts = []
        rootAxis = mathOps.getMatrixAxisAsVector(self.root_ctrl.worldMatrix[0].get(), 'x')
        checkVec = initPoints[1] - initPoints[0]
        checkVec.normal()
        if checkVec == rootAxis:
            rootAxis = mathOps.getMatrixAxisAsVector(self.root_ctrl.worldMatrix[0].get(), 'y')

        # Add locs at each sample point - these are what will drive the curve
        for i in range(self.guide.num_divisions):
            num = str(i+1).zfill(2)
            srt = pm.createNode('transform', name=self.getName('fk_%s_srt' % num))
            loc = dag.addChild(srt, 'locator', name=self.getName('twist_%s_srt' % num))
            outSrt = dag.addChild(self.rig, 'group', name=self.getName('out_%s_srt' % num))

            parentSrt = pm.createNode('transform', name=self.getName('fk_%s_parent_srt' % num))
            parentloc = dag.addChild(parentSrt, 'locator', name=self.getName('twist_%s_parent_srt' % num))

            if i < self.guide.num_divisions-1:
                aimVec = (initPoints[i+1] - initPoints[i]).normal()
            else:
                aimVec = (initPoints[i] - initPoints[i-1]).normal()
            sideVec = rootAxis.cross(aimVec).normal()
            upVec = sideVec.cross(aimVec).normal()
            startPos = initPoints[i]
            xform = pm.datatypes.Matrix(aimVec, upVec, sideVec, startPos)
            pm.xform(srt, ws=1, m=xform)
            pm.xform(parentSrt, ws=1, m=xform)
            if i == 0:
                srt.setParent(self.rig)
                parentSrt.setParent(self.rig)
                buffer = dag.addParent(srt, 'group', name=srt.name().replace('_srt', '_buffer_srt'))
                parentBuffer = dag.addParent(parentSrt, 'group', name=srt.name().replace('_srt', '_parentBuffer_srt'))
                self.parentBuffer = parentBuffer
                self.root_ctrl.worldMatrix[0].connect(buffer.offsetParentMatrix)
                buffer.t.set(0, 0, 0)
            else:
                srt.setParent(self.divs[-1])
                parentSrt.setParent(self.parentDivs[-1])

            # drive curves
            blend = transform.blendMatrices(loc.worldMatrix[0], parentloc.worldMatrix[0], name=self.getName('loc_%s_blend_mtx' % num))
            d = mathOps.decomposeMatrix(blend.outputMatrix, name=self.getName('%s_blend_mtx2Srt' % num))
            d.outputTranslate.connect(self.crv.controlPoints[i])
            railPos = mathOps.createTransformedPoint((0, 1, 0), blend.outputMatrix, name=self.getName('rail_%s_point' % num))
            railPos.output.connect(self.railCrv.controlPoints[i])
            self.divs.append(srt)
            self.locs.append(loc)
            self.parentDivs.append(parentSrt)
            self.parentLocs.append(parentloc)
            self.blends.append(blend)
            self.outSrts.append(outSrt)

        # FK controls
        self.fk_ctrls = []
        step =  1.0 / (self.guide.num_ctrls)
        for index in range(guide.num_ctrls):
            param = step * i
            num = str(index + 1).zfill(2)
            ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.5,
                                name=self.getName('fk_%s' % num), xform=pm.datatypes.Matrix(), parent=self.controls,
                                metaParent=self.controls_list[-1], buffer=1)

            neg = dag.addParent(ctrl, 'group', name=self.getName('fk_%s_negate_srt' % num))
            negRot = pm.createNode('animBlendNodeAdditiveRotation', name=self.getName('fk_%s_negRot' % num))
            negRot.weightA.set(-1)
            ctrl.r.connect(negRot.inputA)
            negRot.output.connect(neg.r)
            negTranslate = mathOps.multiplyVector(ctrl.t, (-1, -1, -1), name=self.getName('fk_%s_negPos' % num))
            negTranslate.output.connect(neg.t)

            self.fk_ctrls.append(ctrl)

        # MAP TO GUIDE LOCS
        mappingPairs = [[self.root_ctrl, guide.root], [self.parentBuffer, guide.locs[-1]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

        # Joints
        if guide.root.add_joint.get():
            for i, ctrl in enumerate(self.outSrts):
                num = str(i + 1).zfill(2)
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                if i > 0:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': ctrl})
            self.mapJointToGuideLocs(self.joints_list[-1]['joint'], self.guide.locs[3])

        # Sine wave stuff
        self.sinePlane = pm.polyPlane(name=self.getName('sine_mesh'))
        self.sinePlane[1].subdivisionsHeight.set(1)
        self.sinePlane[1].subdivisionsWidth.set(len(self.outSrts))
        self.sinePlane[0].setParent(self.rig)

        self.sineDef, self.sineHandle = pm.nonLinear(type='sine', name=self.getName('sine'))
        self.sineHandle.tx.set(-0.5)
        self.sineHandle.rz.set(-90)
        self.sineHandle.s.set((1, 1, 1))
        self.sineHandle.setParent(self.rig)
        self.sineDef.dropoff.set(-1)

        pin = pm.createNode('uvPin', name=self.getName('sine_pin'))
        self.sinePlane[0].worldMesh.connect(pin.deformedGeometry)
        step = 1.0 / (len(self.outSrts)-1)
        for index in range(len(self.outSrts)):
            param = step*index
            pin.coordinate[index].set((param, 0.5))
            d = mathOps.decomposeMatrix(pin.outputMatrix[index], name=self.getName('sine_%s_mtx2Srt' % num))
            d.outputTranslateY.connect(self.locs[index].tz)
            d.outputTranslateY.connect(self.parentLocs[index].tz)



    def addAttributes(self):
        step = 1.0 / (self.guide.num_ctrls)
        minStep = 1.0 / self.guide.num_divisions
        for i, ctrl in enumerate(self.fk_ctrls):
            param = step * i
            attribute.addFloatAttr(ctrl, 'position', minValue=-.5, maxValue=1.5, value=param)
            attribute.addFloatAttr(ctrl, 'falloff', minValue=minStep, maxValue=1.0, value=step)
        attribute.addAngleAttr(self.root_ctrl, 'twist')
        attribute.addFloatAttr(self.root_ctrl, 'parent_blend', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.root_ctrl, 'stretch', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.root_ctrl, 'sine_strength')
        attribute.addFloatAttr(self.root_ctrl, 'sine_scale', minValue=0.01)
        attribute.addFloatAttr(self.root_ctrl, 'sine_offset')


    def addSystems(self):

        # make sub curve to handle stretching on blended positions
        self.subCrv = pm.createNode('subCurve', name=self.getName('sub_crv'))
        self.crv.worldSpace[0].connect(self.subCrv.inputCurve)
        self.subRail = pm.createNode('subCurve', name=self.getName('sub_rail'))
        self.railCrv.worldSpace[0].connect(self.subRail.inputCurve)
        crvInfo = curve.createCurveLength(self.crv, name=self.getName('crv_info'))
        stretch = mathOps.divide(crvInfo.arcLength.get(), crvInfo.arcLength, name=self.getName('crv_stretch'))
        self.stretchBlend = mathOps.blendScalarAttrs(stretch.outputX, 1.0, self.root_ctrl.stretch, name=self.getName('stretch_blend'))
        self.stretchBlend.output.connect(self.subCrv.maxValue)
        self.stretchBlend.output.connect(self.subRail.maxValue)

        rootMtx2Srt = mathOps.decomposeMatrix(self.root_ctrl.worldMatrix[0], name=self.getName('root_ctrl_mtx2Srt'))
        step = 1.0 / (self.guide.num_ctrls)
        for i, ctrl in enumerate(self.fk_ctrls):
            buffer = ctrl.getParent().getParent()
            rootMtx2Srt.outputScale.connect(buffer.s)
            param = step * i
            num = str(i+1).zfill(2)
            mp = curve.createMotionPathNode(self.crv, uValue=param, frontAxis='x', upAxis='y',
                                            name=self.getName('%s_mp' % num))
            self.subCrv.outputCurve.connect(mp.geometryPath, f=1)
            railMp = curve.createMotionPathNode(self.railCrv, uValue=param, follow=0,
                                                name=self.getName('%s_rail_mp' % num))
            self.subRail.outputCurve.connect(railMp.geometryPath, f=1)
            upVec = mathOps.subtractVector([railMp.allCoordinates, mp.allCoordinates],
                                           name=self.getName('%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            mp.allCoordinates.connect(buffer.t)
            mp.rotate.connect(buffer.r)

            ctrl.position.connect(mp.uValue)
            ctrl.position.connect(railMp.uValue)

            buffer.offsetParentMatrix.set(pm.datatypes.Matrix())

        step = 1.0 / (len(self.outSrts)-1)

        # Connect joint drivers to sub curves
        self.mps = []
        self.railMps = []
        for i, outSrt in enumerate(self.outSrts):
            num = str(i+1).zfill(2)
            param = step * i

            mp = curve.createMotionPathNode(self.crv, uValue=param, frontAxis='x', upAxis='y',
                                            name=self.getName('out_%s_mp' % num))
            self.subCrv.outputCurve.connect(mp.geometryPath, f=1)

            railMp = curve.createMotionPathNode(self.railCrv, uValue=param, follow=0,
                                                name=self.getName('%s_rail_mp' % num))
            self.subRail.outputCurve.connect(railMp.geometryPath, f=1)
            upVec = mathOps.subtractVector([railMp.allCoordinates, mp.allCoordinates],
                                           name=self.getName('%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            mp.allCoordinates.connect(outSrt.t)
            mp.rotate.connect(outSrt.r)
            self.mps.append(mp)
            self.railMps.append(railMp)


        # Drive rotation of divs based on ctrls
        step = 1.0 / (self.guide.num_divisions -1)
        for i, div in enumerate(self.divs):
            divNum = str(i+1).zfill(2)
            transform.bakeSrtToOffsetParentMtx(div)
            transform.bakeSrtToOffsetParentMtx(self.parentDivs[i])
            rots = []
            param = step*i
            for x, ctrl in enumerate(self.fk_ctrls):
                ctrlNum = str(x + 1).zfill(2)
                rot = pm.createNode('animBlendNodeAdditiveRotation',
                                    name=self.getName('ctrl_%s_rot_%s_weight' % (ctrlNum, divNum)))
                ctrl.r.connect(rot.inputA)
                dist = pm.createNode('distanceBetween', name=self.getName('ctrl_%s_%s_dist' % (ctrlNum, divNum)))
                dist.point1X.set(param)
                ctrl.position.connect(dist.point2X)
                remap = mathOps.remap(dist.distance, 0, ctrl.falloff, 1, 0,
                                       name=self.getName('ctrl_%s_%s_remap' % (ctrlNum, divNum)))
                weight = anim.easeCurve(remap.outValueX, name=self.getName('ctrl_%s_%s_weight' % (ctrlNum, divNum)))
                weight.output.connect(rot.weightA)
                if rots:
                    rots[-1].output.connect(rot.inputB)
                rots.append(rot)
            twist = mathOps.addAngles(rots[-1].outputX, self.root_ctrl.twist,
                                      name=self.getName('twist_%s_sum' % divNum))
            twist.weightB.set(param)
            twist.output.connect(self.locs[i].rx)
            twist.output.connect(self.parentLocs[i].rx)
            rots[-1].outputY.connect(div.ry)
            rots[-1].outputY.connect(self.parentDivs[i].ry)
            rots[-1].outputZ.connect(div.rz)
            rots[-1].outputZ.connect(self.parentDivs[i].rz)

        # Attach params shape to base srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.base_srt, lw=1, wt=1)
        pm.delete(tempJoint)

        # set up blending to parent curve
        step = 1.0 / (len(self.blends)-1)
        for i, blend in enumerate(self.blends):
            num = str(i+1).zfill(2)
            max = step*i
            localBlend = mathOps.remap(self.root_ctrl.parent_blend, 0, 1, 0, max, name=self.getName('blend_%s_remap' % num))
            localBlend.outValueX.connect(blend.target[0].weight)
            #blend.outputMatrix.connect(self.outSrts[i].offsetParentMatrix)

        # Connect sine wave to attrs
        self.root_ctrl.sine_strength.connect(self.sineDef.amplitude)
        self.root_ctrl.sine_scale.connect(self.sineDef.wavelength)
        self.root_ctrl.sine_offset.connect(self.sineDef.offset)


    def finish(self):

        self.setColours(self.guide)

        # Lock non-keyable attrs
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = self.fk_ctrls
        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList, attrList)

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
    return TVariableFKBlend(guide)
