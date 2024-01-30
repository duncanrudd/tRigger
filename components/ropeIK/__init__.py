from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon, curve, anim
reload(components)

import pymel.core as pm

class TRopeIK(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = (self.guide.guide_side == 'R')
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'ropeIK')
        print 'Created Rope IK Component: %s' % self.comp_name

    def addObjects(self, guide):
        transform.align(self.base_srt, guide.root)
        locs = guide.locs[1:]
        locs.append(guide.root)
        locs.reverse()

        ctrlSize = mathOps.getDistance(guide.locs[0], guide.locs[-1])
        parent = self.base_srt
        self.srts = []

        # start_ctrl
        xform = transform.list2Mtx(pm.xform(guide.root, q=1, m=1, ws=1))
        if self.invert:
            xform = mathOps.getInverseHandedMatrix(xform)
        self.start_ctrl = self.addCtrl(shape='ball', size=ctrlSize*.15,
                                       name=self.getName('start'),
                                       xform=xform, parent=self.base_srt, metaParent=self.base_srt)
        self.mapToGuideLocs(self.start_ctrl, guide.root)

        # start tangent
        xform = transform.list2Mtx(pm.xform(guide.locs[2], q=1, m=1, ws=1))
        if self.invert:
            xform = mathOps.getInverseHandedMatrix(xform)
        self.startTangent_ctrl = self.addCtrl(shape='ball', size=ctrlSize * .075,
                                              name=self.getName('startTangent'),
                                              xform=xform, parent=self.start_ctrl, metaParent=self.start_ctrl)
        self.mapToGuideLocs(self.startTangent_ctrl, guide.locs[1])

        # end_ctrl
        xform = transform.list2Mtx(pm.xform(guide.locs[4], q=1, m=1, ws=1))
        if self.invert:
            xform = mathOps.getInverseHandedMatrix(xform)
        self.end_ctrl = self.addCtrl(shape='ball', size=ctrlSize * .15,
                                     name=self.getName('end'),
                                     xform=xform, parent=self.base_srt, metaParent=self.base_srt)
        self.mapToGuideLocs(self.start_ctrl, guide.locs[3])

        # end_tangent_ctrl
        xform = transform.list2Mtx(pm.xform(guide.locs[3], q=1, m=1, ws=1))
        if self.invert:
            xform = mathOps.getInverseHandedMatrix(xform)
        self.endTangent_ctrl = self.addCtrl(shape='ball', size=ctrlSize * .075,
                                            name=self.getName('endTangent'),
                                            xform=xform, parent=self.end_ctrl, metaParent=self.end_ctrl)
        self.mapToGuideLocs(self.start_ctrl, guide.locs[2])
        self.mapToGuideLocs(self.end_ctrl, guide.locs[-1])

        # Add srts
        for index in range(guide.num_divisions):
            num = str(index + 1).zfill(2)
            srt = dag.addChild(self.rig, 'group', name=self.getName('%s_srt' % num))
            self.srts.append(srt)

        # add joints
        if guide.root.add_joint.get():
            for srt in self.srts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                if srt != self.srts[0]:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': srt})

        # add curves
        # main
        ctrls = [self.start_ctrl, self.startTangent_ctrl, self.endTangent_ctrl, self.end_ctrl]
        self.crv = curve.curveThroughPoints(self.getName('crv'), positions=ctrls, degree=2)
        curve.driveCurve(self.crv, ctrls)
        self.crv.setParent(self.rig)

        # rail
        self.switch = pm.createNode('condition', name=self.getName('upVec_switch'))
        self.switch.colorIfTrue.set((0, 0, -.01))
        self.switch.colorIfFalse.set((0, .01, 0))

        points = [mathOps.createTransformedPoint(self.switch.outColor, ctrl.worldMatrix[0],
                                                 name=ctrl.name().replace('_ctrl', '_rail_point')) for ctrl in ctrls]
        positions = [p.output.get() for p in points]
        self.rail = curve.curveThroughPoints(self.getName('rail'), positions=positions, degree=2)
        for i, p in enumerate(points):
            p.output.connect(self.rail.controlPoints[i])
        self.rail.setParent(self.rig)

        # add sub ctrls
        self.sub_ctrls = []
        self.sub_srts = []
        parent = self.start_ctrl
        for i in range(guide.num_segments):
            num = str(i+1).zfill(2)
            mtx = transform.list2Mtx(pm.xform(guide.root, q=1, m=1, ws=1))
            if self.invert:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.1,
                                name=self.getName('ik_%s' % num),
                                xform=xform, parent=self.controls, metaParent=parent)
            self.sub_ctrls.append(ctrl)
            parent = ctrl

            srt = dag.addChild(self.rig, 'group', name=self.getName('%s_sub_srt' % num))
            self.sub_srts.append(srt)

        # Add sub curves
        self.sub_crv = curve.curveThroughPoints(self.getName('sub_crv'), positions=self.sub_ctrls, degree=2)
        curve.driveCurve(self.sub_crv, self.sub_srts)
        self.sub_crv.setParent(self.rig)

        points = [mathOps.createTransformedPoint(self.switch.outColor, srt.worldMatrix[0],
                                                 name=srt.name().replace('_srt', '_rail_point')) for srt in self.sub_srts]
        positions = [p.output.get() for p in points]
        self.sub_rail = curve.curveThroughPoints(self.getName('sub_rail'), positions=positions, degree=2)
        for i, p in enumerate(points):
            p.output.connect(self.sub_rail.controlPoints[i])
        self.sub_rail.setParent(self.rig)

    def addAttributes(self):
        attribute.addEnumAttr(self.params, 'path_up_vector', ['side', 'top'])
        attribute.addFloatAttr(self.params, 'start_freeze', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'stretch', minValue=0.0, maxValue=1.0)
        for i, ctrl in enumerate(self.sub_ctrls):
            attribute.addFloatAttr(ctrl, 'position', minValue=0, maxValue=1)
            attribute.addFloatAttr(ctrl, 'falloff', minValue=0.1)
            ctrl.falloff.set((1.0 / (self.guide.num_segments-1))*1.5)

    def addSystems(self):
        d = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
        if self.invert:
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))

        switch = pm.createNode('condition', name=self.getName('path_switch'))
        switch.colorIfTrue.set((1, 0, 0))
        switch.colorIfFalse.set((2, 0, 0))
        self.params.path_up_vector.connect(self.switch.firstTerm)
        self.params.path_up_vector.connect(switch.firstTerm)

        # attach ctrls to curve
        for i, ctrl in enumerate(self.sub_ctrls):
            num = str(i + 1).zfill(2)
            param = (1.0 / (self.guide.num_segments - 1)) * i
            ctrl.position.set(param)

            # Attach to curve
            mp = pm.createNode('motionPath', name=self.getName('ctrl_%s_mp' % num))
            self.crv.worldSpace[0].connect(mp.geometryPath)
            ctrl.position.connect(mp.uValue)
            mp.frontAxis.set(0)
            switch.outColorR.connect(mp.upAxis)
            mp.fractionMode.set(1)

            posMp = pm.createNode('motionPath', name=self.getName('ctrl_%s_posMp' % num))
            self.crv.worldSpace[0].connect(posMp.geometryPath)
            ctrl.position.connect(posMp.uValue)
            posMp.fractionMode.set(1)

            railMp = pm.createNode('motionPath', name=self.getName('ctrl_%s_railMp' % num))
            self.rail.worldSpace[0].connect(railMp.geometryPath)
            ctrl.position.connect(railMp.uValue)
            railMp.follow.set(0)
            railMp.fractionMode.set(1)

            upVec = mathOps.subtractVector([railMp.allCoordinates, posMp.allCoordinates], name=self.getName('ctrl_%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            mtx = mathOps.createComposeMatrix(inputTranslate = mp.allCoordinates,
                                              inputRotate=mp.rotate, inputScale=d.outputScale,
                                              name=self.getName('ctrl_%s_subCtrl_mtx' % num))
            mtx.outputMatrix.connect(ctrl.offsetParentMatrix)

            srt = self.sub_srts[i]
            mtx.outputMatrix.connect(srt.offsetParentMatrix)

            # Local offsets
            yList = []
            zList = []
            twistList = []
            for j, ctrl in enumerate(self.sub_ctrls):
                num = str(j + 1).zfill(2)
                dist = pm.createNode('distanceBetween', name=srt.name().replace('srt', 'dist_%s' % num))
                ctrl.position.connect(dist.point1.point1Y)
                dist.point2.point2Y.set(param)
                scaledDist = mathOps.divide(dist.distance, ctrl.falloff,
                                            name=srt.name().replace('srt', 'scaledDist_%s' % num))
                remap = mathOps.remap(scaledDist.outputX, 0, 1, 1, 0,
                                      name=srt.name().replace('srt', 'remap_%s' % num))
                weight = anim.easeCurve(remap.outValueX, name=srt.name().replace('srt', 'weight_%s' % num))

                yMult = mathOps.multiply(weight.output, ctrl.ty)
                zMult = mathOps.multiply(weight.output, ctrl.tz)
                twistMult = mathOps.multiply(weight.output, ctrl.rx)

                yList.append(yMult.output)
                zList.append(zMult.output)
                twistList.append(twistMult.output)

            weight = mathOps.remap(param, 0.0, self.params.start_freeze, 0, 1,
                                   name=srt.name().replace('srt', 'offsetWeight'))
            ySum = mathOps.addScalar(yList, name=srt.name().replace('srt', 'xSum'))
            yMult = mathOps.multiply(ySum.output1D, weight.outValueX, name=srt.name().replace('srt', 'yMult'))
            zSum = mathOps.addScalar(zList, name=srt.name().replace('srt', 'zSum'))
            zMult = mathOps.multiply(zSum.output1D, weight.outValueX, name=srt.name().replace('srt', 'zMult'))
            twistSum = mathOps.addScalar(twistList, name=srt.name().replace('srt', 'twistSum'))
            twistMult = mathOps.multiply(twistSum.output1D, weight.outValueX,
                                         name=srt.name().replace('srt', 'twistMult'))

            yMult.output.connect(srt.ty)
            zMult.output.connect(srt.tz)
            twistMult.output.connect(srt.rx)


        # set up overflow
        tangent = curve.createPointOnCurve(self.sub_crv, name=self.getName('crv_end_tangent'))
        tangent.parameter.set(1.0)

        # attach srts to curve
        crvInfo = curve.createCurveLength(self.sub_crv, self.getName('crvInfo'))
        scaledLength = mathOps.multiply(crvInfo.arcLength.get(), d.outputScaleX, name=self.getName('scaled_crLength'))
        stretch = mathOps.divide(scaledLength.output, crvInfo.arcLength, name=self.getName('stretch'))

        for i, srt in enumerate(self.srts):
            num = str(i + 1).zfill(2)
            baseParam = (1.0 / (self.guide.num_divisions - 1)) * i
            clampedParam = mathOps.multiply(baseParam, stretch.outputX, name=self.getName('div_%s_param' % num))
            param = mathOps.blendScalarAttrs(clampedParam.output, baseParam, self.params.stretch, name=self.getName('stretch_%s_blend' % num))
            overflowParam = mathOps.remap(clampedParam.output, 1, 1001, 0, 1000, name=self.getName('div_%s_overflow_amount' % num))
            overflowLength = mathOps.multiply(crvInfo.arcLength, overflowParam.outValueX, name=self.getName('div_%s_overflow_length' % num))
            overflowVec = mathOps.multiplyVector(tangent.result.normalizedTangent, (1, 1, 1), name=self.getName('div_%s_overflow_vec' % num))
            overflowLength.output.connect(overflowVec.input2X)
            overflowLength.output.connect(overflowVec.input2Y)
            overflowLength.output.connect(overflowVec.input2Z)


            # Attach to curve
            mp = pm.createNode('motionPath', name=self.getName('div_%s_mp' % num))
            self.sub_crv.worldSpace[0].connect(mp.geometryPath)
            param.output.connect(mp.uValue)
            mp.frontAxis.set(0)
            switch.outColorR.connect(mp.upAxis)
            mp.fractionMode.set(1)

            posMp = pm.createNode('motionPath', name=self.getName('div_%s_posMp' % num))
            self.sub_crv.worldSpace[0].connect(posMp.geometryPath)
            param.output.connect(posMp.uValue)
            posMp.fractionMode.set(1)

            railMp = pm.createNode('motionPath', name=self.getName('div_%s_railMp' % num))
            self.sub_rail.worldSpace[0].connect(railMp.geometryPath)
            param.output.connect(railMp.uValue)
            railMp.follow.set(0)
            railMp.fractionMode.set(1)

            upVec = mathOps.subtractVector([railMp.allCoordinates, posMp.allCoordinates], name=self.getName('div_%s_upVec' % num))
            upVec.output3D.connect(mp.worldUpVector)

            pos = mathOps.addVector([overflowVec.output, mp.allCoordinates], name=self.getName('div_%s_pos' % num))
            mtx = mathOps.createComposeMatrix(inputTranslate = pos.output3D,
                                              inputRotate=mp.rotate, inputScale=d.outputScale,
                                              name=self.getName('div_%s_subCtrl_mtx' % num))
            mtx.outputMatrix.connect(srt.offsetParentMatrix)

        # Attach params shape to base srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.base_srt, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):
        self.setColours(self.guide)

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodes = [node for node in self.controls_list if not node == self.params]
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)
        attrList = ['tx', 'rz', 'ry', 'sx', 'sy', 'sz']
        attribute.channelControl(nodeList=self.sub_ctrls, attrList=attrList)


def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TRopeIK(guide)
