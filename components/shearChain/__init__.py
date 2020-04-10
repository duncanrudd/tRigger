from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon, curve
reload(components)

import pymel.core as pm

class TShearChain(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = (self.guide.guide_side == 'R')
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'shearChain')
        print 'Created FK Chain Component: %s' % self.comp_name

    def addObjects(self, guide):
        transform.align(self.base_srt, guide.root)
        locs = guide.locs[1:]
        locs.append(guide.root)
        locs.reverse()

        ctrlSize = mathOps.getDistance(guide.locs[0], guide.locs[-1])
        parent = self.base_srt
        self.inSrts = []
        self.outSrts = []
        self.fkCtrls = []
        self.ikCtrls = []
        for index, loc in enumerate(locs):
            segNum = str(index+1).zfill(2)
            xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
            if self.invert:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.25,
                                name=self.getName('%s_fk' % segNum),
                                xform=xform, parent=parent, metaParent=parent)
            self.fkCtrls.append(ctrl)
            if index == 0:
                ikParent = self.base_srt
            else:
                ikParent = self.ikCtrls[-1]
            ikCtrl = self.addCtrl(shape='box', size=ctrlSize * .1,
                                name=self.getName('%s_ik' % segNum), buffer=1,
                                xform=xform, parent=ctrl, metaParent=ikParent)
            self.ikCtrls.append(ikCtrl)
            parent = ctrl

            inSrt = dag.addChild(self.rig, 'group', name=self.getName('%s_in_srt' % segNum))
            self.inSrts.append(inSrt)

            outSrt = dag.addChild(self.rig, 'group', name=self.getName('%s_out_srt' % segNum))
            self.outSrts.append(outSrt)
            self.mapToGuideLocs(ikCtrl, loc)

        if guide.root.add_joint.get():
            for srt in self.inSrts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                self.joints_list.append({'joint': j, 'driver': srt})
            for srt in self.outSrts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                self.joints_list.append({'joint': j, 'driver': srt})

        # curve
        points = []
        for index, ctrl in enumerate(self.ikCtrls):
            num = str(index + 1).zfill(2)
            dm = mathOps.decomposeMatrix(ctrl.worldMatrix[0], name=self.getName('ik_%s_mtx2Srt' % num))
            points.append(dm.outputTranslate)

        initPoints = [(i, 0, 0) for i in range(len(self.ikCtrls))]
        self.crv = curve.curveThroughPoints(name=self.getName('crv'), positions=initPoints, degree=1, rebuild=0)
        self.crv.setParent(self.rig)
        for index, point in enumerate(points):
            point.connect(self.crv.controlPoints[index])

        # Rail
        railPoints = []
        for index, ctrl in enumerate(self.ikCtrls):
            num = str(index + 1).zfill(2)
            p = mathOps.createTransformedPoint([0, 0, 1], ctrl.worldMatrix[0], name=self.getName('ik_%s_mtx2Srt' % num))
            railPoints.append(p.output)

        self.railCrv = curve.curveThroughPoints(name=self.getName('ik_rail_crv'), positions=initPoints,
                                                degree=1, rebuild=0)
        self.railCrv.setParent(self.rig)
        for index, point in enumerate(railPoints):
            point.connect(self.railCrv.controlPoints[index])

    def addSystems(self):
        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))

        # Negate rotation of ik control buffers by 0.5
        for index, ctrl in enumerate(self.ikCtrls):
            num = str(index +1).zfill(2)
            buffer = ctrl.getParent()
            choice = transform.invertRotateOrder(self.fkCtrls[index], name=self.getName('rotateOrder_%s_invert' % num))
            choice.output.connect(buffer.rotateOrder)
            rotMult = mathOps.multiplyRotationByScalar(self.fkCtrls[index].r, -0.5,
                                                       name=self.getName('rotation_%s_mult' % num))
            self.fkCtrls[index].rotateOrder.connect(rotMult.rotateOrder)
            rotMult.output.connect(buffer.r)

        for index, ctrl in enumerate(self.ikCtrls):
            num = str(index+1).zfill(2)
            inParam = max([0, index-.01])
            outParam = min([len(self.ikCtrls)-1, index+.01])
            inMp = curve.createMotionPathNode(self.crv, uValue=inParam, upAxis='z', fractionMode=0, wu=(0, 0, 1),
                                              follow=1, wut=1, name=self.getName('joint_%s_in_mp' % num))
            inRailMp = curve.createMotionPathNode(self.railCrv, uValue=inParam, follow=0, fractionMode=0,
                                                  name=self.getName('joint_%s_in_railMp' % num))
            inRailMtx = mathOps.createComposeMatrix(inputTranslate=inRailMp.allCoordinates,
                                                    name=self.getName('joint_%s_in_rail_mtx' % num))
            inRailMtx.outputMatrix.connect(inMp.worldUpMatrix)

            outMp = curve.createMotionPathNode(self.crv, uValue=outParam, upAxis='z', fractionMode=0, wu=(0, 0, 1),
                                              follow=1, wut=1, name=self.getName('joint_%s_out_mp' % num))
            outRailMp = curve.createMotionPathNode(self.railCrv, uValue=outParam, follow=0, fractionMode=0,
                                                  name=self.getName('joint_%s_out_railMp' % num))
            outRailMtx = mathOps.createComposeMatrix(inputTranslate=outRailMp.allCoordinates,
                                                    name=self.getName('joint_%s_out_rail_mtx' % num))
            outRailMtx.outputMatrix.connect(outMp.worldUpMatrix)

            inMtx = mathOps.createComposeMatrix(inputRotate=inMp.rotate,
                                                name=self.getName('joint_%s_in_mtx' % num))
            outMtx = mathOps.createComposeMatrix(inputRotate=outMp.rotate,
                                                name=self.getName('joint_%s_out_mtx' % num))
            inInverseMtx = pm.createNode('inverseMatrix', name=self.getName('joint_%s_inverse_mtx' % num))
            inMtx.outputMatrix.connect(inInverseMtx.inputMatrix)
            localMtx = mathOps.multiplyMatrices([outMtx.outputMatrix, inInverseMtx.outputMatrix],
                                                name=self.getName('joint_%s_local_mtx' % num))
            localVec = mathOps.createMatrixAxisVector(localMtx.matrixSum, (1, 0, 0), name=self.getName('joint_%s_local_vec' % num))
            ang = mathOps.angleBetween((1, 0, 0), localVec.output)
            quatY = mathOps.isolateRotationOnAxis(ang.euler, axis='y', name=self.getName('joint_%s_rotY' % num))
            quatZ = mathOps.isolateRotationOnAxis(ang.euler, axis='z', name=self.getName('joint_%s_rotZ' % num))
            inYMult = mathOps.convert(quatY[1].outputRotateY, .75, name=self.getName('joint_%s_inYShear_mult' % num))
            inZMult = mathOps.convert(quatZ[1].outputRotateZ, -.75, name=self.getName('joint_%s_inZShear_mult' % num))
            inMp.allCoordinates.connect(self.inSrts[index].t)
            inMp.rotate.connect(self.inSrts[index].r)
            inYMult.output.connect(self.inSrts[index].shearXZ)
            inZMult.output.connect(self.inSrts[index].shearXY)

            outYMult = mathOps.convert(quatY[1].outputRotateY, -.75, name=self.getName('joint_%s_outYShear_mult' % num))
            outZMult = mathOps.convert(quatZ[1].outputRotateZ, .75, name=self.getName('joint_%s_outZShear_mult' % num))
            outMp.allCoordinates.connect(self.outSrts[index].t)
            outMp.rotate.connect(self.outSrts[index].r)
            outYMult.output.connect(self.outSrts[index].shearXZ)
            outZMult.output.connect(self.outSrts[index].shearXY)

            baseMtx2Srt.outputScale.connect(self.inSrts[index].s)
            baseMtx2Srt.outputScale.connect(self.outSrts[index].s)



    def finish(self):
        self.setColours(self.guide)

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodeList = self.controls_list
        attrList = ['visibility', 'sx', 'sy', 'sz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TShearChain(guide)
