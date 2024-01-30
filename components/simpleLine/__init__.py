from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon, curve
reload(components)
import math

import pymel.core as pm

class TSimpleLine(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = (self.guide.guide_side == 'R')
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'simpleLine')
        print 'Created Simple Line Component: %s' % self.comp_name

    def addObjects(self, guide):
        transform.align(self.base_srt, guide.root)
        locs = guide.locs[1:]

        self.srts = []

        ctrlSize = mathOps.getDistance(guide.locs[1], guide.locs[-1])
        parent = self.base_srt

        # Curve
        points = [(i, 0, 0) for i in range(guide.num_segments)]
        self.crv = curve.curveThroughPoints(self.getName('crv'), points, 3)
        self.crv.setParent(self.rig)

        # Main ctrl
        xform = transform.list2Mtx(pm.xform(self.base_srt, q=1, m=1, ws=1))
        if self.invert:
            xform = mathOps.getInverseHandedMatrix(xform)
        self.mainCtrl = self.addCtrl(shape='box', size=ctrlSize,
                                     name=self.getName('main'),
                                     xform=xform, parent=parent, metaParent=parent)
        metaParent = self.mainCtrl

        # Segment controls
        for index, loc in enumerate(locs):
            segNum = str(index+1).zfill(2)
            xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
            if self.invert:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='box', size=ctrlSize*.15,
                                name=self.getName('%s' % segNum),
                                xform=xform, parent=self.mainCtrl, metaParent=metaParent)
            metaParent = ctrl
            self.mapToGuideLocs(ctrl, loc)

        # SRTs
        for index in range(guide.num_joints):
            num = str(index+1).zfill(2)
            srt = dag.addChild(self.rig, 'group', name=self.getName('%s_srt' % num))
            self.srts.append(srt)

        # Joints
        if guide.root.add_joint.get():
            for srt in self.srts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                if srt != self.srts[0]:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': srt})

        # Map to guide locs
        self.mapToGuideLocs(self.controls_list[1], guide.locs[0])

    def addAttributes(self):
        for ctrl in self.controls_list[2:]:
            attr = attribute.addFloatAttr(ctrl, 'thickness')
            attr.set(1.0)

    def addSystems(self):
        mtx = self.mainCtrl.worldMatrix[0]
        if self.invert:
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
            mtx = transform.multiplyMatrices([negMtx.outputMatrix, mtx], name=self.getName('baseMtxNeg')).matrixSum

        baseMtx2Srt = mathOps.decomposeMatrix(mtx, name=self.getName('baseMtx2Srt'))

        # Attach curve to controls
        for index, ctrl in enumerate(self.controls_list[2:]):
            d = mathOps.decomposeMatrix(ctrl.worldMatrix[0], name=ctrl.name() + '_mtx2Srt')
            d.outputTranslate.connect(self.crv.controlPoints[index])

        # attach srts to curve
        ctrlParams = [(1.0/(self.guide.num_segments - 1)) * i for i in range(self.guide.num_segments)]
        midPoint = (1.0/(self.guide.num_segments - 1)) * 0.5
        print self.controls_list
        for index, srt in enumerate(self.srts):
            num = str(index + 1).zfill(2)
            param = (1.0 / (len(self.srts)-1)) * index

            mp = curve.createMotionPathNode(self.crv, uValue=param, frontAxis='x', upAxis='z', fractionMode=1,
                         follow=1, wut=2, name=self.getName('%s_mp' % num), wuo=self.mainCtrl, wu=(0, 0, 1))
            mp.allCoordinates.connect(srt.t)
            mp.rotate.connect(srt.r)

            # Figure out which two controls to interpolate against for thickness
            ctrls = []
            blendVal = 0
            for i, p in enumerate(ctrlParams):
                dist = p - param
                if  math.fabs(dist) <= midPoint:
                    if dist <= 0:
                        ctrls = [self.controls_list[i+2], self.controls_list[min(i+3, len(self.controls_list)-1)]]
                        blendVal = math.fabs(dist)/(midPoint*2)
                        print 'positive', i
                        print srt.name(), ctrls
                        continue
                    else:
                        ctrls = [self.controls_list[i + 1], self.controls_list[i + 2]]
                        blendVal = 1 - (math.fabs(dist) / (midPoint * 2))
                        print 'negative', i
                        print srt.name(), ctrls
                        continue
            thicknessBlend = mathOps.blendScalarAttrs(ctrls[0].thickness, ctrls[1].thickness, blend=blendVal,
                                                      name=self.getName('%s_thickness_blend' % num))
            thickNessScaled = mathOps.multiply(thicknessBlend.output, baseMtx2Srt.outputScaleX,
                                               name=self.getName('%s_thickness_scaled' % num))
            thickNessScaled.output.connect(srt.sy)
            thickNessScaled.output.connect(srt.sz)
            baseMtx2Srt.outputScaleX.connect(srt.sx)


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
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSimpleLine(guide)
