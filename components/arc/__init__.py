from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps, curve
import pymel.core as pm
import math
reload(components)
reload(transform)

import pymel.core as pm

class TArc(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'arc')
        print 'Created Arc Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = (self.guide.guide_side == 'R')
        xform = pm.PyNode(guide.locs[1]).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        ctrlSize = mathOps.getDistance(guide.locs[1], guide.locs[-1])
        self.start_ctrl = self.addCtrl(shape='box', size=ctrlSize*.2, name=self.getName('start'), xform=xform,
                                       parent=self.base_srt, metaParent=self.base_srt)

        xform = pm.PyNode(guide.locs[2]).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        self.mid_ctrl = self.addCtrl(shape='box', size=ctrlSize * .125, name=self.getName('mid'), xform=xform,
                                     parent=self.base_srt, metaParent=self.start_ctrl, buffer=1)

        xform = pm.PyNode(guide.locs[-1]).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        self.end_ctrl = self.addCtrl(shape='box', size=ctrlSize * .2, name=self.getName('end'), xform=xform,
                                     parent=self.base_srt, metaParent=self.mid_ctrl)

        initPoints = [(i, 0, 0) for i in range(4)]
        self.crv = curve.curveThroughPoints(name=self.getName('crv'), positions=initPoints, degree=2)
        self.crv.setParent(self.rig)

        self.railCrv = curve.curveThroughPoints(name=self.getName('rail'), positions=initPoints, degree=2)
        self.railCrv.setParent(self.rig)

        self.divs = []
        for index in range(guide.num_divisions):
            num = str(index+1).zfill(2)
            div = dag.addChild(self.rig, 'group', name=self.getName('%s_srt' % num))
            self.divs.append(div)

            if guide.root.add_joint.get():
                j = pm.createNode('joint', name=self.getName('%s_jnt' % num))
                self.joints_list.append({'joint': j, 'driver': div})
                self.mapJointToGuideLocs(j, guide.root)
        components.TBaseComponent.addObjects(self, guide)

    def addAttributes(self):
        attribute.addAngleAttr(self.params, 'start_twist')
        attribute.addAngleAttr(self.params, 'mid_twist')
        attribute.addAngleAttr(self.params, 'end_twist')

    def addSystems(self):
        self.mid_ctrl.getParent().inheritsTransform.set(0)
        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
        startMtx2Srt = mathOps.decomposeMatrix(self.start_ctrl.worldMatrix[0], name=self.getName('start_mtx2Srt'))
        midMtx2Srt = mathOps.decomposeMatrix(self.mid_ctrl.worldMatrix[0], name=self.getName('mid_mtx2Srt'))
        endMtx2Srt = mathOps.decomposeMatrix(self.end_ctrl.worldMatrix[0], name=self.getName('end_mtx2Srt'))

        midPosMtx = transform.blendMatrices(self.start_ctrl.worldMatrix[0], self.end_ctrl.worldMatrix[0],
                                            name=self.getName('mid_pos_mtx'))
        midMtx = transform.blend_T_R_matrices(midPosMtx.outputMatrix, self.base_srt.worldMatrix[0])
        midMtx.outputMatrix.connect(self.mid_ctrl.getParent().offsetParentMatrix)

        transform.align(self.mid_ctrl, self.guide.locs[2])
        transform.bakeSrtToOffsetParentMtx(self.mid_ctrl)

        midBlendMtx = transform.blend_T_R_matrices(self.mid_ctrl.worldMatrix[0],
                                                  self.mid_ctrl.getParent().worldMatrix[0],
                                                  name=self.getName('mid_blend_mtx'))
        midRefMtx = mathOps.inverseMatrix(midBlendMtx.outputMatrix, name=self.getName('mid_ref_mtx'))
        midInWorldPos = mathOps.pairBlend(startMtx2Srt.outputTranslate, midMtx2Srt.outputTranslate, weight=0.5,
                                   name=self.getName('mid_in_pos_blend'))
        midOutWorldPos = mathOps.pairBlend(midMtx2Srt.outputTranslate, endMtx2Srt.outputTranslate, weight=0.5,
                                           name=self.getName('mid_out_pos_blend'))
        midInLocalPos = mathOps.createTransformedPoint(midInWorldPos.outTranslate, midRefMtx.outputMatrix,
                                                       name=self.getName('mid_in_local_pos'))
        midOutLocalPos = mathOps.createTransformedPoint(midOutWorldPos.outTranslate, midRefMtx.outputMatrix,
                                                       name=self.getName('mid_out_local_pos'))
        mid_in_pos = mathOps.createTransformedPoint(midInLocalPos.output, self.mid_ctrl.worldMatrix[0],
                                                    name=self.getName('mid_in_pos'))
        mid_out_pos = mathOps.createTransformedPoint(midOutLocalPos.output, self.mid_ctrl.worldMatrix[0],
                                                    name=self.getName('mid_out_pos'))

        offsetVec = (0, 0, 1)
        if self.invert:
            offsetVec = (0, 0, -1)
        railStartPos = mathOps.createTransformedPoint(offsetVec, self.start_ctrl.worldMatrix[0],
                                                      name=self.getName('start_rail_pos'))
        railEndPos = mathOps.createTransformedPoint(offsetVec, self.end_ctrl.worldMatrix[0],
                                                    name=self.getName('end_rail_pos'))
        railInLocalPos = mathOps.addVector([midInLocalPos.output, (0, 0, 1)], name=self.getName('rail_in_local_pos'))
        railOutLocalPos = mathOps.addVector([midOutLocalPos.output, (0, 0, 1)], name=self.getName('rail_out_local_pos'))
        railInPos = mathOps.createTransformedPoint(railInLocalPos.output3D, self.mid_ctrl.worldMatrix[0],
                                                   name=self.getName('rail_in_pos'))
        railOutPos = mathOps.createTransformedPoint(railOutLocalPos.output3D, self.mid_ctrl.worldMatrix[0],
                                                    name=self.getName('rail_out_pos'))

        startMtx2Srt.outputTranslate.connect(self.crv.controlPoints[0])
        mid_in_pos.output.connect(self.crv.controlPoints[1])
        mid_out_pos.output.connect(self.crv.controlPoints[2])
        endMtx2Srt.outputTranslate.connect(self.crv.controlPoints[3])

        railStartPos.output.connect(self.railCrv.controlPoints[0])
        railInPos.output.connect(self.railCrv.controlPoints[1])
        railOutPos.output.connect(self.railCrv.controlPoints[2])
        railEndPos.output.connect(self.railCrv.controlPoints[3])

        for index, div in enumerate(self.divs):
            num = str(index+1).zfill(2)
            param = (1.0 / (self.guide.num_divisions - 1)) * index
            mp = curve.createMotionPathNode(self.crv, uValue=param, frontAxis='x', upAxis='z', wut=1,
                                            name=self.getName('%s_mp' % num))
            railMp = curve.createMotionPathNode(self.railCrv, uValue=param, follow=0,
                                                name=self.getName('%s_rail_mp' % num))
            railMtx = mathOps.createComposeMatrix(inputTranslate = railMp.allCoordinates,
                                                  name=self.getName('%s_rail_mtx' % num))
            railMtx.outputMatrix.connect(mp.worldUpMatrix)

            mp.allCoordinates.connect(div.t)
            mp.rotate.connect(div.r)
            baseMtx2Srt.outputScale.connect(div.s)

            endsTwistSum = mathOps.addAngles(self.params.start_twist, self.params.end_twist,
                                             name=self.getName('ends_twist_%s_sum' % num))
            twistSum = mathOps.addAngles(self.params.mid_twist, endsTwistSum.output,
                                         name=self.getName('twist_%s_sum' % num))
            startMult = 1 - param
            endMult = param
            midMult = (0.5 - (math.fabs(0.5-param)))*2
            if self.invert:
                startMult *=-1
                midMult *=-1
                endMult *=-1
            endsTwistSum.weightA.set(startMult)
            endsTwistSum.weightB.set(endMult)
            twistSum.weightA.set(midMult)
            twistSum.output.connect(mp.frontTwist)

    def finish(self):
        self.setColours(self.guide)

        attribute.channelControl(nodeList=self.controls_list, attrList=['rotateOrder'], keyable=1, lock=0)

        attrList = ['visibility', 'sx', 'sy', 'sz']
        attribute.channelControl(nodeList=self.controls_list, attrList=attrList)

        attrList = ['start_twist', 'mid_twist', 'end_twist']
        for attr in attrList:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.mid_ctrl)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TArc(guide)

