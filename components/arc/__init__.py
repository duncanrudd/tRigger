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
                                     parent=self.controls, metaParent=self.start_ctrl, buffer=1)

        xform = pm.PyNode(guide.locs[-1]).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        self.end_ctrl = self.addCtrl(shape='box', size=ctrlSize * .2, name=self.getName('end'), xform=xform,
                                     parent=self.controls, metaParent=self.mid_ctrl)

        initPoints = [(i, 0, 0) for i in range(3)]
        self.crv = curve.curveThroughPoints(name=self.getName('crv'), positions=initPoints, degree=2)
        self.crv.setParent(self.rig)

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

        mappingPairs = [[self.start_ctrl, self.guide.locs[1]],
                        [self.mid_ctrl, self.guide.locs[2]],
                        [self.end_ctrl, self.guide.locs[3]]]
        for pair in mappingPairs:
            self.mapToGuideLocs(pair[0], pair[1])

    def addAttributes(self):
        attribute.addAngleAttr(self.params, 'start_twist')
        attribute.addAngleAttr(self.params, 'mid_twist')
        attribute.addAngleAttr(self.params, 'end_twist')

    def addSystems(self):
        baseMtx2Srt = mathOps.decomposeMatrix(self.base_srt.worldMatrix[0], name=self.getName('base_mtx2Srt'))
        startMtx2Srt = mathOps.decomposeMatrix(self.start_ctrl.worldMatrix[0], name=self.getName('start_mtx2Srt'))
        midMtx2Srt = mathOps.decomposeMatrix(self.mid_ctrl.worldMatrix[0], name=self.getName('mid_mtx2Srt'))
        endMtx2Srt = mathOps.decomposeMatrix(self.end_ctrl.worldMatrix[0], name=self.getName('end_mtx2Srt'))

        mainVec = mathOps.subtractVector([startMtx2Srt.outputTranslate, endMtx2Srt.outputTranslate],
                                         name=self.getName('main_vec'))
        mainNorm = mathOps.normalize(mainVec.output3D, name=self.getName('main_norm'))
        startVec = mathOps.subtractVector([startMtx2Srt.outputTranslate, midMtx2Srt.outputTranslate],
                                          name=self.getName('start_vec'))
        startDist = mathOps.createDotProduct(startVec.output3D, mainNorm.output, normalize=0, name=self.getName('startDist'))


        # CONSTRAIN MID CONTROL BUFFER
        aimMtx = transform.createAimMatrix(self.start_ctrl, self.end_ctrl, name=self.getName('aim_mtx'))
        mainDist = mathOps.distance((0, 0, 0), mainVec.output3D, name=self.getName('main_dist'))
        startRatio = startDist.outputX.get() / mainDist.distance.get()
        midMult = mathOps.multiply(mainDist.distance, startRatio, name=self.getName('mid_mult'))
        aimMtx.outputMatrix.connect(self.mid_ctrl.getParent().offsetParentMatrix)
        midMult.output.connect(self.mid_ctrl.getParent().tx)
        displace = (self.guide.locs[2].worldMatrix[0].get() * self.mid_ctrl.worldInverseMatrix.get()).translate.get()[2]
        self.mid_ctrl.getParent().tz.set(displace)

        startMtx2Srt.outputTranslate.connect(self.crv.controlPoints[0])
        midMtx2Srt.outputTranslate.connect(self.crv.controlPoints[1])
        endMtx2Srt.outputTranslate.connect(self.crv.controlPoints[2])

        for index, div in enumerate(self.divs):
            num = str(index + 1).zfill(2)
            param = (1.0 / (self.guide.num_divisions - 1)) * index
            mp = curve.createMotionPathNode(self.crv, name=self.getName('%s_mp' % num), follow=0)
            mp.uValue.set(param)

            posMtx = mathOps.createComposeMatrix(inputTranslate=mp.allCoordinates)

            divMtx = transform.blend_T_R_matrices(posMtx.outputMatrix, aimMtx.outputMatrix,
                                                  name=self.getName('div_%s_mtx' % num))

            divMtx.outputMatrix.connect(div.offsetParentMatrix)

        # Attach params shape to mid ctrl
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.mid_ctrl, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):
        self.setColours(self.guide)

        nodes = [node for node in self.controls_list if not node == self.params]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        attrList = ['visibility']
        attribute.channelControl(nodeList=self.controls_list, attrList=attrList)

        attrList = ['sx', 'sy', 'sz']
        attribute.channelControl(nodeList=[self.start_ctrl, self.end_ctrl], attrList=attrList)

        attrList = []
        for attr in attrList:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.mid_ctrl)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TArc(guide)

