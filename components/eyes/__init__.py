from tRigger import components
import math
from tRigger.core import attribute, transform, dag, mathOps, curve, icon
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)
reload(curve)

import pymel.core as pm

class TEyes(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'eyes')
        print 'Created Eyes Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = guide.guide_side == 'R'
        if self.invert:
            self.negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
        ctrlSize = 5
        self.rig_srt = dag.addChild(self.rig, 'group', self.getName('rig_base_srt'))
        self.rig_srt.offsetParentMatrix.set(self.base_srt.worldMatrix[0].get())

        self.ctrl = self.addCtrl(shape='box', size=ctrlSize*5,
                                 name=self.getName(''), xform=self.base_srt.worldMatrix[0].get(),
                                 parent=self.base_srt, metaParent=self.base_srt)
        self.eyeball_ctrl = self.addCtrl(shape='arrowForward', size=ctrlSize*5,
                                 name=self.getName('eyeball'), xform=self.base_srt.worldMatrix[0].get(),
                                 parent=self.controls, metaParent=self.ctrl)
        if self.invert:
            self.ctrl.offsetParentMatrix.set(self.negMtx.outputMatrix.get() * self.ctrl.offsetParentMatrix.get())
        self.aimCtrl = self.addCtrl(shape='circlePoint', size=ctrlSize*5,
                                 name=self.getName('aim'), xform=guide.locs[-1].worldMatrix[0].get(),
                                 parent=self.controls, metaParent=self.ctrl)
        self.tugSrt = dag.addChild(self.rig, 'group', self.getName('tug_srt'))
        self.tugSrt.offsetParentMatrix.set(self.base_srt.worldMatrix[0].get())

        if self.guide.add_joint:
            root_j = pm.createNode('joint', name=self.getName('root_jnt'))
            driver = self.ctrl
            if self.invert:
                self.root_srt = dag.addChild(self.rig, 'group', self.getName('root_out_srt'))
                rootNegMtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, self.ctrl.worldMatrix[0]],
                                                      name=self.getName('root_neg_mtx'))
                rootNegMtx.matrixSum.connect(self.root_srt.offsetParentMatrix)
                driver = self.root_srt
            self.joints_list.append({'joint': root_j, 'driver': driver})
            eyeball_j = pm.createNode('joint', name=self.getName('eyeball_jnt'))
            eyeball_j.setParent(root_j)
            self.joints_list.append({'joint': eyeball_j, 'driver': self.eyeball_ctrl})
            tug_j = pm.createNode('joint', name=self.getName('tug_jnt'))
            tug_j.setParent(root_j)
            self.joints_list.append({'joint': tug_j, 'driver': self.tugSrt})

        # GUIDE TO RIG MAPPING
        self.mapToGuideLocs(self.ctrl, guide.locs[0])
        self.mapToGuideLocs(self.aimCtrl, guide.locs[-1])

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'skin_tug', minValue=0.0, maxValue=1.0)

    def addSystems(self):
        # AIM STUFF
        aimMtx = transform.createNonRollMatrix(self.ctrl, self.aimCtrl, axis='z',
                                               name=[self.getName('aim_orbit_mtx'),
                                                     self.getName('aim_elevation_mtx')])
        mtxAttr = aimMtx[1].outputMatrix
        if self.invert:
            negAimMtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, mtxAttr],
                                                 name=self.getName('neg_aim_mtx'))
            mtxAttr = negAimMtx.matrixSum
        mtxAttr.connect(self.eyeball_ctrl.offsetParentMatrix)

        mtxAttr = self.ctrl.worldMatrix[0]
        if self.invert:
            negAimMtx = mathOps.multiplyMatrices([self.negMtx.outputMatrix, mtxAttr],
                                                 name=self.getName('neg_tug_mtx'))
            mtxAttr = negAimMtx.matrixSum
        offset = transform.pmMtx2fourFourMtx(mtxAttr.get() * self.eyeball_ctrl.worldInverseMatrix[0].get(),
                                             name=self.getName('ctrl2Aim_offset_mtx'))
        ctrlMtx = mathOps.multiplyMatrices([offset.output, self.eyeball_ctrl.worldMatrix[0]],
                                           name=self.getName('tug_ctrl_mtx'))
        tugMtx = transform.blendMatrices(mtxAttr, ctrlMtx.matrixSum,
                                         name=self.getName('tug_mtx'))
        self.params.skin_tug.connect(tugMtx.target[0].weight)
        tugMtx.outputMatrix.connect(self.tugSrt.offsetParentMatrix)

        if not self.guide.root.local_rig.get():
            self.base_srt.worldMatrix[0].connect(self.rig_srt.offsetParentMatrix)

        # Attach params shape to eyeball_ctrl
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.ctrl, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):
        self.setColours(self.guide)

        attrList = ['visibility']
        nodeList = [self.ctrl]
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['visibility', 'sx', 'sy', 'sz', 'tx', 'ty', 'tz']
        nodeList = [self.eyeball_ctrl]
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        attrList = ['visibility', 'sx', 'sy', 'sz', 'rx', 'ry', 'rz']
        nodeList = [self.aimCtrl]
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodeList = self.controls_list[4:]
        attrList = ['visibility', 'rx', 'ry', 'rz', 'sx', 'sy', 'sz', 'tz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TEyes(guide)

