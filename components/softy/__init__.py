from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps
import pymel.core as pm
reload(components)
reload(transform)

import pymel.core as pm

class TSofty(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'softy')
        print 'Created Softy Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = (self.guide.guide_side == 'R')
        xform = pm.PyNode(guide.root).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        self.rootCtrl = self.addCtrl(shape='squarePoint', size=20.0,
                         name=self.getName('root'), xform=xform, parent=self.base_srt, metaParent=self.base_srt)
        self.handleCtrl = self.addCtrl(shape='ball', size=10.0,
                                     name=self.getName('handle'), xform=xform, parent=self.rootCtrl,
                                     metaParent=self.rootCtrl)

        rootXform = self.rootCtrl.worldMatrix[0]
        handleXform = self.handleCtrl.matrix
        if self.invert:
            rootXform = mathOps.createInverseHandedMatrix(self.rootCtrl.worldMatrix[0],
                                                      name=self.getName('root_out')).matrixSum
            handleXform = mathOps.createInverseHandedMatrix(self.handleCtrl.worldMatrix[0],
                                                      name=self.getName('handle_negMtx')).matrixSum


        self.root_out_srt = dag.addChild(self.rig, 'group', name=self.getName('root_out_srt'))
        rootXform.connect(self.root_out_srt.offsetParentMatrix)
        if self.invert:
            handleXform = mathOps.multiplyMatrices([handleXform, self.root_out_srt.worldInverseMatrix[0]],
                                                   name=self.getName('handle_mtx')).matrixSum
        d = mathOps.decomposeMatrix(handleXform, name=self.getName('handle_mtx2Srt'))

        pm.select(None)
        sm = pm.softMod()
        self.handle = sm[1]
        self.mod = sm[0]
        self.set = pm.listConnections(self.mod.message)[0]
        self.handle.rename(self.getName('handle'))
        self.mod.rename(self.getName('softMod'))
        self.set.rename(self.getName('softModSet'))
        self.handle.setParent(self.root_out_srt)
        d.outputTranslate.connect(self.handle.t)
        d.outputRotate.connect(self.handle.r)
        d.outputScale.connect(self.handle.s)

        components.TBaseComponent.addObjects(self, guide)

    def addAttributes(self):
        attribute.addFloatAttr(self.handleCtrl, 'strength', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.handleCtrl, 'falloff', minValue=0)
        self.handleCtrl.strength.set(1)
        self.handleCtrl.falloff.set(10)
        attribute.addBoolAttr(self.mod, 'isTSofty')
        self.mod.isTSofty.set(1)

    def addSystems(self):
        self.handleCtrl.strength.connect(self.mod.envelope)
        self.handleCtrl.falloff.connect(self.mod.falloffRadius)

        d = mathOps.decomposeMatrix(self.root_out_srt.worldMatrix[0], name=self.getName('root_mtx2Srt'))
        d.outputTranslate.connect(self.mod.falloffCenter)
        self.root_out_srt.worldInverseMatrix[0].connect(self.mod.bindPreMatrix)

    def finish(self):
        self.setColours(self.guide)

        nodes = [node for node in self.controls_list if not node == self.params]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        attrList = ['visibility']
        attribute.channelControl(nodeList=self.controls_list, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSofty(guide)

