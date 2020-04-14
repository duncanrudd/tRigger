from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon
reload(components)

import pymel.core as pm

class TFkChain(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = (self.guide.guide_side == 'R')
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'fkChain')
        print 'Created FK Chain Component: %s' % self.comp_name

    def addObjects(self, guide):
        transform.align(self.base_srt, guide.root)
        locs = guide.locs[1:]
        locs.append(guide.root)
        locs.reverse()

        ctrlSize = mathOps.getDistance(guide.locs[0], guide.locs[-1])
        parent = self.base_srt
        self.srts = []
        for index, loc in enumerate(locs):
            segNum = str(index+1).zfill(2)
            xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
            if self.invert:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='box', size=ctrlSize*.15,
                                name=self.getName('%s' % segNum),
                                xform=xform, parent=parent, metaParent=parent)
            parent = ctrl
            srt = dag.addChild(self.rig, 'group', name=self.getName('%s_srt' % segNum))
            self.srts.append(srt)
            self.mapToGuideLocs(ctrl, loc)

        if guide.root.add_joint.get():
            for srt in self.srts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                self.joints_list.append({'joint': j, 'driver': srt})

    def addSystems(self):
        if self.invert:
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))

        for index, ctrl in enumerate(self.controls_list):
            driver = ctrl.worldMatrix[0]
            num = str(index+1).zfill(2)
            if self.invert:
                mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, driver],
                                               name=self.getName('ctrl_%s_neg_mtx' % num))
                driver = mtx.matrixSum
            driver.connect(self.srts[index].offsetParentMatrix)

        # Attach params shape to end srt
        pm.cluster(icon.getShapes(self.params), wn=[self.base_srt, self.base_srt],
                   name=self.getName('params_cluster'))

    def finish(self):
        self.setColours(self.guide)

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TFkChain(guide)
