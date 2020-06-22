from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon
import math
reload(components)

import pymel.core as pm

class TControlArray(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = (self.guide.guide_side == 'R')
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'controlArray')
        print 'Created FK Chain Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = (self.guide.guide_side == 'R')
        transform.align(self.base_srt, guide.root)
        locs = guide.locs[1:]
        locs.reverse()

        ctrlSize = max(.1, mathOps.getDistance(guide.locs[0], guide.locs[-1]))
        self.srts = []
        for loc in locs:
            buffer = 0
            if loc.hasAttr('buffer'):
                buffer = loc.buffer.get()
            ctrlName = loc.ctrl_name.get()
            parentKey = guide.locDict[loc.name().split('_')[2]]['parent']
            if 'root' in parentKey:
                parent = self.base_srt
            else:
                parentName = [l for l in locs if parentKey in l.name()][0].ctrl_name.get()
                parent = pm.PyNode(self.getName(parentName) + '_ctrl')
            xform = loc.worldMatrix[0].get()
            ctrlInvert = self.invert
            try:
                ctrlInvert = math.fabs(ctrlInvert - loc.invert.get())
            except:
                pass
            if ctrlInvert:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='box', size=ctrlSize*.15,
                                name=self.getName(ctrlName), buffer=buffer,
                                xform=xform, parent=parent, metaParent=parent)
            if loc.spaces.get():
                ctrl.setParent(self.controls)

            srt = dag.addChild(self.rig, 'group', name=ctrl.name().replace('ctrl', 'srt'))
            self.srts.append(srt)
            self.mapToGuideLocs(srt, loc)
            self.mapToGuideLocs(self.base_srt, guide.root)
            self.copyGuideMapping(srt, ctrl)
            # Add mapping connection to control - used when mapping controller tags
            self.mapToControl(ctrl, srt)

            if guide.root.add_joint.get():
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                if parent != self.base_srt:
                    parent = pm.PyNode(self.getName(parentName) + '_jnt')
                    j.setParent(parent)
                self.joints_list.append({'joint': j, 'driver': srt})

    def addSystems(self):
        local = self.guide.local_rig
        print 'Local = %s' % local
        if local:
            self.localOffsetMtx = transform.pmMtx2fourFourMtx(self.base_srt.worldMatrix[0].get(),
                                                              name=self.getName('local_offset_mtx'))
        if self.invert:
                negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1),
                                                     name=self.getName('neg_mtx'))
        for index, srt in enumerate(self.srts):
            driver = self.controls_list[index+1].worldMatrix[0]
            if local:
                localMtx = mathOps.multiplyMatrices([driver, self.base_srt.worldInverseMatrix[0],
                                                     self.localOffsetMtx.output],
                                                    name=srt.name().replace('_srt', '_local_mtx'))
                driver = localMtx.matrixSum
            if self.invert:
                neg = mathOps.multiplyMatrices([negMtx.outputMatrix, driver],
                                               name=srt.name().replace('_srt', '_negScale_mtx'))
                driver = neg.matrixSum
            driver.connect(srt.offsetParentMatrix)

    def finish(self):
        self.setColours(self.guide)

        ctrls = [node for node in self.controls_list if not node == self.params]

        attribute.channelControl(nodeList=ctrls, attrList=['rotateOrder'], keyable=1, lock=0)

        animAttrList = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz', 'ro', 'sx', 'sy', 'sz']

        for ctrl in ctrls:
            mappedNode = pm.listConnections(ctrl.mapping_node)[0]
            guideNode = self.getGuideLocFromMappedNode(mappedNode)
            attrList = [attr for attr in animAttrList if not
                        pm.Attribute('%s.enable_%s' % (guideNode.name(), attr)).get()]
            attribute.channelControl(nodeList=[ctrl], attrList=attrList)


        attrList = ['visibility']
        attribute.channelControl(nodeList=self.controls_list, attrList=attrList)



def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TControlArray(guide)
