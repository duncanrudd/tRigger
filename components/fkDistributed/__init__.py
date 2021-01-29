from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon
reload(components)

import pymel.core as pm

class TFkDistributed(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = (self.guide.guide_side == 'R')
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'fkDistributed')
        print 'Created FK Chain Component: %s' % self.comp_name

    def addObjects(self, guide):
        transform.align(self.base_srt, guide.root)
        locs = guide.locs[1:]
        locs.append(guide.root)
        locs.reverse()

        ctrlSize = mathOps.getDistance(guide.locs[0], guide.locs[-1])
        parent = self.base_srt
        metaParent = self.base_srt
        self.srts = []
        self.out_srts = []
        for index, loc in enumerate(locs):
            segNum = str(index+1).zfill(2)
            xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))

            if self.invert:
                xform = mathOps.getInverseHandedMatrix(xform)
            ctrl = self.addCtrl(shape='box', size=ctrlSize * .15,
                                name=self.getName('%s' % segNum),
                                xform=xform, parent=parent, metaParent=metaParent, buffer=1)
            parent = ctrl
            metaParent = ctrl
            self.mapToGuideLocs(ctrl, loc)


            if index < (len(locs)-1):
                points = mathOps.getPointsAlongVector(loc, locs[index+1], divs=self.guide.sub_segments+1)
                for x in range(guide.sub_segments):
                    subNum = str(x+1).zfill(2)
                    xform.translate = points[x]
                    srt = pm.createNode('transform', name=self.getName('%s_%s_srt' % (segNum, subNum)))
                    srt.offsetParentMatrix.set(xform)
                    srt.setParent(parent)
                    parent=srt
                    self.srts.append(srt)

                    outSrt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_out_srt' % (segNum, subNum)))
                    self.out_srts.append(outSrt)
            else:
                srt = pm.createNode('transform', name=self.getName('%s_01_srt' % segNum))
                srt.offsetParentMatrix.set(xform)
                srt.setParent(parent)
                self.srts.append(srt)

                outSrt = dag.addChild(self.rig, 'group', name=self.getName('%s_01_out_srt' % segNum))
                self.out_srts.append(outSrt)


        if guide.root.add_joint.get():
            for srt in self.out_srts:
                j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                if srt != self.out_srts[0]:
                    j.setParent(self.joints_list[-1]['joint'])
                self.joints_list.append({'joint': j, 'driver': srt})

    def addSystems(self):
        if self.invert:
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))

        for index, ctrl in enumerate(self.controls_list[1:-1]):
            for x in range(self.guide.sub_segments):
                if x > 0:
                    sub = self.srts[(index*self.guide.sub_segments)+x]
                    transform.bakeSrtToOffsetParentMtx(sub)
                    ctrl.r.connect(sub.r)
                    ctrl.ro.connect(sub.ro)

        for index, ctrl in enumerate(self.srts):
            driver = ctrl.worldMatrix[0]
            num = str(index+1).zfill(2)
            if self.invert:
                mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, driver],
                                               name=self.getName('ctrl_%s_neg_mtx' % num))
                driver = mtx.matrixSum
            driver.connect(self.out_srts[index].offsetParentMatrix)

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
    return TFkDistributed(guide)
