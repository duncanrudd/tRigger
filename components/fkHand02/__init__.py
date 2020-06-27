from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon
reload(components)

import pymel.core as pm

class TFkHand02(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = 0
        if self.guide.guide_side == 'R':
            self.invert = 1
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'fkHand02')
        print 'Created FK Hand 02 Component: %s' % self.comp_name

    def addObjects(self, guide):
        transform.align(self.base_srt, guide.root)
        self.scale_srt = dag.addChild(self.base_srt, 'group', name=self.getName('scale_srt'))
        # Master ctrl
        xform = self.guide.locs[1].worldMatrix[0].get()
        if self.invert:
            xform = mathOps.getInverseHandedMatrix(xform)
        self.masterCtrl = self.addCtrl(shape='squarePoint', size=1,
                                       name=self.getName('master'), buffer=1,
                                       xform=xform, parent=self.base_srt, metaParent=self.base_srt)

        self.fingerDict = {}
        # fingers
        for i in range(guide.num_digits):
            num = str(i+1).zfill(2)
            self.fingerDict[num] = {}
            self.fingerDict[num]['controls'] = []
            self.fingerDict[num]['srts'] = []
            locs = [pm.PyNode('%s_%s%s_%s_%s_guide' % (guide.guide_name, guide.guide_side,
                                                       guide.guide_index, num,
                                                       str(seg+1).zfill(2))) for seg in range(guide.num_segments)][:-1]
            ctrlSize = mathOps.getDistance(locs[0], locs[-1])
            parent = self.scale_srt
            for index, loc in enumerate(locs):
                segNum = str(index+1).zfill(2)
                xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
                if self.invert:
                    xform = mathOps.getInverseHandedMatrix(xform)
                metaParent = parent
                if index == 0:
                    metaParent = self.masterCtrl
                ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.15,
                                    name=self.getName('%s_%s' % (num, segNum)), buffer=1,
                                    xform=xform, parent=parent, metaParent=metaParent)
                parent = ctrl
                self.fingerDict[num]['controls'].append(ctrl)
                srt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_base_srt' % (num, segNum)))
                self.fingerDict[num]['srts'].append(srt)
                if loc != locs[-1]:
                    srt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_knuckle_srt' % (num, segNum)))
                    self.fingerDict[num]['srts'].append(srt)
        if guide.thumb:
            self.fingerDict['thumb'] = {}
            self.fingerDict['thumb']['controls'] = []
            self.fingerDict['thumb']['srts'] = []
            locs = [pm.PyNode('%s_%s%s_thumb_%s_guide' % (guide.guide_name, guide.guide_side,
                                                          guide.guide_index,
                                                          str(seg+1).zfill(2))) for seg in range(guide.num_segments-1)][:-1]
            ctrlSize = mathOps.getDistance(locs[0], locs[-1])
            parent = self.scale_srt
            for index, loc in enumerate(locs):
                segNum = str(index+1).zfill(2)
                xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
                if self.invert:
                    xform = mathOps.getInverseHandedMatrix(xform)
                metaParent = parent
                if index == 0:
                    metaParent = self.masterCtrl
                ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.15,
                                    name=self.getName('thumb_%s' % segNum), buffer=1,
                                    xform=xform, parent=parent, metaParent=parent)
                parent = ctrl
                self.fingerDict['thumb']['controls'].append(ctrl)

                srt = dag.addChild(self.rig, 'group', name=self.getName('thumb_%s_base_srt' % (segNum)))
                self.fingerDict['thumb']['srts'].append(srt)
                if loc != locs[-1]:
                    srt = dag.addChild(self.rig, 'group', name=self.getName('thumb_%s_knuckle_srt' % (segNum)))
                    self.fingerDict['thumb']['srts'].append(srt)

        if guide.root.add_joint.get():
            base = pm.createNode('joint', name=self.getName('base_jnt'))
            self.joints_list.append({'joint': base, 'driver': self.base_srt})
            for key in self.fingerDict.keys():
                for srt in self.fingerDict[key]['srts']:
                    j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                    if srt == self.fingerDict[key]['srts'][0]:
                        j.setParent(base)
                    else:
                        j.setParent(self.joints_list[-1]['joint'])
                    self.joints_list.append({'joint': j, 'driver': srt})

    def addSystems(self):
        aimAxis = 1
        if self.invert:
            aimAxis = -1
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
        for key in self.fingerDict.keys():
            finger = self.fingerDict[key]
            for index, ctrl in enumerate(finger['controls']):
                if ctrl != finger['controls'][-1]:
                    dist = mathOps.getDistance(ctrl, finger['controls'][index+1])*0.5
                else:
                    dist = mathOps.getDistance(ctrl, finger['controls'][index-1])*0.5
                baseSrt = finger['srts'][index*2]
                knuckleSrt = None
                try:
                    knuckleSrt = finger['srts'][(index*2)+1]
                except:
                    pass
                mtxAttr = ctrl.worldMatrix[0]
                if self.invert:
                    mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, mtxAttr],
                                                   name=ctrl.name().replace('_ctrl', '_neg_mtx'))
                    mtxAttr = mtx.matrixSum
                mtxAttr.connect(baseSrt.offsetParentMatrix)
                baseSrt.tx.set(dist*aimAxis)
                if knuckleSrt:
                    print knuckleSrt
                    ctrl = finger['controls'][index+1]
                    blendMtx = transform.blendMatrices(ctrl.getParent().worldMatrix[0], ctrl.worldMatrix[0],
                                                       name=ctrl.name().replace('_ctrl', '_blend_mtx'))
                    mtxAttr = blendMtx.outputMatrix
                    if self.invert:
                        mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, mtxAttr],
                                                       name=ctrl.name().replace('_ctrl', '_neg_mtx'))
                        mtxAttr = mtx.matrixSum
                    mtxAttr.connect(knuckleSrt.offsetParentMatrix)

        self.masterCtrl.s.connect(self.scale_srt.s)
        dist = mathOps.getDistance(self.fingerDict['01']['controls'][0], self.fingerDict['01']['controls'][-1]) * 0.5
        self.masterCtrl.getParent().s.set((dist, dist, dist))
        pm.transformLimits(self.masterCtrl, tx=(-1, 1), ty=(-1, 1), tz=(-1, 1), etx=(1, 1), ety=(1, 1), etz=(1, 1))

        curlBackVals = [10, 90 , 20, 30]
        curlInVals = [-10, -90, -135, -60]
        curlResults = []

        for i in range(4):
            num = str(i+1).zfill(2)
            remap = mathOps.remap(self.masterCtrl.tx, -1, 0, curlBackVals[i], 0,
                                      name=self.getName('seg_%s_curl_remap' % num))
            self.masterCtrl.tx.connect(remap.valueY)
            remap.oldMinY.set(0)
            remap.oldMaxY.set(1)
            remap.minY.set(0)
            remap.maxY.set(curlInVals[i])

            result = mathOps.multiplyAngleByScalar(1.0, remap.outValueX, name=self.getName('seg_%s_curl_sum' % num))
            remap.outValueY.connect(result.weightB)
            result.inputB.set(1.0)
            curlResults.append(result)

        spreadOutVals = [-10, -45]
        spreadInVals = [10, 10]
        spreadResults = []

        for i in range(2):
            num = str(i+1).zfill(2)
            remap = mathOps.remap(self.masterCtrl.ty, -1, 0, spreadInVals[i], 0,
                                      name=self.getName('seg_%s_spread_remap' % num))
            self.masterCtrl.ty.connect(remap.valueY)
            remap.oldMinY.set(0)
            remap.oldMaxY.set(1)
            remap.minY.set(0)
            remap.maxY.set(spreadOutVals[i])

            result = mathOps.multiplyAngleByScalar(1.0, remap.outValueX, name=self.getName('seg_%s_spread_sum' % num))
            remap.outValueY.connect(result.weightB)
            result.inputB.set(1.0)
            spreadResults.append(result)

        fingers = [self.fingerDict[str(i+1).zfill(2)] for i in range(len(self.fingerDict.keys())-1)]
        for f, finger in enumerate(fingers):
            param = ((2.0 / (len(fingers)-1)) * f) - 1.0
            num = str(f+1).zfill(2)
            bias = mathOps.multiply(self.masterCtrl.tz, param, name=self.getName('seg_%s_curl_bias' % num))
            weight = mathOps.addScalar([1.0, bias.output], name=self.getName('seg_%s_curl_weight' % num))
            for index, ctrl in enumerate(finger['controls']):
                curlIndex = min(3, index)
                buffer = ctrl.getParent()
                iNum = str(index+1).zfill(2)
                curl = mathOps.multiplyAngleByScalar(curlResults[curlIndex].output, weight.output1D,
                                                     name=self.getName('seg_%s_curl_%s_weight' % (num, iNum)))
                curl.output.connect(buffer.rz)

                if index < 2:
                    spread = mathOps.multiplyAngleByScalar(spreadResults[index].output, param,
                                                         name=self.getName('seg_%s_spread_%s_weight' % (num, iNum)))
                    spread.output.connect(buffer.ry)
                    if index == 1:
                        self.masterCtrl.ry.connect(spread.inputB)
        # Not sure why this doesn't get added automatically?
        self.controls_list.append(self.masterCtrl)

    def finish(self):
        self.setColours(self.guide)

        # --------------------------------------------------
        # Set lock / hide properties on controls attrs
        # --------------------------------------------------
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)
        nodeList = [node for node in self.controls_list if not node == self.masterCtrl]
        attrList = ['tx', 'ty', 'tz']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)
        attrList = ['rx', 'rz']
        attribute.channelControl(nodeList=[self.masterCtrl], attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TFkHand02(guide)
