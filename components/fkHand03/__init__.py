from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon
reload(components)

import pymel.core as pm

class TFkHand03(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = 0
        if self.guide.guide_side == 'R':
            self.invert = 1
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'fkHand03')
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
                                                       str(seg+1).zfill(2))) for seg in range(guide.num_segments)]
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
                if loc != locs[-1]:
                    srt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_base_srt' % (num, segNum)))
                    self.fingerDict[num]['srts'].append(srt)
                    srt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_knuckle_srt' % (num, segNum)))
                    self.fingerDict[num]['srts'].append(srt)
        if guide.thumb:
            self.fingerDict['thumb'] = {}
            self.fingerDict['thumb']['controls'] = []
            self.fingerDict['thumb']['srts'] = []
            locs = [pm.PyNode('%s_%s%s_thumb_%s_guide' % (guide.guide_name, guide.guide_side,
                                                          guide.guide_index,
                                                          str(seg+1).zfill(2))) for seg in range(guide.num_segments-1)]
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
                                    xform=xform, parent=parent, metaParent=metaParent)
                parent = ctrl
                self.fingerDict['thumb']['controls'].append(ctrl)

                if loc != locs[-1]:
                    srt = dag.addChild(self.rig, 'group', name=self.getName('thumb_%s_base_srt' % (segNum)))
                    self.fingerDict['thumb']['srts'].append(srt)
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
            negMtx = mathOps.createComposeMatrix(inputScale=(1, 1, -1), name=self.getName('neg_mtx'))
        for key in self.fingerDict.keys():
            finger = self.fingerDict[key]
            bases = []
            knuckles = []
            for index, ctrl in enumerate(finger['controls'][:-1]):
                if ctrl != finger['controls'][-1]:
                    dist = mathOps.getDistance(ctrl, finger['controls'][index+1])*0.5
                else:
                    dist = mathOps.getDistance(ctrl, finger['controls'][index-1])*0.5
                baseSrt = finger['srts'][index*2]
                bases.append(baseSrt)
                try:
                    knuckleSrt = finger['srts'][(index*2)+1]
                    knuckles.append(knuckleSrt)
                except:
                    pass
                mtxAttr = ctrl.worldMatrix[0]
                if self.invert:
                    mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, mtxAttr],
                                                   name=ctrl.name().replace('_ctrl', '_neg_mtx'))
                    mtxAttr = mtx.matrixSum

                blendPosMtx = transform.blendMatrices(ctrl.worldMatrix[0], finger['controls'][index+1].worldMatrix[0],
                                                      name=ctrl.name().replace('_ctrl', '_blendPosMtx'))
                aimMtx = transform.createAimMatrix(mtxAttr, finger['controls'][index+1].worldMatrix[0],
                                                   name=ctrl.name().replace('_ctrl', '_aimMtx'))
                resultMtx = transform.blendMatrices(blendPosMtx.outputMatrix, aimMtx.outputMatrix, weight=1,
                                                      name=ctrl.name().replace('_ctrl', '_baseMtx'))
                resultMtx.target[0].useTranslate.set(0)
                resultMtx.outputMatrix.connect(baseSrt.offsetParentMatrix)

            for index, knuckle in enumerate(knuckles):
                ctrl = finger['controls'][index+1]
                prev = bases[index]
                next = None
                try:
                    next = bases[index+1]
                except:
                    pass
                if next:
                    rotMtx = transform.blendMatrices(prev.worldMatrix[0], next.worldMatrix[0],
                                                     name=ctrl.name().replace('_ctrl', '_knuckleRot_mtx'))
                else:
                    rotMtx = transform.blendMatrices(prev.worldMatrix[0], prev.worldMatrix[0],
                                                     name=ctrl.name().replace('_ctrl', '_knuckleRot_mtx'))
                blendMtx = transform.blend_T_R_matrices(ctrl.worldMatrix[0], rotMtx.outputMatrix,
                                                        name=ctrl.name().replace('_ctrl', '_knuckle_mtx'))
                blendMtx.target[0].useScale.set(1)
                blendMtx.outputMatrix.connect(knuckle.offsetParentMatrix)

                if next:
                    prevAim = mathOps.createMatrixAxisVector(prev.worldMatrix[0], (1, 0, 0),
                                                       name=ctrl.name().replace('_ctrl', '_inAxis'))
                    nextAim = mathOps.createMatrixAxisVector(next.worldMatrix[0], (1, 0, 0),
                                                       name=ctrl.name().replace('_ctrl', '_outAxis'))
                    dot = mathOps.createDotProduct(prevAim.output, nextAim.output,
                                      name=ctrl.name().replace('_ctrl', '_dot'))
                    scaleRange = mathOps.remap(dot.outputX, -1, 1, 2, 1,
                                               name=ctrl.name().replace('_ctrl', '_dot_remap'))
                    scaleRange.outValueX.connect(knuckle.sy)

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

        if self.guide.thumb:
            fingers = [self.fingerDict[str(i+1).zfill(2)] for i in range(len(self.fingerDict.keys())-1)]
        else:
            fingers = [self.fingerDict[str(i+1).zfill(2)] for i in range(len(self.fingerDict.keys()))]
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
                if index == 1:
                    self.masterCtrl.rz.connect(curl.inputB)
                    self.masterCtrl.rx.connect(buffer.rx)
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

        # Alias attrs on master control
        attrPairs = [
            ['curl', self.masterCtrl.tx],
            ['curl_bias', self.masterCtrl.tz],
            ['spread', self.masterCtrl.ty],
            ['swing', self.masterCtrl.ry],
            ['roll', self.masterCtrl.rx],
            ['bend', self.masterCtrl.rz],
        ]
        for pair in attrPairs:
            pm.aliasAttr(pair[0], pair[1])

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TFkHand03(guide)
