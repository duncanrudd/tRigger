from tRigger import components
from tRigger.core import attribute, transform, mathOps, dag, icon
reload(components)

import pymel.core as pm

class TFkHand(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        self.invert = 0
        if self.guide.guide_side == 'R':
            self.invert = 1
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'fkHand')
        print 'Created FK Hand Component: %s' % self.comp_name

    def addObjects(self, guide):
        transform.align(self.base_srt, guide.root)
        self.fingerDict = {}
        skewStartIndex = -2 + self.guide.skew_start
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
            parent = self.base_srt
            for index, loc in enumerate(locs):
                segNum = str(index+1).zfill(2)
                xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
                if self.invert:
                    xform = mathOps.getInverseHandedMatrix(xform)
                ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.15,
                                    name=self.getName('%s_%s' % (num, segNum)),
                                    xform=xform, parent=parent, metaParent=parent)
                parent = ctrl
                self.fingerDict[num]['controls'].append(ctrl)
                srt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_base_srt' % (num, segNum)))
                self.fingerDict[num]['srts'].append(srt)
                if skewStartIndex < index < (guide.num_segments-2):
                    # add srt for tip of segment
                    srt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_tip_srt' % (num, segNum)))
                    self.fingerDict[num]['srts'].append(srt)
        if guide.thumb:
            self.fingerDict['thumb'] = {}
            self.fingerDict['thumb']['controls'] = []
            self.fingerDict['thumb']['srts'] = []
            locs = [pm.PyNode('%s_%s%s_thumb_%s_guide' % (guide.guide_name, guide.guide_side,
                                                          guide.guide_index,
                                                          str(seg+1).zfill(2))) for seg in range(guide.num_segments-1)]
            ctrlSize = mathOps.getDistance(locs[0], locs[-1])
            parent = self.base_srt
            for index, loc in enumerate(locs):
                segNum = str(index+1).zfill(2)
                xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
                if self.invert:
                    xform = mathOps.getInverseHandedMatrix(xform)

                ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.15,
                                    name=self.getName('thumb_%s' % segNum),
                                    xform=xform, parent=parent, metaParent=parent)
                parent = ctrl
                self.fingerDict['thumb']['controls'].append(ctrl)

                srt = dag.addChild(self.rig, 'group', name=self.getName('thumb_%s_base_srt' % (segNum)))
                self.fingerDict['thumb']['srts'].append(srt)
                if skewStartIndex < index < (guide.num_segments-3):
                    # add srt for tip of segment
                    srt = dag.addChild(self.rig, 'group', name=self.getName('thumb_%s_tip_srt' % (segNum)))
                    self.fingerDict['thumb']['srts'].append(srt)

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('base_jnt'))
            self.joints_list.append({'joint': j, 'driver': self.base_srt})
            for key in self.fingerDict.keys():
                for srt in self.fingerDict[key]['srts']:
                    j = pm.createNode('joint', name=srt.name().replace('srt', 'jnt'))
                    self.joints_list.append({'joint': j, 'driver': srt})

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'bulge', minValue=0, maxValue=1)
        attribute.addFloatAttr(self.params, 'hand_scale', minValue=0.01, value=1)

    def addSystems(self):
        aimAxis = (1, 0, 0)
        if self.invert:
            aimAxis = (-1, 0, 0)
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
        for key in self.fingerDict.keys():
            finger = self.fingerDict[key]
            finger['aims'] = []
            for index, ctrl in enumerate(finger['controls']):
                baseMtx = ctrl.worldMatrix[0]
                if self.invert:
                    baseMtx = mathOps.createInverseHandedMatrix(baseMtx, composeMtx=negMtx,
                                                                name=ctrl.name().replace('ctrl', 'base_mtx')).matrixSum
                baseSrt = pm.PyNode(ctrl.name().replace('ctrl', 'base_srt'))
                if ctrl != finger['controls'][-1]:
                    aimMtx = transform.createAimMatrix(baseMtx, finger['controls'][index+1],
                                                       name=ctrl.name().replace('ctrl', 'aim_mtx'))
                    aimMtx.primaryInputAxis.set(aimAxis)
                    aimMtx.outputMatrix.connect(baseSrt.offsetParentMatrix)
                    vec = mathOps.createMatrixAxisVector(aimMtx.outputMatrix, (1, 0, 0),
                                                         name=aimMtx.name().replace('mtx', 'vec'))
                    finger['aims'].append(vec)
                    skewStartIndex = -2 + self.guide.skew_start
                    if skewStartIndex < index < (len(finger['controls'])-2):
                        tipSrt = pm.PyNode(ctrl.name().replace('ctrl', 'tip_srt'))
                        blendMtx = transform.blendMatrices(aimMtx.outputMatrix,
                                                           finger['controls'][index+1].worldMatrix[0],
                                                           name=aimMtx.name().replace('aim', 'tip'))
                        blendMtx.target[0].weight.set(.99)
                        blendMtx.target[0].useScale.set(0)
                        blendMtx.target[0].useRotate.set(0)
                        blendMtx.target[0].useShear.set(0)
                        blendMtx.outputMatrix.connect(tipSrt.offsetParentMatrix)
                else:
                    baseMtx.connect(baseSrt.offsetParentMatrix)
            skewStartIndex = self.guide.skew_start - 1
            for index, aim in enumerate(finger['aims'][skewStartIndex:-1]):
                mtx = pm.listConnections(aim.matrix, s=1, p=1, d=0)[0]
                upVec = mathOps.createMatrixAxisVector(mtx, (0, 1, 0), name=aim.name().replace('aim_vec', 'up_vec'))
                dot = mathOps.createDotProduct(finger['aims'][index+self.guide.skew_start].output, upVec.output,
                                               name=aim.name().replace('aim_vec', 'dot'))
                dotRange = mathOps.remap(dot.outputX, -.1, .1, 1, -1, name=aim.name().replace('aim_vec', 'signed_dot'))
                angle = mathOps.angleBetween(aim.output, finger['aims'][index+self.guide.skew_start].output,
                                             name=aim.name().replace('aim_vec', 'angle'))
                signedAngle = pm.createNode('animBlendNodeAdditiveDA',
                                            name=aim.name().replace('aim_vec', 'signed_angle'))
                angle.angle.connect(signedAngle.inputA)
                dotRange.outValueX.connect(signedAngle.weightA)
                angleUC = pm.createNode('unitConversion', name=aim.name().replace('aim_vec', 'angle_uc'))
                signedAngle.output.connect(angleUC.input)
                angleUC.conversionFactor.set(.67)
                inSrt = pm.PyNode(aim.name().replace('aim_vec', 'tip_srt'))
                outSrt = pm.PyNode(finger['aims'][index+self.guide.skew_start].name().replace('aim_vec', 'base_srt'))
                angleUC.output.connect(inSrt.shearXY)
                outShear = mathOps.multiply(angleUC.output, -1, name=aim.name().replace('aim_vec', 'out_shear'))
                outShear.output.connect(outSrt.shearXY)

                bulgeClamp = mathOps.clamp(angleUC.output, 0, 100000, name=aim.name().replace('aim_vec', 'bulge_clamp'))
                bulgeMult = mathOps.multiply(angleUC.output, self.params.bulge,
                                             name=aim.name().replace('aim_vec', 'bulge_mult'))
                bulgeSum = mathOps.addScalar([1.0, bulgeMult.output], name=aim.name().replace('aim_vec', 'bulge_sum'))
                bulgeSum.output1D.connect(inSrt.sz)
                bulgeSum.output1D.connect(outSrt.sz)

        self.params.hand_scale.connect(self.base_srt.sx)
        self.params.hand_scale.connect(self.base_srt.sz)
        self.params.hand_scale.connect(self.base_srt.sy)

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
        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TFkHand(guide)
