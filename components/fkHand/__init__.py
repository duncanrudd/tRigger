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
        self.srts = []
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
            parent = self.base_srt
            for index, loc in enumerate(locs):
                segNum = str(index+1).zfill(2)
                xform = transform.list2Mtx(pm.xform(loc, q=1, m=1, ws=1))
                if self.invert:
                    xform = mathOps.getInverseHandedMatrix(xform)
                ctrl = self.addCtrl(shape='pringle', size=ctrlSize*.15,
                                    name=self.getName('%s_%s' % (num, segNum)), xform=xform, parent=parent)
                parent = ctrl
                self.fingerDict[num]['controls'].append(ctrl)
                srt = dag.addChild(self.rig, 'group', name=self.getName('%s_%s_base_srt' % (num, segNum)))
                self.fingerDict[num]['srts'].append(srt)
                if 0 < index < (guide.num_segments-2):
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
                                    name=self.getName('thumb_%s' % segNum), xform=xform, parent=parent)
                parent = ctrl
                self.fingerDict['thumb']['controls'].append(ctrl)

                srt = dag.addChild(self.rig, 'group', name=self.getName('thumb_%s_base_srt' % (segNum)))
                self.fingerDict['thumb']['srts'].append(srt)
                if 0 < index < (guide.num_segments-3):
                    # add srt for tip of segment
                    srt = dag.addChild(self.rig, 'group', name=self.getName('thumb_%s_tip_srt' % (segNum)))
                    self.fingerDict['thumb']['srts'].append(srt)

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
                    if 0 < index < (len(finger['controls'])-2):
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
            for index, aim in enumerate(finger['aims'][1:-1]):
                mtx = pm.listConnections(aim.matrix, s=1, p=1, d=0)[0]
                upVec = mathOps.createMatrixAxisVector(mtx, (0, 1, 0), name=aim.name().replace('aim_vec', 'up_vec'))
                dot = mathOps.createDotProduct(finger['aims'][index+2].output, upVec.output,
                                               name=aim.name().replace('aim_vec', 'dot'))
                dotRange = mathOps.remap(dot.outputX, -.1, .1, 1, -1, name=aim.name().replace('aim_vec', 'signed_dot'))
                angle = mathOps.angleBetween(aim.output, finger['aims'][index+2].output,
                                             name=aim.name().replace('aim_vec', 'angle'))
                signedAngle = pm.createNode('animBlendNodeAdditiveDA',
                                            name=aim.name().replace('aim_vec', 'signed_angle'))
                angle.angle.connect(signedAngle.inputA)
                dotRange.outValueX.connect(signedAngle.weightA)
                angleUC = pm.createNode('unitConversion', name=aim.name().replace('aim_vec', 'angle_uc'))
                signedAngle.output.connect(angleUC.input)
                angleUC.conversionFactor.set(.67)
                inSrt = pm.PyNode(aim.name().replace('aim_vec', 'tip_srt'))
                outSrt = pm.PyNode(finger['aims'][index+2].name().replace('aim_vec', 'base_srt'))
                angleUC.output.connect(inSrt.shearXY)
                outShear = mathOps.multiply(angleUC.output, -1, name=aim.name().replace('aim_vec', 'out_shear'))
                outShear.output.connect(outSrt.shearXY)



    def finish(self):
        colour = pm.Attribute('guide.centre_colour').get()
        if self.comp_side == 'R':
            colour = pm.Attribute('guide.right_colour').get()
        elif self.comp_side == 'L':
            colour = pm.Attribute('guide.left_colour').get()

        for node in self.controls_list:
            icon.setColourRGB(node, colour)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TFkHand(guide)
