from tRigger import components
import math
from tRigger.core import attribute, transform, dag, mathOps, curve, icon
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)
reload(curve)

import pymel.core as pm

class TEyelid(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'eyelid')
        print 'Created Eyelid Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = guide.guide_side == 'R'
        if self.invert:
            self.negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
        upperLocs = [loc for loc in guide.locs if 'upper' in loc.name()]
        ctrlSize = mathOps.getDistance(guide.root, upperLocs[0]) * .1
        self.upperCtrls = []
        self.upperSrts = []
        self.lowerCtrls = []
        self.lowerSrts = []
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

        for index in range(guide.num_ctrls):
            num = str(index).zfill(2)
            param = (1.0 / (guide.num_ctrls - 1)) * index
            if self.guide.root.bias_tweak_ctrls.get():
                lowParam = param * param
                highParam = 1 - ((1-param)*(1-param))
                param = (lowParam*(1-param)) + (highParam*param)

            def _make_ctrl_mtx(pathPos, prefix='upper'):
                localPathPos = mathOps.getTransformedPoint(pathPos, self.base_srt.worldInverseMatrix[0].get())
                aimVecNormal = pm.datatypes.Vector(localPathPos).normal()
                upVec = pm.datatypes.Vector((0, 1, 0))
                sideVec = upVec.cross(aimVecNormal)
                upVec = aimVecNormal.cross(sideVec)
                offsetMtx = pm.datatypes.Matrix(sideVec, upVec, aimVecNormal, localPathPos)
                if self.invert:
                    negMtx = pm.datatypes.Matrix((-1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0))
                    offsetMtx = negMtx*offsetMtx
                return offsetMtx

            if 0 < index < (guide.num_ctrls-1):
                pathPos = curve.sampleCurvePosition(guide.upperCrv, param)
                ctrlMtx = _make_ctrl_mtx(pathPos)*self.base_srt.worldMatrix[0].get()
                ctrl = self.addCtrl(shape='triNorth', size=ctrlSize,
                                    name=self.getName('%s_upper' % num), xform=ctrlMtx, parent=self.ctrl,
                                    metaParent=self.eyeball_ctrl)
                self.upperCtrls.append(ctrl)
                srt = dag.addChild(self.rig_srt, 'group', self.getName('%s_upper_srt' % num))
                transform.align(srt, ctrl)
                self.upperSrts.append(srt)
                if self.guide.add_joint:
                    j = pm.createNode('joint', name=self.getName('%s_upper_jnt' % num))
                    self.joints_list.append({'joint': j, 'driver': srt})

                pathPos = curve.sampleCurvePosition(guide.lowerCrv, param)
                ctrlMtx = _make_ctrl_mtx(pathPos, prefix='lower')*self.base_srt.worldMatrix[0].get()
                lowerCtrl = self.addCtrl(shape='triSouth', size=ctrlSize,
                                         name=self.getName('%s_lower' % num), xform=ctrlMtx, parent=self.ctrl,
                                         metaParent=self.eyeball_ctrl)
                self.lowerCtrls.append(lowerCtrl)
                srt = dag.addChild(self.rig_srt, 'group', self.getName('%s_lower_srt' % num))
                transform.align(srt, lowerCtrl)
                self.lowerSrts.append(srt)
                if self.guide.add_joint:
                    j = pm.createNode('joint', name=self.getName('%s_lower_jnt' % num))
                    self.joints_list.append({'joint': j, 'driver': srt})
            elif index == 0:
                pathPos = curve.sampleCurvePosition(guide.upperCrv, 0.0)
                ctrlMtx = _make_ctrl_mtx(pathPos)*self.base_srt.worldMatrix[0].get()
                self.inner_ctrl = self.addCtrl(shape='triEast', size=ctrlSize,
                                               name=self.getName('inner'), xform=ctrlMtx, parent=self.ctrl,
                                               metaParent=self.eyeball_ctrl)
                self.innerSrt = dag.addChild(self.rig_srt, 'group', self.getName('inner_srt'))
                transform.align(self.innerSrt, self.inner_ctrl)
                if self.guide.add_joint:
                    j = pm.createNode('joint', name=self.getName('inner_jnt'))
                    self.joints_list.append({'joint': j, 'driver': self.innerSrt})
            else:
                pathPos = curve.sampleCurvePosition(guide.upperCrv, 1.0)
                ctrlMtx = _make_ctrl_mtx(pathPos)*self.base_srt.worldMatrix[0].get()
                self.outer_ctrl = self.addCtrl(shape='triWest', size=ctrlSize,
                                               name=self.getName('outer'), xform=ctrlMtx, parent=self.ctrl,
                                               metaParent=self.eyeball_ctrl)
                self.outerSrt = dag.addChild(self.rig_srt, 'group', self.getName('outer_srt'))
                transform.align(self.outerSrt, self.outer_ctrl)
                if self.guide.add_joint:
                    j = pm.createNode('joint', name=self.getName('outer_jnt'))
                    self.joints_list.append({'joint': j, 'driver': self.outerSrt})

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
        self.mapToGuideLocs(self.aimCtrl, guide.locs[-1])

    def addAttributes(self):
        attribute.addBoolAttr(self.params, 'show_tweak_ctrls')
        attribute.addAngleAttr(self.params, 'upper_inner')
        attribute.addAngleAttr(self.params, 'upper_outer')
        attribute.addAngleAttr(self.params, 'upper_lid')
        attribute.addAngleAttr(self.params, 'lower_lid')
        attribute.addAngleAttr(self.params, 'lower_inner')
        attribute.addAngleAttr(self.params, 'lower_outer')
        attribute.addFloatAttr(self.params, 'blink', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'blink_height', minValue=0.0, maxValue=1.0)
        attribute.addFloatAttr(self.params, 'auto_lids', minValue=0.0, maxValue=1.0)
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
        autoMtx = mathOps.multiplyMatrices([self.eyeball_ctrl.worldMatrix[0], self.tugSrt.worldInverseMatrix[0]],
                                           name=self.getName('auto_mtx'))
        autoMtx2Srt = mathOps.decomposeMatrix(autoMtx.matrixSum, name=self.getName('auto_mtx2Srt'))
        autoMult = mathOps.multiplyAngleByScalar(autoMtx2Srt.outputRotateX, self.params.auto_lids,
                                                 name=self.getName('auto_mult'))
        autoMult.output.connect(self.rig_srt.rx)


        if not self.guide.root.local_rig.get():
            self.base_srt.worldMatrix[0].connect(self.rig_srt.offsetParentMatrix)

        for ctrl in self.controls_list:
            if not ctrl in [self.eyeball_ctrl, self.params, self.ctrl]:
                self.params.show_tweak_ctrls.connect(ctrl.visibility)
                
        for index, ctrl in enumerate(self.upperCtrls):
            num = str(index+1).zfill(2)
            param = (1.0 / (len(self.upperCtrls)+1))*(index+1)
            midDist = (0.5 - math.fabs(param-0.5))
            mult = math.sin(math.pi*midDist)
            multScaled = 1 - ((1-mult) * (1-mult))

            # UPPER
            elevationMtx = mathOps.createComposeMatrix(name=self.getName('%s_upper_elevation_mtx' % num))
            lidMult = mathOps.multiplyAngleByScalar(self.params.upper_lid, multScaled,
                                                    name=self.getName('%s_upper_lid_mult' % num))

            innerMult = mathOps.multiplyAngleByScalar(self.params.upper_inner, multScaled*(1 - param),
                                                      name=self.getName('%s_upper_inner_mult' % num))
            self.params.upper_outer.connect(innerMult.inputB)
            innerMult.weightB.set(multScaled*param)
            innerMult.output.connect(lidMult.inputB)
            lidMult.output.connect(elevationMtx.inputRotateX)
            offsetMtx = mathOps.composeMatrixFromMatrix(ctrl.offsetParentMatrix.get())
            lidMtx = mathOps.multiplyMatrices([offsetMtx.outputMatrix, elevationMtx.outputMatrix],
                                              name=self.getName('%s_upper_lid_mtx' % num))
            lidMtx.matrixSum.connect(ctrl.offsetParentMatrix)

            # LOWER
            elevationMtx = mathOps.createComposeMatrix(name=self.getName('%s_lower_elevation_mtx' % num))
            lidMult = mathOps.multiplyAngleByScalar(self.params.lower_lid, multScaled,
                                                    name=self.getName('%s_lower_lid_mult' % num))

            innerMult = mathOps.multiplyAngleByScalar(self.params.lower_inner, multScaled*(1 - param),
                                                      name=self.getName('%s_lower_inner_mult' % num))
            self.params.lower_outer.connect(innerMult.inputB)
            innerMult.weightB.set(multScaled*param)
            innerMult.output.connect(lidMult.inputB)
            lidMult.output.connect(elevationMtx.inputRotateX)
            offsetMtx = mathOps.composeMatrixFromMatrix(self.lowerCtrls[index].offsetParentMatrix.get())
            lidMtx = mathOps.multiplyMatrices([offsetMtx.outputMatrix, elevationMtx.outputMatrix],
                                              name=self.getName('%s_lower_lid_mtx' % num))
            lidMtx.matrixSum.connect(self.lowerCtrls[index].offsetParentMatrix)


            # SRT STUFF
            dist = mathOps.getDistance(self.base_srt, ctrl)
            localMtx = mathOps.multiplyMatrices([ctrl.matrix, ctrl.offsetParentMatrix],
                                                name=self.getName('%s_upper_ctrl_mtx' % num))
            localDm = mathOps.decomposeMatrix(localMtx.matrixSum, name=self.getName('%s_upper_ctrl_mtx2Srt' % num))


            dist = mathOps.getDistance(self.base_srt, self.lowerCtrls[index])
            localMtx = mathOps.multiplyMatrices([self.lowerCtrls[index].matrix, self.lowerCtrls[index].offsetParentMatrix],
                                                name=self.getName('%s_lower_ctrl_mtx' % num))
            lowerLocalDm = mathOps.decomposeMatrix(localMtx.matrixSum, name=self.getName('%s_lower_ctrl_mtx2Srt' % num))

            pb = mathOps.pairBlend(translateB=localDm.outputTranslate, translateA=lowerLocalDm.outputTranslate,
                                   rotateB=localDm.outputRotate, rotateA=lowerLocalDm.outputRotate,
                                   weight=self.params.blink_height,
                                   name=self.getName('%s_blink_height_blend' % num), quatBlend=1)

            upperPb = mathOps.pairBlend(translateA=localDm.outputTranslate, translateB=pb.outTranslate,
                                        rotateA=localDm.outputRotate, rotateB=pb.outRotate,
                                        weight=self.params.blink,
                                        name=self.getName('%s_upper_blink_blend' % num), quatBlend=1)

            lowerPb = mathOps.pairBlend(translateA=lowerLocalDm.outputTranslate, translateB=pb.outTranslate,
                                        rotateA=lowerLocalDm.outputRotate, rotateB=pb.outRotate,
                                        weight=self.params.blink,
                                        name=self.getName('%s_lower_blink_blend' % num), quatBlend=1)

            normalPos = mathOps.normalize(upperPb.outTranslate, name=self.getName('%s_upper_normal_pos' % num))
            scaledPos = mathOps.multiplyVector(normalPos.output, (dist, dist, dist),
                                               name=self.getName('%s_upper_clamped_pos' % num))

            scaledPos.output.connect(self.upperSrts[index].t)
            upperPb.outRotate.connect(self.upperSrts[index].r)

            normalPos = mathOps.normalize(lowerPb.outTranslate, name=self.getName('%s_lower_normal_pos' % num))
            scaledPos = mathOps.multiplyVector(normalPos.output, (dist, dist, dist),
                                               name=self.getName('%s_lower_clamped_pos' % num))

            scaledPos.output.connect(self.lowerSrts[index].t)
            lowerPb.outRotate.connect(self.lowerSrts[index].r)
            
        # INNER AND OUTER SRT STUFF
        dist = mathOps.getDistance(self.base_srt, self.innerSrt)
        localMtx = mathOps.multiplyMatrices([self.inner_ctrl.matrix, self.inner_ctrl.offsetParentMatrix],
                                            name=self.getName('inner_ctrl_mtx'))
        localDm = mathOps.decomposeMatrix(localMtx.matrixSum, name=self.getName('inner_ctrl_mtx2Srt'))
        normalPos = mathOps.normalize(localDm.outputTranslate, name=self.getName('inner_normal_pos'))
        scaledPos = mathOps.multiplyVector(normalPos.output, (dist, dist, dist),
                                           name=self.getName('inner_clamped_pos'))
        localDm.outputRotate.connect(self.innerSrt.r)
        scaledPos.output.connect(self.innerSrt.t)
        
        dist = mathOps.getDistance(self.base_srt, self.outerSrt)
        localMtx = mathOps.multiplyMatrices([self.outer_ctrl.matrix, self.outer_ctrl.offsetParentMatrix],
                                            name=self.getName('outer_ctrl_mtx'))
        localDm = mathOps.decomposeMatrix(localMtx.matrixSum, name=self.getName('outer_ctrl_mtx2Srt'))
        normalPos = mathOps.normalize(localDm.outputTranslate, name=self.getName('outer_normal_pos'))
        scaledPos = mathOps.multiplyVector(normalPos.output, (dist, dist, dist),
                                           name=self.getName('outer_clamped_pos'))
        localDm.outputRotate.connect(self.outerSrt.r)
        scaledPos.output.connect(self.outerSrt.t)

        # ---------------------------------
        # Internal spaces switching setup
        # ---------------------------------
        # self.spaces['%s' % (self.aimCtrl.name())] = 'eye: %s.worldMatrix[0]' % self.ctrl.name()

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

        attrList = ['upper_lid', 'upper_inner', 'upper_outer', 'lower_lid',
                    'lower_inner', 'lower_outer', 'blink', 'blink_height',
                    'show_tweak_ctrls', 'auto_lids', 'skin_tug']
        for attr in attrList:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.ctrl)

        spaceAttrs = [attr for attr in ['aim_ctrl_parent_space', 'aim_ctrl_translate_space',
                                        'aim_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.aimCtrl)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TEyelid(guide)

