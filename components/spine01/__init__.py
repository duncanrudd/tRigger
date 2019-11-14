from tRigger import components
from tRigger.core import attribute, dag, mathOps, transform
reload(components)
reload(mathOps)

import pymel.core as pm

class TSpine01(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'spine01')
        print 'Created Spine01 Component: %s' % self.comp_name

    def addObjects(self, guide):
        # FK controls - base, lower upper
        xform = pm.xform(pm.PyNode(guide.root), q=1, ws=1, m=1)
        self.base_ctrl = self.addCtrl(shape='circlePoint', size=20.0,
                                      name=self.getName('fk_base'), xform=xform, parent=self.base_srt, buffer=1)
        xform = pm.xform(pm.PyNode(guide.divisionLocs[guide.num_divisions-1]), q=1, ws=1, m=1)
        self.lower_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                       name=self.getName('fk_base'), xform=xform, parent=self.base_ctrl, buffer=1)
        xform = pm.xform(pm.PyNode(guide.divisionLocs[len(guide.divisionLocs)-1]), q=1, ws=1, m=1)
        self.upper_ctrl = self.addCtrl(shape='circlePoint', size=15.0,
                                       name=self.getName('fk_base'), xform=xform, parent=self.lower_ctrl, buffer=1)

        self.divs = []
        for i, div in enumerate(guide.divisionLocs):
            num = str(i+1).zfill(2)
            mtx = pm.xform(div, q=1, ws=1, m=1)
            parent = self.rig
            if i > 0:
                parent = self.divs[i-1]
            node = dag.addChild(parent, 'group', name=self.getName('div_%s_driven_srt' % num))
            pm.xform(node, ws=1, m=mtx)
            buffer = dag.addParent(node, 'group', name=self.getName('div_%s_buffer_srt' % num))
            self.mapToGuideLocs(node, div)
            self.divs.append(node)

        # IK controls - lower, mid, upper
        xform = pm.xform(self.base_ctrl, q=1, ws=1, m=1)
        self.ik_lower_ctrl = self.addCtrl(shape='squarePoint', size=10.0,
                                          name=self.getName('ik_lower'), xform=xform, parent=self.controls, buffer=1)
        xform = pm.xform(self.lower_ctrl, q=1, ws=1, m=1)
        self.ik_mid_ctrl = self.addCtrl(shape='squarePoint', size=8.0,
                                        name=self.getName('ik_mid'), xform=xform, parent=self.controls, buffer=1)
        xform = pm.xform(self.upper_ctrl, q=1, ws=1, m=1)
        self.ik_upper_ctrl = self.addCtrl(shape='squarePoint', size=10.0,
                                          name=self.getName('ik_base'), xform=xform, parent=self.controls, buffer=1)

        # Drive ik mid buffer
        dm = transform.decomposeMatrix(self.lower_ctrl.worldMatrix[0], name=self.getName('lower_ctrl_mtx2Srt'))
        pb = pm.createNode('pairBlend', name=self.getName('ik_mid_translate_blend'))
        pb.weight.set(self.guide.root.mid_point.get())
        ikDist = mathOps.distance(self.ik_lower_ctrl, self.ik_upper_ctrl, name=self.getName('ik_dist'))

        ikLowerAimLen = mathOps.multiply(ikDist.distance, 0.2, name=self.getName('ik_lower_aim_len'))
        ikLowerAim = mathOps.createTransformedPoint((0, 0, 0), self.ik_lower_ctrl.worldMatrix[0],
                                                    name=self.getName('ik_lower_aim_point'))
        ikLowerAimLen.output.connect(ikLowerAim.input1Y)
        ikLowerAimRef = mathOps.createTransformedPoint((0, 0, 0), self.ik_lower_ctrl.getParent().worldMatrix[0],
                                                        name=self.getName('ik_lower_aim_ref_point'))
        ikLowerAimLen.output.connect(ikLowerAimRef.input1Y)
        ikLowerDisplace = mathOps.subtractVector([ikLowerAim.output, ikLowerAimRef.output],
                                                 name=self.getName('ik_lower_displace'))
        ikUpperAimLen = mathOps.multiply(ikDist.distance, -0.2, name=self.getName('ik_upper_aim_len'))
        ikUpperAim = mathOps.createTransformedPoint((0, 0, 0), self.ik_upper_ctrl.worldMatrix[0],
                                                    name=self.getName('ik_upper_aim_point'))
        ikUpperAimLen.output.connect(ikUpperAim.input1Y)
        ikUpperAimRef = mathOps.createTransformedPoint((0, 0, 0), self.ik_upper_ctrl.getParent().worldMatrix[0],
                                                        name=self.getName('ik_Upper_aim_ref_point'))
        ikUpperAimLen.output.connect(ikUpperAimRef.input1Y)
        ikUpperDisplace = mathOps.subtractVector([ikUpperAim.output, ikUpperAimRef.output],
                                                 name=self.getName('ik_upper_displace'))

        ikLowerDisplace.output3D.connect(pb.inTranslate1)
        ikUpperDisplace.output3D.connect(pb.inTranslate2)
        midPosSum = mathOps.addVector([dm.outputTranslate, pb.outTranslate], name=self.getName('ik_mid_translate_sum'))
        midPosSum.output3D.connect(self.ik_mid_ctrl.getParent().t)
        dm = transform.decomposeMatrix(self.base_ctrl.worldMatrix[0], name=self.getName('fk_base_ctrl_mtx2Srt'))
        ikAimVec = mathOps.subtractVector([ikUpperAim.output, ikLowerAim.output], name=self.getName('ik_mid_aimVec'))
        ikAimVecOffset = mathOps.addVector([ikAimVec.output3D, dm.outputTranslate],
                                           name=self.getName('ik_mid_aimVec_offset'))
        ikAimVecLocal = mathOps.createTransformedPoint(ikAimVecOffset.output3D, self.base_ctrl.worldInverseMatrix[0],
                                                       name=self.getName('ik_aimVec_local'))
        ikAimVecNormal = mathOps.normalize(ikAimVecLocal.output, name=self.getName('ik_aimVec_normal'))
        ang = mathOps.angleBetween((0, 1, 0),ikAimVecLocal.output, name=self.getName('ik_mid_angle'))
        ikUpMtx = mathOps.createComposeMatrix(inputRotate=ang.euler, name=self.getName('ik_up_mtx'))
        ikUpVec = mathOps.createMatrixAxisVector(ikUpMtx.outputMatrix, (1, 0, 0), self.getName('ik_upVec'))
        ikSideVec = mathOps.createCrossProduct(ikUpVec.output, ikAimVecNormal.output, name=self.getName('ik_sideVec'))

        ikAimMtx = mathOps.vectors2Mtx44((ikUpVec.outputX, ikUpVec.outputY, ikUpVec.outputZ),
                                         (ikAimVecNormal.outputX, ikAimVecNormal.outputY, ikAimVecNormal.outputZ),
                                         (ikSideVec.outputX, ikSideVec.outputY, ikSideVec.outputZ),
                                         name=self.getName('ik_mid_mtx'))

        ikMidMtx = mathOps.multiplyMatrices([ikAimMtx.output, self.base_ctrl.worldMatrix[0]],
                                            name=self.getName('ik_mid_mtx'))

        dm = transform.decomposeMatrix(ikMidMtx.matrixSum, name=self.getName('ik_mid_mtx2Srt'))
        transform.connectSrt(dm, self.ik_mid_ctrl.getParent(), t=0)


        #####
        # Create matrix from cross products to drive mid buffer
        #####

        # IK Curve and rail curve
        points = [pm.xform(self.base_ctrl, q=1, ws=1, t=1)]


    def addSystems(self):
        pass



def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSpine01(guide)
