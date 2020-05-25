from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps, curve
import pymel.core as pm
reload(components)
reload(transform)
reload(curve)

import pymel.core as pm

class TElephantFoot(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'elephantFoot')
        print 'Created Control Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = (self.guide.guide_side == 'R')
        xform = pm.PyNode(guide.locs[-2]).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('neg_mtx'))
        self.roll_ctrl = self.addCtrl(shape='pringle', size=5.0, name=self.getName('roll'), xform=xform,
                                      parent=self.base_srt, metaParent=self.base_srt)
        xform = pm.PyNode(guide.locs[-1]).worldMatrix[0].get()
        self.floor_ctrl = self.addCtrl(shape='planeUp', size=5.0, name=self.getName('floor'), xform=xform,
                                      parent=self.base_srt, metaParent=self.base_srt)
        self.inner_crv = pm.duplicate(self.guide.locs[1], name=self.getName('inner_crv'))[0]
        self.inner_crv.setParent(self.rig)
        curve.rebuildUniform(self.inner_crv)
        self.outer_crv = pm.duplicate(self.guide.locs[2], name=self.getName('outer_crv'))[0]
        self.outer_crv.setParent(self.rig)
        curve.rebuildUniform(self.outer_crv)
        self.reader_crv = pm.circle(name=self.getName('reader_crv'), radius=1, normal=(0,1,0), ch=0)[0]
        self.reader_crv.setParent(self.rig)
        curve.rebuildUniform(self.reader_crv)

        # Add ctrls for perimeter plus srts and joints if required
        self.driven_srt = dag.addChild(self.base_srt, 'locator', name=self.getName('driven_srt'))
        self.edgeCtrls = []
        for index in range(self.guide.num_ctrls):
            num = str(index+1).zfill(2)
            param = (1.0 / self.guide.num_ctrls) * index
            pos = curve.sampleCurvePosition(self.outer_crv, param)
            zAxis = pm.datatypes.Vector((pos[0], 0.0, pos[2]))
            zAxis.normalize()
            xAxis = pm.datatypes.Vector(0, 1, 0).cross(zAxis)
            localXform = transform.list2Mtx(list(xAxis) + [0] + [0, 1, 0, 0] + list(zAxis) + [0] + list(pos) + [0])
            if self.invert:
                localXform = mathOps.invertHandedness(localXform)
            xform = localXform*self.base_srt.worldMatrix[0].get()
            ctrl = self.addCtrl(shape='triNorth', size=1.0, name=self.getName('edge_%s' % num), xform=xform,
                                parent=self.driven_srt, metaParent=self.base_srt, buffer=1)
            self.edgeCtrls.append(ctrl)

            if self.guide.add_joint:
                srt = dag.addChild(self.rig, 'group', name=self.getName('edge_%s_srt' % num))
                driver = ctrl.worldMatrix[0]
                if self.invert:
                    mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, driver],
                                                   name=self.getName('edge_%s_neg_mtx'))
                    driver = mtx.matrixSum
                driver.connect(srt.offsetParentMatrix)
                j = pm.createNode('joint', name=self.getName('edge_%s_jnt' % num))
                self.joints_list.append({'joint': j, 'driver': srt})
        components.TBaseComponent.addObjects(self, guide)
        self.ikEnd_srt = dag.addChild(self.rig, 'locator', name=self.getName('ikEnd_srt'))

    def addAttributes(self):
        attribute.addFloatAttr(self.params, 'slide_rate', minValue=0.0)
        attribute.addFloatAttr(self.params, 'align_depth', minValue=0.0)
        attribute.addAngleAttr(self.params, 'roll_start_angle', minValue=0.0, maxValue=180, value=30)
        attribute.addAngleAttr(self.params, 'roll_end_angle', minValue=0.0, maxValue=180, value=60)

    def addSystems(self):
        # Set up nearest point on reader crv based on rotation of roll ctrl
        roll_y_axis = mathOps.createMatrixAxisVector(self.roll_ctrl.matrix, (0, 1, 0), name=self.getName('roll_y_axis'))
        roll_projection = mathOps.normalize((0, .001, 0), name=self.getName('roll_projection'))
        roll_y_axis.outputX.connect(roll_projection.input1X)
        roll_y_axis.outputZ.connect(roll_projection.input1Z)
        roll_angle = mathOps.angleBetween(roll_y_axis.output, (0, 1, 0), name=self.getName('reader_roll_angle'))
        rollRemap = mathOps.remap(roll_angle.angle, self.params.roll_start_angle, self.params.roll_end_angle, 0, 1,
                                  name=self.getName('roll_remap'))
        reader_point = curve.createNearestPointOnCurve(self.reader_crv, roll_projection.output,#
                                                       name=self.getName('reader_nearest_point'))

        # Set up moving pivot for roll. Blends between inner and outer curves based on param of nearest point on reader crv
        inner_mp = curve.createMotionPathNode(self.inner_crv, uValue=reader_point.result.parameter, follow=0,
                                              name=self.getName('inner_mp'))
        outer_mp = curve.createMotionPathNode(self.outer_crv, uValue=reader_point.result.parameter, follow=0,
                                              name=self.getName('outer_mp'))
        roll_blend = mathOps.pairBlend(translateA=inner_mp.allCoordinates, translateB=outer_mp.allCoordinates,
                                       weight=rollRemap.outValueX, name=self.getName('roll_blend'))
        aim_angle = mathOps.angleBetween((0, 0, 1), (0, 0, 1), name=self.getName('reader_aim_angle'))
        roll_blend.outTranslateX.connect(aim_angle.vector2X)
        roll_blend.outTranslateZ.connect(aim_angle.vector2Z)
        aim_result = mathOps.addAngles(aim_angle.eulerY, 180, name=self.getName('reader_aim_result'))
        aim_switch = pm.createNode('condition', name=self.getName('reader_aim_switch'))
        reader_point.result.position.positionZ.connect(aim_switch.firstTerm)
        aim_switch.secondTerm.set(-1)
        aim_switch.colorIfTrueR.set(1)
        aim_switch.colorIfFalseR.set(0)
        aim_switch.outColorR.connect(aim_result.weightB)
        aim_switch_reverse = mathOps.reverse(aim_switch.outColorR, name=self.getName('reader_aim_switch_reverse'))
        aim_switch_reverse.outputX.connect(aim_result.weightA)

        # Create displacement
        pivot_mtx = mathOps.createComposeMatrix(inputTranslate=roll_blend.outTranslate, inputRotate=(0, 0, 0),
                                                name=self.getName('pivot_mtx'))
        aim_result.output.connect(pivot_mtx.inputRotateY)
        pivot_inverse_mtx = mathOps.inverseMatrix(pivot_mtx.outputMatrix, name=self.getName('pivot_inverse_mtx'))

        roll_mtx = mathOps.createComposeMatrix(inputTranslate=roll_blend.outTranslate, inputRotate=(0, 0, 0),
                                                name=self.getName('roll_mtx'))
        aim_result.output.connect(roll_mtx.inputRotateY)
        roll_angle.angle.connect(roll_mtx.inputRotateX)

        displace_mtx = mathOps.multiplyMatrices([pivot_inverse_mtx.outputMatrix, roll_mtx.outputMatrix],
                                                name=self.getName('displace_mtx'))
        displace_mtx.matrixSum.connect(self.ikEnd_srt.offsetParentMatrix)
        displace_mtx.matrixSum.connect(self.driven_srt.offsetParentMatrix)

        # Get collider normal (y axis of worldMAtrix)
        self.floorNormal = mathOps.createMatrixAxisVector(self.floor_ctrl.worldMatrix[0], (0, 1, 0),
                                                          name=self.getName('floor_normal'))
        for index, ctrl in enumerate(self.edgeCtrls):
            num = str(index + 1).zfill(2)
            self.collide(ctrl, num)

    def collide(self, ctrl, num):
        target = ctrl.getParent()
        # Get target position in space of collider object
        targetLocalMtx = mathOps.multiplyMatrices([target.worldMatrix[0], self.floor_ctrl.worldInverseMatrix[0]],
                                                  name=self.getName('target_%s_local_mtx' % num))
        targetLocalDM = mathOps.decomposeMatrix(targetLocalMtx.matrixSum,
                                                name=self.getName('target_%s_local_mtx2Srt' % num))
        # Clamp the Y output of that position to a maximum of 0.0
        targetYClamp = mathOps.clamp(targetLocalDM.outputTranslateY, -10000, 0.0,
                                     name=self.getName('target_%s_Y_clamp' % num))
        # Convert to absolute value (initial collision depth)
        initialDepth = mathOps.multiply(targetYClamp.outputR, -1, name=self.getName('%s_initial_depth' % num))
        # Multiply by 0.5 (slide amount)
        maxSlide = mathOps.multiply(initialDepth.output, self.params.slide_rate,
                                    name=self.getName('%s_slide_rate' % num))
        # Use as input1Z to get a point in the worldSpace of target object
        slidePoint = mathOps.createTransformedPoint((0, 0, 0), target.worldMatrix[0],
                                                    name=self.getName('%s_slide_point' % num))
        maxSlide.output.connect(slidePoint.input1Z)
        # Get output in space of collider object
        displacedDepth = mathOps.createTransformedPoint(slidePoint.output, self.floor_ctrl.worldInverseMatrix[0],
                                                        name=self.getName('%s_displaced_depth' % num))
        # clamp outputY to a maximum of 0.0 (displaced_collision_depth expr input)
        displacedYClamp = mathOps.clamp(displacedDepth.outputY, -10000, 0.0,
                                     name=self.getName('displacedDepth_%s_Y_clamp' % num))
        # Get target normal (y axis of worldMAtrix)
        targetNormal = mathOps.createMatrixAxisVector(target.worldMatrix[0], (0, 1, 0),
                                                      name=self.getName('target_%s_normal' % num))
        # get angle between target and collider normals
        collisionAngle = mathOps.angleBetween(targetNormal.output, self.floorNormal.output,
                                              name=self.getName('collision_%s_angle' % num))
        # add 90 degrees to the result
        collide90 = mathOps.addAngles(collisionAngle.angle, 90.0,
                                      name=self.getName('collision_%s_angle_plus90' % num))
        # invert result and add 180 degrees (thirdAngle expression input)
        thirdAngle = mathOps.addAngles(collide90.output, 180.0,
                                      name=self.getName('collision_%s_third_angle' % num))
        thirdAngle.weightA.set(-1)
        # get collider worldMatrix in target space
        floorLocalMtx = mathOps.multiplyMatrices([self.floor_ctrl.worldMatrix[0], target.worldInverseMatrix[0]],
                                                 name=self.getName('floor_%s_local_mtx' % num))
        # Get Y axis of result
        alignNormal = mathOps.createMatrixAxisVector(floorLocalMtx.matrixSum, (0, 1, 0),
                                                     name=self.getName('align_%s_normal' % num))
        # Get angle between result and world Y axis (collider align)
        alignAngle = mathOps.angleBetween((0, 1, 0), alignNormal.output, name=self.getName('align_%s_angle' % num))
        # set range of initial collision depth 0-MAX DEPTH >> 0-1
        alignWeight = mathOps.remap(initialDepth.output, 0, self.params.align_depth, 0, 1,
                                    name=self.getName('align_%s_weight' % num))
        # Use result as the weight in a rotation pairblend between collider align result and 0.0 (result rotation)
        alignBlend = mathOps.pairBlend(weight=alignWeight.outValueX,
                                       name=self.getName('align_%s_blend' % num))
        alignAngle.euler.connect(alignBlend.inRotate2)
        resultMtx = mathOps.createComposeMatrix(inputRotate=alignBlend.outRotate,
                                                name=self.getName('result_%s_mtx' % num))
        maxSlide.output.connect(resultMtx.inputTranslateZ)
        # Expression
        exprString =  '$ang = %s.output;\n' % thirdAngle.name()
        exprString += '$opp = %s.outputR;\n' % displacedYClamp.name()
        exprString += '$hyp = $opp / (sin($ang));\n'
        exprString += '%s.inputTranslateY = -$hyp' % resultMtx.name()

        expr = pm.expression(s=exprString, name=self.getName('collision_%s_expr' % num), alwaysEvaluate=0,
                             unitConversion='none')

        resultMtx.outputMatrix.connect(ctrl.offsetParentMatrix)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TElephantFoot(guide)