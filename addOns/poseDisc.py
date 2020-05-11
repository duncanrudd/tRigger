import pymel.core as pm
import math

from tRigger.core import dag, attribute, mathOps, anim
reload(dag)
reload(attribute)
reload(mathOps)
reload(anim)

class TPoseDisc(object):
    def __init__(self, name, rig, host, ref, target, numPoses=4, axis='x'):
        '''
        Ouputs a 0-1 value for each pose on the edge of an imaginary disc based on the angle between ref and target
        vectors.
        Becomes part of 'host'.
        If 'ref' and 'target' are in different components, inputs are requested from those components in order to
        respect component boundaries.
        Args:
            name: (str) the name of the poseDisc
            rig: (dict) the post build description of the rig returned by guideUtils.buildFromGuide method
            host: (tRigger.components.TBaseComponent) the component class instance to which the poseDisc will be added
            ref: (pm.PyNode) the node against which to measure pose angles
            target: (pm.PyNode) the node to read pose angles from
            numPoses: (int) the number of points on the edge of the disc which we want to output values for
        Returns:
            (TPoseDisc) the newly created poseDisc class instance

        '''
        self.config = dag.addChild(host.rig, 'group', host.getName('%s_config' % name))
        self.result = dag.addChild(host.rig, 'group', host.getName('%s_result' % name))
        for i in range(numPoses):
            num = str(i+1).zfill(2)
            attribute.addDividerAttr(self.config, 'pose_%s' % num)
            attribute.addFloatAttr(self.config, 'pose_%s_falloff' % num, minValue=0, maxValue=180,
                                   value=(360.0/numPoses))
            attribute.addFloatAttr(self.config, 'pose_%s_start' % num, minValue=0, maxValue=1, value=0)
            attribute.addFloatAttr(self.config, 'pose_%s_end' % num, minValue=0, maxValue=1, value=1)
            attribute.addFloatAttr(self.config, 'pose_%s_ease' % num, minValue=0, maxValue=1, value=1)

        attribute.addDividerAttr(host.output, '%s_outputs' % name)
        for i in range(numPoses):
            num = str(i+1).zfill(2)
            attribute.addFloatAttr(self.result, 'pose_%s_weight' % (num), minValue=0, maxValue=1)

        refCmpnt = rig['_'.join(ref.name().split('_')[:2]) + '_comp']
        targetCmpnt = rig['_'.join(target.name().split('_')[:2]) + '_comp']

        refInput = ref.worldInverseMatrix[0]
        if not refCmpnt == host:
            output = refCmpnt.addOutput(ref)
            input = attribute.addMatrixAttr(host.input, '%s_ref_in_mtx' % name)
            output.connect(input)
            refInput = mathOps.inverseMatrix(input, name=host.getName('%s_ref_inverse_mtx' % name)).outputMatrix

        targetInput = target.worldMatrix[0]
        if not targetCmpnt == host:
            output = targetCmpnt.addOutput(target)
            targetInput = attribute.addMatrixAttr(host.input, '%s_target_in_mtx' % name)
            output.connect(targetInput)

        localTargetMtx = mathOps.multiplyMatrices([targetInput, refInput],
                                                  name=host.getName('%s_local_target_mtx' % name))

        # Create heading point (nearest point on circle to target's [y, z] vector
        # or [0, 0] if ref and target are aligned
        axisCheck = mathOps.getMatrixAxisAsVector(localTargetMtx.matrixSum, axis.lower())
        targ = (1, 0, 0)
        axisIndex = 0
        if 'y' in axis.lower():
            targ = (0, 1, 0)
            axisIndex = 1
        elif 'z' in axis.lower():
            targ = (0, 0, 1)
            axisIndex = 2
        if axisCheck[axisIndex] < 0.0:
            targ = (targ[0]*-1, targ[1]*-1, targ[2]*-1)
        targetAxis = mathOps.createMatrixAxisVector(localTargetMtx.matrixSum, targ,
                                                    name=host.getName('%s_target_axis' % name))
        targetProjection = mathOps.normalize(targ, name=host.getName('%starget_projection_vec' % name))

        if 'x' not in axis.lower():
            targetAxis.outputX.connect(targetProjection.input1X)
        if 'y' not in axis.lower():
            targetAxis.outputY.connect(targetProjection.input1Y)
        if 'z' not in axis.lower():
            targetAxis.outputZ.connect(targetProjection.input1Z)

        projectionCheck = pm.createNode('condition', name=host.getName('%s_target_projection_check' % name))
        dist = pm.createNode('distanceBetween', name=host.getName('%s_projection_check_dist' % name))

        sourceAttr = pm.general.Attribute('%s.output%s' % (targetAxis.name(), axis.upper()))
        destAttr = pm.general.Attribute('%s.point1%s' % (dist.name(), axis.upper()))
        sourceAttr.connect(destAttr)
        dist.point2.set((0, 0, 0))
        dist.distance.connect(projectionCheck.firstTerm)
        projectionCheck.secondTerm.set(1)
        projectionCheck.colorIfTrueR.set(1)
        projectionCheck.colorIfFalseR.set(0)
        destAttr = pm.general.Attribute('%s.input1%s' % (targetProjection.name(), axis.upper()))
        projectionCheck.outColorR.connect(destAttr)

        # Measure the angle between ref and target vectors and normalize the 0 - half pi radian range
        poseAngle = mathOps.angleBetween(targetAxis.output,
                                         (math.fabs(targ[0]), math.fabs(targ[1]), math.fabs(targ[2])),
                                         name=host.getName('%s_pose_angle' % name))
        poseAngleNormal = mathOps.convert(poseAngle.angle, 0.636537,
                                          name=host.getName('%s_pose_angle_normalized' % name))
        poseStrength = pm.createNode('condition', name=host.getName('%s_pose_strength' % name))
        sourceAttr = pm.general.Attribute('%s.output%s' % (targetAxis.name(), axis.upper()))
        sourceAttr.connect(poseStrength.firstTerm)
        poseStrength.operation.set(5)
        poseAngleNormal.output.connect(poseStrength.colorIfFalseR)
        poseStrength.colorIfTrueR.set(1)

        # Create normalized points to represent each pose required
        step = (math.pi * 2) / numPoses
        for i in range(numPoses):
            num = str(i+1).zfill(2)

            y = math.cos(step * i)
            z = math.sin(step * i)

            refAngle = (0, y, z)
            if 'y' in axis.lower():
                refAngle = (y, 0, z)
            elif 'y' in axis.lower():
                refAngle = (y, z, 0)
            angle = mathOps.angleBetween(targetProjection.output, refAngle,
                                         name=host.getName('%s_pose_%s_angle' % (name, num)))
            degrees = mathOps.convert(angle.angle, 57.29578, name=host.getName('%s_pose_%s_rad2Deg' % (name, num)))
            falloffAttr = pm.Attribute('%s.pose_%s_falloff' % (self.config.name(), num))
            poseMatch = mathOps.remap(degrees.output, 0.0, falloffAttr, 1.0, 0.0,
                                      name=host.getName('%s_pose_%s_match' % (name, num)))
            startAttr = pm.Attribute('%s.pose_%s_start' % (self.config.name(), num))
            endAttr = pm.Attribute('%s.pose_%s_end' % (self.config.name(), num))
            poseActivation = mathOps.remap(poseStrength.outColorR, startAttr, endAttr, 0.0, 1.0,
                                           name=host.getName('%s_pose_%s_strength' % (name, num)))
            poseSum = mathOps.multiply(poseMatch.outValueX, poseActivation.outValueX,
                                       name=host.getName('%s_pose_%s_sum' % (name, num)))
            poseEase = anim.easeCurve(poseSum.output, name=host.getName('%s_pose_%s_ease' % (name, num)))
            easeAttr = pm.Attribute('%s.pose_%s_ease' % (self.config.name(), num))
            poseResult = mathOps.blendScalarAttrs(poseSum.output, poseEase.output, easeAttr,
                                                  name=host.getName('%s_pose_%s_result' % (name, num)))
            resultAttr = pm.Attribute('%s.pose_%s_weight' % (self.result.name(), num))
            poseResult.output.connect(resultAttr)

