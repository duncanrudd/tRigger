import pymel.core as pm
import math
import json
import os
from tRigger.core import dag, attribute, mathOps, anim, transform
reload(dag)
reload(attribute)
reload(mathOps)
reload(anim)

class TPoseReader(object):
    def __init__(self, name, ref, target, numPoses=4, axis='x', createLocs=1):
        '''
        Ouputs a 0-1 value for each pose on the edge of an imaginary disc based on the angle between ref and target
        vectors.
        Args:
            name: (str) the name of the poseDisc
            ref: (pm.PyNode) the node against which to measure pose angles
            target: (pm.PyNode) the node to read pose angles from
            numPoses: (int) the number of points on the edge of the disc which we want to output values for
        Returns:
            (TPoseDisc) the newly created poseDisc class instance

        '''
        self.config = pm.createNode('transform', name='%s_poseReader' % name)

        attribute.addMessageAttr(self.config, 'ref')
        ref.message.connect(self.config.ref)
        attribute.addMessageAttr(self.config, 'target')
        target.message.connect(self.config.target)
        attribute.addMultiMessageAttr(self.config, 'poses')
        attribute.addStringAttr(self.config, 'axis', value=axis)
        attribute.addBoolAttr(self.config, 'is_tPoseReader')

        for i in range(numPoses):
            num = str(i+1).zfill(2)
            attribute.addDividerAttr(self.config, 'pose_%s' % num)
            attribute.addFloatAttr(self.config, 'pose_%s_falloff' % num, minValue=0, maxValue=360,
                                   value=(360.0/numPoses))
            attribute.addFloatAttr(self.config, 'pose_%s_start' % num, minValue=0, maxValue=1, value=0)
            attribute.addFloatAttr(self.config, 'pose_%s_end' % num, minValue=0, maxValue=1, value=1)
            attribute.addFloatAttr(self.config, 'pose_%s_ease' % num, minValue=0, maxValue=1, value=1)


        attribute.addDividerAttr(self.config, 'Results')
        for i in range(numPoses):
            num = str(i+1).zfill(2)
            attribute.addFloatAttr(self.config, 'pose_%s_weight' % num, minValue=0, maxValue=1)

        refInput = ref.worldInverseMatrix[0]
        targetInput = target.worldMatrix[0]
        localTargetMtx = mathOps.multiplyMatrices([targetInput, refInput], name='%s_local_target_mtx' % name)
        ref.worldMatrix[0].connect(self.config.offsetParentMatrix)

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
                                                    name='%s_target_axis' % name)
        targetProjection = mathOps.normalize(targ, name='%starget_projection_vec' % name)

        if 'x' not in axis.lower():
            targetAxis.outputX.connect(targetProjection.input1X)
        if 'y' not in axis.lower():
            targetAxis.outputY.connect(targetProjection.input1Y)
        if 'z' not in axis.lower():
            targetAxis.outputZ.connect(targetProjection.input1Z)

        projectionCheck = pm.createNode('condition', name='%s_target_projection_check' % name)
        dist = pm.createNode('distanceBetween', name='%s_projection_check_dist' % name)

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
                                         name='%s_pose_angle' % name)
        poseAngleNormal = mathOps.convert(poseAngle.angle, 0.636537,
                                          name='%s_pose_angle_normalized' % name)
        poseStrength = pm.createNode('condition', name='%s_pose_strength' % name)
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
                                         name='%s_pose_%s_angle' % (name, num))
            angle.message.connect(self.config.poses[i])
            degrees = mathOps.convert(angle.angle, 57.29578, name='%s_pose_%s_rad2Deg' % (name, num))
            falloffAttr = pm.Attribute('%s.pose_%s_falloff' % (self.config.name(), num))
            poseMatch = mathOps.remap(degrees.output, 0.0, falloffAttr, 1.0, 0.0,
                                      name='%s_pose_%s_match' % (name, num))
            startAttr = pm.Attribute('%s.pose_%s_start' % (self.config.name(), num))
            endAttr = pm.Attribute('%s.pose_%s_end' % (self.config.name(), num))
            poseActivation = mathOps.remap(poseStrength.outColorR, startAttr, endAttr, 0.0, 1.0,
                                           name='%s_pose_%s_strength' % (name, num))
            poseSum = mathOps.multiply(poseMatch.outValueX, poseActivation.outValueX,
                                       name='%s_pose_%s_sum' % (name, num))
            poseEase = anim.easeCurve(poseSum.output, name='%s_pose_%s_ease' % (name, num))
            easeAttr = pm.Attribute('%s.pose_%s_ease' % (self.config.name(), num))
            poseResult = mathOps.blendScalarAttrs(poseSum.output, poseEase.output, easeAttr,
                                                  name='%s_pose_%s_result' % (name, num))
            resultAttr = pm.Attribute('%s.pose_%s_weight' % (self.config.name(), num))
            poseResult.output.connect(resultAttr)

        if createLocs:
            createTargetLocs(self.config)

def createTargetLocs(config):
    poseNodes = pm.listConnections(config.poses, s=1, d=0)
    for node in poseNodes:
        inputs = pm.listConnections(node.vector2, s=1, d=0)
        if not inputs:
            loc = dag.addChild(config, 'locator', name=node.name().replace('_angle', '_temp_srt'))
            loc.t.set(node.vector2.get())
            loc.t.connect(node.vector2)

def serialize(config):
    '''
    Returns a dictionary with all the necessary data needed to rebuild the poseReader from an external file
    Args:
        config: the poseReader dag node
    Returns:
        (dictionary) data required to rebuild the poseReader
    '''
    returnDict = {'name': config.name().replace('_poseReader', ''),
                  'ref': config.ref.get().name(),
                  'axis': config.axis.get(),
                  'target': config.target.get().name(),
                  'targets': []}

    poseNodes = pm.listConnections(config.poses, s=1, d=0)
    for index, node in enumerate(poseNodes):
        num = str(index+1).zfill(2)
        falloffAttr = pm.general.Attribute('%s.pose_%s_falloff' % (config.name(), num))
        startAttr = pm.general.Attribute('%s.pose_%s_start' % (config.name(), num))
        endAttr = pm.general.Attribute('%s.pose_%s_end' % (config.name(), num))
        easeAttr = pm.general.Attribute('%s.pose_%s_ease' % (config.name(), num))

        returnDict['targets'].append({'poseVec': tuple(node.vector2.get()),
                                      'start': startAttr.get(),
                                      'end': endAttr.get(),
                                      'ease': easeAttr.get(),
                                      'falloff': falloffAttr.get()})
    return returnDict

def buildFromDict(dict):
    pr = TPoseReader(name=dict['name'], ref=pm.PyNode(dict['ref']), target=pm.PyNode(dict['target']),
                     numPoses=len(dict['targets']), axis=dict['axis'], createLocs=0)
    poseNodes = pm.listConnections(pr.config.poses, s=1, d=0)

    for index, target in enumerate(dict['targets']):
        num = str(index+1).zfill(2)
        falloffAttr = pm.general.Attribute('%s.pose_%s_falloff' % (pr.config.name(), num))
        startAttr = pm.general.Attribute('%s.pose_%s_start' % (pr.config.name(), num))
        endAttr = pm.general.Attribute('%s.pose_%s_end' % (pr.config.name(), num))
        easeAttr = pm.general.Attribute('%s.pose_%s_ease' % (pr.config.name(), num))

        falloffAttr.set(target['falloff'])
        startAttr.set(target['start'])
        endAttr.set(target['end'])
        easeAttr.set(target['ease'])
        poseNodes[index].vector2.set(target['poseVec'])

    return pr

def exportPoseReaders():
    poseReaders = [node for node in pm.ls(type='transform') if node.hasAttr('is_tPoseReader')]
    poseReadersDict = {}
    for pr in poseReaders:
        poseReadersDict[pr.name()] = serialize(pr)
    fileName = pm.fileDialog2(fileFilter='*.json', caption='Export Pose Readers to file')
    if fileName:
        with open(fileName[0], 'w') as outfile:
            json.dump(poseReadersDict, outfile, sort_keys=True, indent=4, separators=(',', ': '))
        return 'Pose Readers saved successfully to: %s' % fileName[0]
    return 'Saving pose readers cancelled'

def buildFromFile(filename):
    '''
    Loads a json file created via the exportPoseReaders function
    and builds a poseReader setup for every key in the main dictionary
    '''
    if not os.path.exists(filename):
        return 'Pose Readers file not found: %s' % filename

    prDict = {}

    with open(filename) as json_file:
        prDict = json.load(json_file)

    poseReaders = []

    for key in prDict.keys():
        poseReaders.append(buildFromDict(prDict[key]))

    return poseReaders


