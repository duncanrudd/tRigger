'''
Add joints to a rig which can accept multiple inputs, each driving the joint towards a defined target transform.

Workflow:
    Add drivenJoint specifying parent joint and optionally a number of targets (default=1 target)
    Joint is created under a buffer transform. Target locators are created under the same transform for each target.
    A weight attr is created on the buffer transform for each required pose. Connect the driver attrs to these.
    Move, scale, rotate the target locators to the desired pose.
    call the saveTargets function to serialize target data onto the buffer transform and delete the target locators.

    Editing:
        Call the editTargets function to re-create the target locators. Make the required changes and call
        the saveTargets function.

    Adding:
        Call the addTargets function and specify the number of targets to add. This will create locators for the new
        targets as well as target attrs on the buffer transform.
        Move, scale, rotate the target locators to the desired pose.
        Call the saveTargets function to serialize new target data and delete the target locators.

    Removing:
        call editTargets function to re-create target locators. Delete the locators for the targets you want to remove.
        call saveTargets to serialize the remaining targets and delete the target locators.
        Attrs on the buffer that don't have an associated locator when saveTargets is called will be removed.

    Saving data to disk:
        the buffer transform will have an 'is_tDrivenJoint' attribute (on by default at creation)
        collect nodes which have this attribute and populate a dictionary with keys for each node.
        Dictionary should store parent joint, driver attrs, poseNames, poseTargetSrts for each node.
        dump this dictionary to a json file.
'''

import pymel.core as pm
import json
import os
from tRigger.core import dag, mathOps, transform, attribute
reload(transform)

class TDrivenJoint(object):
    def __init__(self, name, parentJoint=None, numTargets=1, createLocs=1):
        object.__init__(self)
        self.numTargets = numTargets
        self.name = name
        if not parentJoint:
            parentJoint = pm.PyNode('joints')
        self.buffer = dag.addChild(parentJoint, 'group', name='%s_buffer_srt' % name)
        attribute.addStringAttr(self.buffer, 'targetDict')
        attribute.addBoolAttr(self.buffer, 'is_tDrivenJoint')
        j = dag.addChild(self.buffer, 'joint', name='%s_driven_jnt' % name)
        self.resultMtx = pm.createNode('multMatrix', name='%s_result_mtx' % name)
        self.resultMtx.matrixSum.connect(j.offsetParentMatrix)

        self.build()
        if createLocs:
            createTargetLocs(self.buffer)

    def build(self):
        # Add attrs to buffer
        for i in range(self.numTargets):
            num = str(i+1).zfill(2)
            attribute.addDividerAttr(self.buffer, 'target_%s' % num)
            inputAttr = attribute.addFloatAttr(self.buffer, 'target_%s_input' % num)
            startAttr = attribute.addFloatAttr(self.buffer, 'target_%s_start' % num)
            endAttr = attribute.addFloatAttr(self.buffer, 'target_%s_end' % num, value=1)
            interpAttr = attribute.addEnumAttr(self.buffer, 'target_%s_interpolation' % num,
                                               ['none', 'linear', 'smooth', 'spline'])
            interpAttr.set(1)
            mtxAttr = attribute.addMatrixAttr(self.buffer, 'target_%s_mtx' % num)

            # Add network to normalize and connect target inputs to target matrices
            norm = pm.createNode('remapValue', name='%s_target_%s_remap' % (self.name, num))
            interpAttr.connect(norm.value[0].value_Interp)
            startAttr.connect(norm.inputMin)
            endAttr.connect(norm.inputMax)
            inputAttr.connect(norm.inputValue)
            blend = transform.blendMatrices(pm.datatypes.Matrix(), mtxAttr, name='%s_target_%s_mtx' % (self.name, num))
            norm.outValue.connect(blend.target[0].weight)
            blend.outputMatrix.connect(self.resultMtx.matrixIn[i])

def createTargetLocs(buffer):
    targAttrs = [attr for attr in pm.listAttr(buffer) if '_mtx' in attr]
    for attrName in targAttrs:
        attr = pm.general.Attribute('%s.%s' % (buffer.name(), attrName))
        inputs = pm.listConnections(attr, s=1, d=0, type='transform')
        if not inputs:
            loc = dag.addChild(buffer, 'locator', name=attrName.replace('_mtx', '_temp_srt'))
            mtx = attr.get()
            loc.t.set(mtx.translate.get())
            loc.r.set(mtx.rotate.get())
            loc.s.set(mtx.scale.get())
            loc.matrix.connect(attr)

def serialize(buffer):
    '''
    Returns a dictionary with all the necessary data needed to rebuild the drivenJoint from an external file
    Args:
        buffer: the parent buffer of the driven joint
    Returns:
        (dictionary) data required to rebuild the drivenJoint
    '''
    parentMtx = buffer.matrix.get() * buffer.offsetParentMatrix.get()
    returnDict = {'name': buffer.name().replace('_buffer_srt', ''),
                  'parentJoint': buffer.getParent().name(),
                  'parentMtx': transform.mtx2List(parentMtx),
                  'targets': []}

    targAttrs = [attr for attr in pm.listAttr(buffer) if '_mtx' in attr]
    for index, attrName in enumerate(targAttrs):
        mtxAttr = pm.general.Attribute('%s.%s' % (buffer.name(), attrName))
        inputAttr = pm.general.Attribute('%s.%s' % (buffer.name(), attrName.replace('_mtx', '_input')))
        startAttr = pm.general.Attribute('%s.%s' % (buffer.name(), attrName.replace('_mtx', '_start')))
        endAttr = pm.general.Attribute('%s.%s' % (buffer.name(), attrName.replace('_mtx', '_end')))
        interpAttr = pm.general.Attribute('%s.%s' % (buffer.name(), attrName.replace('_mtx', '_interpolation')))
        driver = pm.listConnections(inputAttr, s=1, d=0)
        if driver:
            if type(driver[0]) == pm.nodetypes.UnitConversion:
                driver = pm.listConnections(driver[0].input, s=1, d=0, p=1)
                if driver:
                    driver = driver[0].name()
            else:
                driver = pm.listConnections(inputAttr, s=1, d=0, p=1)[0].name()

        returnDict['targets'].append({'driver': driver,
                                      'start': startAttr.get(),
                                      'end': endAttr.get(),
                                      'interp': interpAttr.get(),
                                      'matrix': transform.mtx2List(mtxAttr.get())})

    return returnDict

def buildFromDict(dict):
    dj = TDrivenJoint(name=dict['name'], parentJoint=pm.PyNode(dict['parentJoint']), numTargets=len(dict['targets']),
                      createLocs=0)
    dj.buffer.offsetParentMatrix.set(dict['parentMtx'])
    for index, target in enumerate(dict['targets']):
        num = str(index+1).zfill(2)
        mtxAttr = pm.general.Attribute('%s.target_%s_mtx' % (dj.buffer.name(), num))
        inputAttr = pm.general.Attribute('%s.target_%s_input' % (dj.buffer.name(), num))
        startAttr = pm.general.Attribute('%s.target_%s_start' % (dj.buffer.name(), num))
        endAttr = pm.general.Attribute('%s.target_%s_end' % (dj.buffer.name(), num))
        interpAttr = pm.general.Attribute('%s.target_%s_interpolation' % (dj.buffer.name(), num))

        mtxAttr.set(target['matrix'])
        if target['driver']:
            pm.general.Attribute(target['driver']).connect(inputAttr)
        startAttr.set(target['start'])
        endAttr.set(target['end'])
        interpAttr.set(target['interp'])

def mirror(buffer):
    '''
    creates a drivenJoint setup which tries to mirror the behaviour of the supplied buffer. If a mirrored setup already
    exists it will be modified to match the supplied buffer's setup
    '''
    # First check to see if a mirrored setup already exists.
    pass

def exportDrivenJoints():
    drivenJoints = [node for node in pm.ls(type='transform') if node.hasAttr('is_tDrivenJoint')]
    drivenJointsDict = {}
    for dj in drivenJoints:
        drivenJointsDict[dj.name()] = serialize(dj)
    fileName = pm.fileDialog2(fileFilter='*.json', caption='Export driven joints to file')
    if fileName:
        with open(fileName[0], 'w') as outfile:
            json.dump(drivenJointsDict, outfile, sort_keys=True, indent=4, separators=(',', ': '))
        return 'Driven Joints saved successfully to: %s' % fileName[0]
    return 'Saving driven joints cancelled'

def buildFromFile(filename):
    '''
    Loads a json file created via the exportDrivenJoints function
    and builds a drivenJoint setup for every key in the main dictionary
    '''
    if not os.path.exists(filename):
        return 'Driven Joints file not found: %s' % filename

    djDict = {}

    with open(filename) as json_file:
        djDict = json.load(json_file)

    for key in djDict.keys():
        buildFromDict(djDict[key])


'''
TODO:
    Mirroring / Matching opposite setups
    Look at integrating addOn into components so buffer and config are kept out of the joints hierarchy and component
    edges are preserved.
'''



