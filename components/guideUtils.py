import pymel.core as pm
import os, sys, json
import tRigger.components as tComponents
from tRigger.core import transform, attribute
reload(attribute)
reload(tComponents)

def validateGuideRoot(guideRoot):
    if not guideRoot or not guideRoot.hasAttr('is_tGuide'):
        sel = pm.selected()
        if sel:
            if sel[0].hasAttr('is_tGuide'):
                guideRoot = sel[0]
    return guideRoot

def compileGuide(guideRoot=None):
    '''
    :param guideRoot: Instance of TGuide class.
    :return: Dictionary representing all the guide components ready to be built
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'
    compRoots = findComponentRoots(guideRoot)
    if not compRoots:
        return 'Specified Guide contains no valid components'
    guideDict = {}
    for root in compRoots:
        compType = root.guide_type.get()
        guideDict[root.name()] = {}
        guideDict[root.name()]['module'] = findModuleFromCompType(compType)
        # Create python instances of all components by calling their 'instantiateFromDagNode' method
        exec("import tRigger.components.%s.guide as mod" % compType)
        reload(mod)
        guideDict[root.name()]['pObj'] = mod.instantiateFromDagNode(root)

    return guideDict

def findComponentRoots(guideRoot=None):
    '''
    :param guideRoot: Instance of TGuide class.
    :return: List of component root nodes in specified guide hierarchy
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'
    nodes = [guideRoot]
    nodes = nodes + pm.listRelatives(guideRoot, c=1, ad=1, s=0)
    compRoots = [node for node in nodes if node.hasAttr('guide_type')]
    return compRoots

def findModuleFromCompType(comp_type):
    modulePath = os.path.join(os.path.dirname(__file__), comp_type)
    modulePath = os.path.join(os.path.dirname(__file__), comp_type)
    if os.path.isfile(os.path.join(modulePath, '__init__.py')):
        return modulePath
    else:
        return 'Unable to find the python module for component type: %s' % comp_type

def findOppositeGuide(guideRoot):
    '''
    Finds the corresponding guide component on the opposite side of the guide
    Args:
        guideRoot: (pm.PyNode) Root node of the guide whose opposite we're interested in.

    Returns:
        (instance) instance of the opposite sided component guide or None if it doesn't exist.
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'
    guideTop = None
    parent = guideRoot
    while not guideTop:
        if parent.hasAttr('is_tGuide') or not parent.getParent():
            guideTop = parent
        else:
            parent = parent.getParent()
    if not guideTop:
        return None
    guide = compileGuide(guideTop)
    sourceGuide = guide[guideRoot.name()]
    sourceSide = sourceGuide['pObj'].guide_side
    sourceIndex = sourceGuide['pObj'].guide_index
    destGuide = None
    if sourceSide == 'L':
        try:
            return guide[guideRoot.name().replace('_L%s_' % sourceIndex, '_R%s_' % sourceIndex)]
        except:
            return None
    elif sourceSide == 'R':
        try:
            return guide[guideRoot.name().replace('_R%s_' % sourceIndex, '_L%s_' % sourceIndex)]
        except:
            return None


    
def buildFromGuide(guideRoot=None, buildLevel='objects'):
    '''
    :param guideRoot: Instance of TGuide class
    :return: Dictionary representing all the rig components
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'
    guideDict = compileGuide(guideRoot)

    # Check for customSteps
    customStepDict = None
    if guideRoot.hasAttr('custom_step_file'):
        if os.path.isfile(guideRoot.custom_step_file.get()):
            with open(guideRoot.custom_step_file.get()) as json_file:
                customStepDict = json.load(json_file)

    # Pre build scripts
    if customStepDict:
        cleanUpPath = 0
        thePath = customStepDict['pre_scripts']['path']
        if not thePath in sys.path:
            cleanUpPath = 1
            sys.path.append(thePath)

        for step in customStepDict['pre_scripts']['scripts']:
            evalString = 'import %s as tempStep' % step
            exec(evalString)
            reload(tempStep)
            if 'runStep' in dir(tempStep):
                tempStep.runStep()
            else:
                print 'No runStep method found in script: %s.py' % step
        if cleanUpPath:
            sys.path.remove(thePath)

    returnDict = {}

    buildDict = {'objects': 0, 'attributes': 1, 'systems': 2, 'connections': 3, 'deformers': 4, 'finish': 5}

    # Create rig instance
    rObj = tComponents.TRig(name='rig')

    for cmpnt in guideDict.keys():
        exec("import tRigger.components.%s as mod" % guideDict[cmpnt]['pObj'].guide_type)
        reload(mod)
        cObj = mod.build(guideDict[cmpnt]['pObj'])
        cObj.root.setParent(rObj.components)
        cObj.addObjects(guideDict[cmpnt]['pObj'])
        fullCompName = '%s_%s%s_comp' % (guideDict[cmpnt]['pObj'].guide_name,
                                         guideDict[cmpnt]['pObj'].guide_side,
                                         guideDict[cmpnt]['pObj'].guide_index)
        returnDict[fullCompName] = cObj

    for cmpnt in returnDict.keys():
        if buildDict[buildLevel] >= 1:
            returnDict[cmpnt].addAttributes()
    for cmpnt in returnDict.keys():
        if buildDict[buildLevel] >= 2:
            returnDict[cmpnt].addSystems()
    for cmpnt in returnDict.keys():
        if buildDict[buildLevel] >= 3:
            returnDict[cmpnt].addConnections(returnDict)
    for cmpnt in returnDict.keys():
        if buildDict[buildLevel] >= 4:
            returnDict[cmpnt].addDeformers(returnDict, rObj)
    for cmpnt in returnDict.keys():
        if buildDict[buildLevel] >= 5:
            returnDict[cmpnt].finish()
            rObj.root.hide_controls_on_playback.connect(returnDict[cmpnt].controls.hideOnPlayback)
            rObj.root.show_controls.connect(returnDict[cmpnt].controls.visibility)
            returnDict[cmpnt].rig.visibility.set(0)
            returnDict[cmpnt].addCtrlTags()
            rObj.root.message.connect(returnDict[cmpnt].root.meta_rig_root)
    for cmpnt in returnDict.keys():
        if buildDict[buildLevel] >= 5:
            returnDict[cmpnt].connectCtrlTags()
    rObj.root.show_joints.connect(rObj.joints.visibility)
    rObj.geo.overrideDisplayType.set(2)
    rObj.root.lock_geo.connect(rObj.geo.overrideEnabled)

    # Post build scripts
    if customStepDict:
        cleanUpPath = 0
        thePath = customStepDict['post_scripts']['path']
        if not thePath in sys.path:
            cleanUpPath = 1
            sys.path.append(thePath)

        for step in customStepDict['post_scripts']['scripts']:
            evalString = 'import %s as tempStep' % step
            exec(evalString)
            reload(tempStep)
            if 'runStep' in dir(tempStep):
                tempStep.runStep(returnDict)
            else:
                print 'No runStep method found in script: %s.py' % step
        if cleanUpPath:
            sys.path.remove(thePath)
    return returnDict

def duplicateGuide(guideRoot, includeChildGuides=0):
    '''
    Creates a duplicate of the specified guide.
    Args:
        guideRoot: (pm.PyNode) Root node of the guide to be duplicated.
        includeChildGuides: (bool) Whether or not to also duplicate child guides

    Returns:
        (instance) instance of the newly created guide.
    '''
    guideDict = compileGuide(guideRoot)

    for root in guideDict.keys():
        rootNode = pm.PyNode(root)
        if rootNode == guideRoot:
            compType = rootNode.guide_type.get()
            exec("import tRigger.components.%s.guide as mod" % compType)
            reload(mod)

            params = {}
            for param in guideDict[root]['pObj'].params:
                params[param] = pm.getAttr('%s.%s' % (guideRoot.name(), param))
            guide = mod.buildGuide(**params)
            guide.root.setParent(rootNode.getParent())

            for loc, refLoc in zip(guide.locs, guideDict[root]['pObj'].locs):
                attribute.copyAttrValues(refLoc, loc, ['t', 'r', 's', 'offsetParentMatrix'])
                if loc.hasAttr('spaces'):
                    attribute.copyAttrValues(refLoc, loc, ['spaces', 'splitTranslateAndRotate'])
            return guide

def mirrorGuide(guideRoot, includeChildGuides=0):
    '''
    If a matching opposite guide is found function will copy positions and params from guideRoot's component to it.
    If not. A duplicate will be made and mirrored.
    Args:
        guideRoot: (pm.PyNode) Root node of the guide to be duplicated.
        includeChildGuides: (bool) Whether or not to also duplicate child guides

    Returns:
        (instance) instance of the opposite sided component guide or the newly created mirrored component guide.
    '''
    if guideRoot.guide_side.get() == 'C':
        return "Can't mirror a centre guide"
    oppositeSide = 'L'
    if guideRoot.guide_side.get() == 'L':
        oppositeSide = 'R'
    oppositeGuide = findOppositeGuide(guideRoot)
    if not oppositeGuide:
        oppositeGuide = duplicateGuide(guideRoot, includeChildGuides=includeChildGuides)
        oppositeGuide.root.guide_side.set(oppositeSide)
        oppositeGuide.root.guide_index.set(guideRoot.guide_index.get())
        try:
            if oppositeSide == 'L':
                oppositeGuide.root.setParent(pm.PyNode(guideRoot.getParent().name().replace('R', 'L')))
            else:
                oppositeGuide.root.setParent(pm.PyNode(guideRoot.getParent().name().replace('L', 'R')))
        except:
            pass
    else:
        oppositeGuide = oppositeGuide['pObj']

    sourceGuide = instantiateGuide(guideRoot)

    for sourceLoc, destLoc in zip(sourceGuide.locs, oppositeGuide.locs):
        sourceMtx = transform.list2Mtx(pm.xform(sourceLoc, q=1, m=1))
        mirrorMtx = transform.getOppositeMatrix(sourceMtx)
        pm.xform(destLoc, m=mirrorMtx)

    for loc in oppositeGuide.locs:
        if loc.hasAttr('spaces'):
            if oppositeSide == 'L':
                loc.spaces.set(loc.spaces.get().replace('_R', '_L'))
            else:
                loc.spaces.set(loc.spaces.get().replace('_L', '_R'))

    params = [param for param in sourceGuide.params if param not in
              ['guide_name', 'guide_side', 'guide_index']]

    attribute.copyAttrValues(guideRoot, oppositeGuide.root, params)




def instantiateGuide(guideRoot):
    '''
    Creates an instance of the specified guide's class
    Args:
        guideRoot: (pm.PyNode) the node from which to instatiate
    Returns:
        (class instance) the python object representing the class instance
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'

    compType = guideRoot.guide_type.get()
    exec("import tRigger.components.%s.guide as mod" % compType)
    reload(mod)

    return mod.instantiateFromDagNode(guideRoot)

def addConnection(rig, source, dest):
    '''
    Creates required input / output matrix attrs on source and dest input groups respectively.
    Connects dest.offsetParentMatrix to input and source.worldMatrix[0] to output.
    Requires a tRig python object.

    Example usage:
        After rig build finishes, connect reverse foot to ik ankle
    Args:
        rig: (TRig) the dictionary describing the rig to act upon.
        source: (pm.PyNode) the driving node
        dest: (pm.PyNode) the driven node
    Returns:
        None
    '''
    sourceComp = rig['_'.join(source.name().split('_')[:2]) + '_comp']
    destComp = rig['_'.join(dest.name().split('_')[:2]) + '_comp']
    output = sourceComp.addOutput(source)
    input = attribute.addMatrixAttr(destComp.input, '%s_inMtx' % dest.name())
    output.connect(input)
    destComp.connectToInput(output, dest)

def extractControls(controls):
    '''
    Stores a copy of the supplied control objects in the guide
    Args:
        controls: (list) the controls whose shapes are to be stored
    Returns:
        None
    '''
    try:
        parent = pm.PyNode('guide|controllers')
    except:
        return 'No guide controllers group found'
    for control in controls:
        newName = control.name().replace('_ctrl', '') + '_stored'
        try:
            pm.delete(pm.PyNode(newName))
        except:
            pass
        transform = pm.createNode('transform', name=newName)
        transform.setParent(parent)
        shapes = control.getShapes()
        newShapes = pm.duplicate(*shapes, addShape=1)
        for shape in newShapes:
            pm.parent(shape, transform, s=1, r=1)

