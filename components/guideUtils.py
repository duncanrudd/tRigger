import pymel.core as pm
import os
import tRigger.components as tComponents
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

    compRoots = [node for node in pm.listRelatives(guideRoot, c=1, ad=1, s=0) if node.hasAttr('guide_type')]
    return compRoots

def findModuleFromCompType(comp_type):
    modulePath = os.path.join(os.path.dirname(__file__), comp_type)
    modulePath = os.path.join(os.path.dirname(__file__), comp_type)
    if os.path.isfile(os.path.join(modulePath, '__init__.py')):
        return modulePath
    else:
        return 'Unable to find the python module for component type: %s' % comp_type
    
def buildFromGuide(guideRoot=None, buildLevel='objects'):
    '''
    :param guideRoot: Instance of TGuide class
    :return: Dictionary representing all the rig components
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'
    guideDict = compileGuide(guideRoot)

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
            returnDict[cmpnt].addDeformers()
    for cmpnt in returnDict.keys():
        if buildDict[buildLevel] >= 5:
            returnDict[cmpnt].finish()
    return returnDict