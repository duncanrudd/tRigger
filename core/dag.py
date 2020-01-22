import pymel.core as pm

from tRigger.core import transform

def addChild(parent, childType, name, zero=1):
    '''
    adds a new node of type childType. Parents it to the parent node.
    :param childType: 'group', 'joint', 'locator'
    :return: newly created child node
    '''
    node = None
    if childType == 'group':
        node = pm.group(empty=1, name=name)
        node.setParent(parent)
    elif childType == 'locator':
        node = pm.spaceLocator(name=name)
        node.setParent(parent)
    elif childType == 'joint':
        node = pm.joint(name=name)
        if not node.getParent() == parent:
            node.setParent(parent)
    if node:
        if zero:
            transform.align(node, parent)
        return node
    else:
        return 'addChild: node not created'

def addParent(child, parentType, name, zero=1):
    '''
    adds a new node of type parentType. Parents node to it.
    :param childType: 'group', 'joint', 'locator'
    :return: newly created parent node
    '''
    node = None
    if not child:
        child = pm.selected()[0]

    parent = pm.listRelatives(child, p=1, fullPath=1)
    if type(parent) == type([]):
        if len(parent) > 0:
            parent = parent[0]

    if parentType == 'group':
        node = pm.group(empty=1, name=name)
    elif parentType == 'locator':
        node = pm.spaceLocator(name=name)

    if node:
        if zero:
            transform.align(node, child)
        if parent:
            node.setParent(parent)
        child.setParent(node)
        return node
    else:
        return 'addParent: node not created'

def setDisplayType(node, displayType):
    displayDict = {'normal': 0, 'template': 1, 'reference': 2}
    node.overrideEnabled.set(1)
    node.overrideDisplayType.set(displayDict[displayType])

def setOutlinerColour(node, colour):
    rgb = ("R", "G", "B")

    node.useOutlinerColor.set(1)

    for channel, value in zip(rgb, colour):
        pm.Attribute('%s.outlinerColor%s' % (node.name(), channel)).set(value)
