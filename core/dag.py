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