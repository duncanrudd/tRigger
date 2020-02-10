import pymel.core as pm

def parentConnect(parent=None, child=None):
    '''
    Specific method for connecting parent / child relationships in a metarig

    '''
    #Validation
    if not parent or not child and (len(pm.selected()) == 2):
        parent = pm.selected()[0]
        child = pm.selected()[1]

    if not parent or not child:
        return 'Argument Error, Please supply a parent and child node'

    if parent in getAllMetaChildren(child):
        return '%s is a meta descendant of %s and cannot also be its parent' % (parent, child)

    # Connect to new parent
    parent.message.connect(child.meta_parent, f=1)

def getMetaChildren(node=None):
    '''
    returns a list of all metaChildren of Node
    '''
    if not node and len(pm.selected()) == 1:
        node = pm.selected()[0]
    if not node:
        return 'Please supply a node whose children you wish to list'

    metaChildren = [pm.PyNode(conn.split('.')[0]) for conn in pm.listConnections(node.message, d=1, s=0, p=1)
                    if conn.split('.')[1] == 'meta_parent']
    if not metaChildren:
        metaChildren=[]

    return metaChildren

def getAllMetaChildren(node=None):
    '''
    returns a list of all metaDescendants of Node
    '''
    if not node and len(pm.selected()) == 1:
        node = pm.selected()[0]
    if not node:
        return 'Please supply a node whose descendants you wish to list'

    metaChildren = []

    def _getAllMetaChildrenRecurse(node):
        mc = getMetaChildren(node)
        if mc:
            for n in mc:
                _getAllMetaChildrenRecurse(n)
        metaChildren.append(node)

    mc = getMetaChildren(node)
    for n in mc:
        _getAllMetaChildrenRecurse(n)

    return metaChildren

def getComponentControls(root):
    '''
    Returns a list of all the nodes whose 'meta_component_root' attr is connected to root's message attr
    Args:
        root: (pm.PyNode) the dag root of the component
    Returns:
        [] List of controls that belong to the component
    '''
    controls = [pm.PyNode(conn.split('.')[0]) for conn in pm.listConnections(root.message, s=0, d=1, p=1)
                if conn.split('.')[1] == 'meta_component_root']
    return controls

def getMetaParent(node):
    '''
    Returns the node whose message attr is connected to node's meta_parent_attr
    Args:
        node: (pm.PyNode) the node whose parent we want to get

    Returns:
        (pm.PyNode) the parent node
    '''
    parent = pm.listConnections(node.meta_parent, s=1, d=0)
    if parent:
        return parent[0]
    else:
        return None