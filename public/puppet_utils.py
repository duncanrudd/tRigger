import pymel.core as pm
import json

def setDefaults(node):
    '''
    Stores the current values of all keyable attributes on the supplied node as a json encoded dictionary string to
     the node's 'defaults' attribute
    Args:
        node: (pm.PyNode) The node whose attributes are to be stored
    Returns:
        None
    '''
    attrs = pm.listAttr(node, k=1)
    attrDict = {}
    for attr in attrs:
        a = pm.Attribute('%s.%s' % (node.name(), attr))
        attrDict[attr] = a.get()
    node.defaults.set(json.dumps(attrDict))

def getDefaults(node):
    '''
    Loads the node's 'defaults' string attr via json and parses it to a dict object
    Args:
        node: (pm.PyNode) The node whose defaults to get

    Returns:
        (dict) The dictionary of attribute name / value pairs
    '''
    return json.loads(node.defaults.get())

def restoreDefaults(node, selected=0):
    '''
    Sets all attributes with keys in the node's 'defaults' dict to their stored values
    Args:
        node: (pm.PyNode) the node whose defaults to restore
        selected: (bool) if true, only restores the selected channels in the channel box

    Returns:
        None
    '''
    if pm.hasAttr(node, 'defaults'):
        attrDict = getDefaults(node)
        keys = attrDict.keys()
        if selected:
            channels = [channel for channel in pm.channelBox('mainChannelBox', q=1, selectedMainAttributes=1)
                        if pm.hasAttr(node, channel)]
            keys = [pm.listAttr('%s.%s' % (node.name(), channel))[0] for channel in channels]
        for key in keys:
            attr = pm.Attribute('%s.%s' % (node.name(), key))
            attr.set(attrDict[key])

def getComponentRoot(node):
    if pm.hasAttr(node, 'is_tControl'):
        return pm.listConnections(node.meta_component_root)[0]

def getComponentCtrls(node):
    if pm.hasAttr(node, 'meta_rig_root'):
        root = node
    elif pm.hasAttr(node, 'is_tControl'):
        root = getComponentRoot(node)
    if root:
        return [pm.PyNode(conn.split('.')[0]) for conn in pm.listConnections(root.message, d=1, s=0, p=1)
                if conn.split('.')[1] == 'meta_component_root']

def getCtrlIndex(node):
    if pm.hasAttr(node, 'is_tControl'):
        ctrls = getComponentCtrls(node)
        return ctrls.index(node)

def getComponents(node):
    if pm.hasAttr(node, 'rig_name'):
        return [pm.PyNode(conn.split('.')[0]) for conn in pm.listConnections(node.message, d=1, s=0, p=1)
               if conn.split('.')[1] == 'meta_rig_root']


def getOppositeControl(node):
    '''
    Returns the corresponding control on the other side of the rig
    Args:
        node: (pm.PyNode) the node whose opposite we're interested in

    Returns:
        (pm.PyNode) the opposite control node
    '''
    if pm.hasAttr(node, 'is_tControl'):
        targetSide = 'L'
        cmpntRoot = getComponentRoot(node)
        if cmpntRoot.comp_side.get() == 'L':
            targetSide = 'R'
        elif cmpntRoot.comp_side.get() == 'C':
            return node
        rigRoot = pm.listConnections(cmpntRoot.meta_rig_root)[0]
        components = getComponents(rigRoot)
        oppositeCmpnt = [c for c in components if
                         c.comp_type.get() == cmpntRoot.comp_type.get() and
                         c.comp_name.get() == cmpntRoot.comp_name.get() and
                         c.comp_index.get() == cmpntRoot.comp_index.get() and
                         c.comp_side.get() == targetSide]
        if oppositeCmpnt:
            oppositeCmpnt = oppositeCmpnt[0]
        cmpntChildren = getComponentCtrls(oppositeCmpnt)
        ctrlIndex = getCtrlIndex(node)
        return cmpntChildren[ctrlIndex]
    else:
        return 'please select a valid control node'

def getAllControlsBelow(node):
    '''
    Returns all controls downstream from the node
    Args:
        node: (pm.PyNode) the node whose descendants we are interested in

    Returns:
        ([pm.PyNode]) List of downstream nodes (including node)
    '''
    metaChildren = []

    def _getAllMetaChildrenRecurse(node):
        mc = getControlsBelow(node)
        if mc:
            for n in mc:
                _getAllMetaChildrenRecurse(n)
        metaChildren.append(node)

    mc = getControlsBelow(node)
    for n in mc:
        _getAllMetaChildrenRecurse(n)

    return

def getControlsBelow(node=None):
    '''
    Returns all controls for whom node is meta_parent
    Args:
        node: (pm.PyNode) the node whose child ctrls we are interested in

    Returns:
        ([pm.PyNode]) List of child nodes
    '''
    metaChildren = [pm.PyNode(conn.split('.')[0]) for conn in pm.listConnections(node.message, d=1, s=0, p=1)
                    if conn.split('.')[1] == 'meta_parent']
    if not metaChildren:
        metaChildren=[]

    return metaChildren
