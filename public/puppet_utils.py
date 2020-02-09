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
