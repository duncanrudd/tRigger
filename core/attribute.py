import pymel.core as pm
from maya.api import OpenMaya as om2

def addMessageAttr(node, name):
    pm.addAttr(node, ln=name, at='message')
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addMultiMessageAttr(node, name):
    pm.addAttr(node, ln=name, at='message', multi=1)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addStringAttr(node, name, value=''):
    pm.addAttr(node, ln=name, dt="string")
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    return attr

def addIntAttr(node, name, value=0, minValue=None, maxValue=None):
    pm.addAttr(node, ln=name, at='long')
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    if minValue is not None:
        pm.addAttr(attr, e=1, minValue=minValue, hasMinValue=1)
    if maxValue is not None:
        pm.addAttr(attr, e=1, maxValue=maxValue, hasMaxValue=1)
    return attr

def addBoolAttr(node, name, value=True):
    pm.addAttr(node, ln=name, at='bool')
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    return attr

def addMatrixAttr(node, name):
    pm.addAttr(node, ln=name, at='matrix')
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addAngleAttr(node, name, value=0, k=1, h=0, minValue=None, maxValue=None):
    pm.addAttr(node, ln=name, at='doubleAngle', k=k, h=h)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    if minValue is not None:
        pm.addAttr(attr, e=1, minValue=minValue, hasMinValue=1)
    if maxValue is not None:
        pm.addAttr(attr, e=1, maxValue=maxValue, hasMaxValue=1)
    return attr

def addFloatAttr(node, name, value=0, k=1, h=0, minValue=None, maxValue=None):
    pm.addAttr(node, ln=name, at='float', k=k, h=h)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    if minValue is not None:
        pm.addAttr(attr, e=1, minValue=minValue, hasMinValue=1)
    if maxValue is not None:
        pm.addAttr(attr, e=1, maxValue=maxValue, hasMaxValue=1)
    return attr

def addEnumAttr(node, name, enumNames, k=1, h=0):
    pm.addAttr(node, ln=name, at='enum', enumName=':'.join(enumNames), k=k, h=h)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addColourAttr(node, name):
    pm.addAttr(node, ln=name, at='float3', uac=1)
    pm.addAttr(node, ln='%sR' % name, at='float', p='%s' % name)
    pm.addAttr(node, ln='%sG' % name, at='float', p='%s' % name)
    pm.addAttr(node, ln='%sB' % name, at='float', p='%s' % name)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addCallbackToAttr(mob, attr, fn):
    '''
    :param node: the target node
    :param attr: the name of the attribute to add the callback to
    :param fn: the function to call when the callback is triggered
    '''
    # Make sure mob is an MObject
    if not type(mob) == om2.MObject:
        ls = om2.MGlobal.getSelectionListByName(mob.name())
        mob = ls.getDependNode(0)

    # Remove existing callbacks from the node
    '''
    print 'Removing Callbacks:'
    for eachCB in om2.MMessage.nodeCallbacks(mob):
        print eachCB
        om2.MMessage.removeCallback(eachCB)
    '''

    return om2.MNodeMessage.addAttributeChangedCallback(mob, fn)

def removeAttributeCallback(mob, callback):
    '''
    Removes the specified callback from the MMobject
    Args:
        mob: (pyNode or MMobject) the node to remove the callback from
        callback: (int) the callback ID to remove

    Returns:
    '''
    # Make sure mob is an MObject
    if not type(mob) == om2.MObject:
        ls = om2.MGlobal.getSelectionListByName(mob.name())
        mob = ls.getDependNode(0)

    try:
        om2.MMessage.removeCallback(callback)
        print 'Attribute removed: %s' % callback
    except:
        print 'no callback found at ID: %s' % callback

def proxyAttribute(sourceAttr, node, alias=None):
    '''
    Creates a proxy of sourceAttr on node
    Args:
        sourceAttr: (pm.general.Attribute) Attribute to proxy
        node: (pm.PyNode) node for the proxy attribute to live on
    Returns:
        (pm.general.Attribute) The proxied attribute
    '''
    attrName=alias
    if attrName is None:
        attrName=str(sourceAttr).split('.')[1]
    pm.addAttr(node, ln=attrName, proxy=sourceAttr)

def channelControl(lock=True, keyable=False, channelBox=False, nodeList=[], attrList=[]):
    '''
    Takes a list of nodes and sets locks/unlocks shows/hides attributes in attrList
    '''
    if nodeList:
        for node in nodeList:
            if attrList:
                for a in attrList:
                    if node.hasAttr(a):
                        pm.setAttr('%s.%s' % (node, a), lock=lock, keyable=keyable, channelBox=channelBox)
            else:
                return 'attrCtrl: No nodes supplied for attribute control'
    else:
        return 'attrCtrl: No nodes supplied for attribute control'

def copyAttrValues(sourceNode, destNode, attrList):
    '''
    Copies the values of attributes from sourceNode to destNode
    Args:
        sourceNode: (pm.PyNode) node to copy values from
        destNode: (pm.PyNode) node to copy values to
        attrList: ([string]) names of attributes to copy
    Returns:
        None
    '''
    for attr in attrList:
        sourceAttr = pm.Attribute('%s.%s' % (sourceNode.name(), attr))
        destAttr = pm.Attribute('%s.%s' % (destNode.name(), attr))
        if pm.getAttr(destAttr, settable=1):
            destAttr.set(sourceAttr.get())
        else:
            print 'Skipping %s as it is either locked or has an incoming connection' % destAttr





