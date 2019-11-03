import pymel.core as pm
from maya.api import OpenMaya as om2

def addStringAttr(node, name, value=''):
    pm.addAttr(node, ln=name, dt="string")
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    return attr

def addIntAttr(node, name, value=0):
    pm.addAttr(node, ln=name, at='long')
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
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

def addAngleAttr(node, name, value=0, k=1, h=0):
    pm.addAttr(node, ln=name, at='doubleAngle', k=k, h=h)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    return attr

def addFloatAttr(node, name, value=0, k=1, h=0, minValue=None, maxValue=None):
    pm.addAttr(node, ln=name, at='float', k=k, h=h, minValue=minValue, maxValue=maxValue)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    attr.set(value)
    if minValue:
        pm.addAttr(attr, e=1, minValue=minValue)
    if maxValue:
        pm.addAttr(attr, e=1, maxValue=maxValue)
    return attr

def addEnumAttr(node, name, enumNames, k=1, h=0):
    pm.addAttr(node, ln=name, at='enum', enumName=':'.join(enumNames), k=k, h=h)
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
    for eachCB in om2.MMessage.nodeCallbacks(mob):
        om2.MMessage.removeCallback(eachCB)

    om2.MNodeMessage.addAttributeChangedCallback(mob, fn)




