import pymel.core as pm

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
