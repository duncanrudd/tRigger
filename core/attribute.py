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

def addMatrixAttr(node, name):
    pm.addAttr(node, ln=name, at='matrix')
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addAngleAttr(node, name, value=0, k=1, h=0):
    pm.addAttr(node, ln=name, at='doubleAngle', k=k, h=h)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addFloatAttr(node, name, value=0, k=1, h=0):
    pm.addAttr(node, ln=name, at='float', k=k, h=h)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

def addEnumAttr(node, name, enumNames, k=1, h=0):
    pm.addAttr(node, ln=name, at='enum', enumName=':'.join(enumNames), k=k, h=h)
    attr = pm.Attribute('%s.%s' % (node.name(), name))
    return attr

