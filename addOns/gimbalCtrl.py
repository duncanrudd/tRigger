from tRigger.core import attribute, icon, transform
import pymel.core as pm

def gimbalCtrl(ctrl):

    dup = pm.duplicate(ctrl, name=ctrl.name().replace('_ctrl', '_gimbal_ctrl'))[0]
    pm.delete(pm.listRelatives(dup, type='transform'))

    children = pm.listRelatives(ctrl, type='transform')
    dup.setParent(ctrl)
    dup.inheritsTransform.set(1)
    attrList = ['tx', 'ty', 'tz', 'rx', 'ry', 'rx', 'sx', 'sy', 'sz', 'visibility']
    attribute.channelControl(nodeList=[dup],
                             attrList=attrList, lock=0, keyable=1)
    pm.xform(dup, ws=1, m=ctrl.worldMatrix[0].get())
    transform.bakeSrtToOffsetParentMtx(dup)
    dup.visibility.set(1)

    for child in children:
        child.setParent(dup)

    conns = pm.listConnections(ctrl, s=0, d=1, p=1, c=1)
    for conn in conns:
        if not 'controllerObject' in conn[1].name():
            swapAttr = pm.general.Attribute(conn[0].name().replace('_ctrl', '_gimbal_ctrl'))
            pm.disconnectAttr(conn[0], conn[1])
            swapAttr.connect(conn[1], f=1)
    if ctrl.hasAttr('meta_parent'):
        ctrl.message.connect(dup.meta_parent)
        meta_root = pm.listConnections(ctrl.meta_component_root, s=1, d=0, p=1)
        if meta_root:
            meta_root[0].connect(dup.meta_component_root)
        pm.controller(dup)
        tag = pm.controller(dup, q=1)
        parentTag = pm.controller(ctrl, q=1)
        if parentTag:
            tag, parentTag = pm.PyNode(tag[0]), pm.PyNode(parentTag[0])
            index = attribute.getNextAvailableIndex(parentTag.children)
            tag.parent.connect(pm.Attribute('%s.children[%s]' % (parentTag.name(), str(index))))
    attribute.addBoolAttr(ctrl, 'show_gimbal_ctrl')
    ctrl.show_gimbal_ctrl.set(0)
    shapes = icon.getShapes(dup)
    for shape in shapes:
        pm.select('%s.cv[*]' % shape.name())
        pm.scale((.8, .8, .8))
        ctrl.show_gimbal_ctrl.connect(shape.visibility)
    attribute.channelControl(lock=0, channelBox=1, nodeList=[ctrl], attrList=['show_gimbal_ctrl'])
    lockAttrs = []
    for attr in attrList:
        if pm.getAttr('%s.%s' % (ctrl.name(), attr), lock=1):
            lockAttrs.append(attr)
    if lockAttrs:
        attribute.channelControl(nodeList=[dup], attrList=lockAttrs)
    return dup
