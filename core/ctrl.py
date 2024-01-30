import pymel.core as pm

def disableCtrl(ctrl):
    '''
    Args:
        ctrl: (PyNode) or ([PyNodes]) the animation control(s) to disable

    Disables the isTCtrl attribute for the selected control(s)
    Sets visibility to off and locks visibility attr
    Removes control(s) from quick select sets
    '''
    if type(ctrl) == pm.nodetypes.Transform:
        ctrls = [ctrl]
    for c in ctrls:
        ctrl.is_tControl.set(0)
        pm.setAttr(ctrl.visibility, l=0)
        ctrl.visibility.set(0)
        pm.setAttr(ctrl.visibility, l=1)

        compName = '_'.join(ctrl.name().split('_')[:2])
        setName = '%s_ctrls_set' % compName

        pm.sets(ctrl, rm=pm.PyNode(setName))