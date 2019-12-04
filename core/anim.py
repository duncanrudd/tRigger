import pymel.core as pm
from tRigger.core import mathOps

reload(mathOps)

def easeCurve(input, name=None):
    '''
    creates an animCurve with keys at 0 and 1
    Source attr is normalized and used as time input to the animCurve
    returns the new animCurve node
    '''
    crv = pm.createNode('animCurveUU')
    if name:
        crv.rename(name)
    input.connect(crv.input)
    pm.setKeyframe(crv, float=float(0), value=float(0), itt='auto', ott='auto')
    pm.setKeyframe(crv, float=float(1), value=float(1), itt='auto', ott='auto')

    return crv

