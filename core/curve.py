import pymel.core as pm
from tRigger.core import transform

def curveThroughPoints(name, positions=None, degree=3, bezier=0, rebuild=1):
    if not positions:
        positions = [pm.xform(p, q=1, ws=1, t=1) for p in pm.selected()]
    elif type(positions[0]) == pm.nodetypes.Transform:
        positions = [pm.xform(p, q=1, ws=1, t=1) for p in positions]

    if len(positions) < (degree + 1):
        return 'Please supply at least 4 points'

    # create the curve
    knots = range(len(positions) + degree - 1)

    crv = pm.curve(p=positions, k=knots, d=degree, name=name)
    if rebuild:
        pm.rebuildCurve(crv, ch=0, rpo=1, kr=0, kcp=1, d=degree)
    return crv

def driveCurve(crv, drivers):
    for i in range(len(drivers)):
        d = transform.decomposeMatrix(drivers[i].worldMatrix[0],
                                      name='%s_cv_%s_mtx2Srt_utl' % (crv.name(), str(i)))
        d.outputTranslate.connect(crv.controlPoints[i])

def createMotionPathNode(crv, uValue=0, frontAxis='x', upAxis='y', fractionMode=1,
                         follow=1, wut=3, name=None, wuo=None, wu=(0, 1, 0)):
    mp = pm.createNode('motionPath')
    if name:
        mp.rename(name)
    crv.worldSpace[0].connect(mp.geometryPath)
    if type(uValue) == pm.general.Attribute:
        uValue.connect(mp.uValue)
    else:
        mp.uValue.set(uValue)
    mp.fractionMode.set(fractionMode)
    if follow:
        axisDict = {'x': 0, 'y': 1, 'z': 2, '-x': 0, '-y': 1, '-z': 2}
        mp.follow.set(1)
        mp.frontAxis.set(axisDict[frontAxis])
        if '-' in frontAxis:
            mp.inverseFront.set(1)
        mp.upAxis.set(axisDict[upAxis])
        if '-' in upAxis:
            mp.inverseUp.set(1)
        mp.worldUpType.set(wut)
        if wuo:
            wuo.worldMatrix[0].connect(mp.worldUpMatrix)
        mp.worldUpVector.set(wu)
    return mp

def createCurveInfo(crv, name=None):
    node = pm.createNode('curveInfo')
    if name:
        node.rename(name)
    crv.worldSpace[0].connect(node.inputCurve)
    return node

def getCurveLength(crv):
    node = createCurveInfo(crv)
    crvLength = node.arcLength.get()
    pm.delete(node)
    return crvLength

def scaleCVs(nodes, scale):
    if type(nodes) == pm.nodetypes.Transform:
        nodes = pm.listRelatives(nodes, c=1, s=1)
    if type(nodes) != type([]):
        nodes = [nodes]
    for node in nodes:
        pm.select('%s.cv[:]' % node.name())
        pm.scale(scale, scale, scale, objectSpace=1)

def sampleCurvePosition(crv, uValue, fractionMode=1):
    mp = createMotionPathNode(crv, uValue, fractionMode=fractionMode)
    pos = mp.allCoordinates.get()
    pm.delete(mp)
    return pos
