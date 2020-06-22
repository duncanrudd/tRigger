import pymel.core as pm
from tRigger.core import transform, mathOps
reload(mathOps)

def rebuildUniform(crv, degree=3):
    pm.rebuildCurve(crv, ch=0, rpo=1, kr=0, kcp=1, d=degree)

def curveThroughPoints(name, positions=None, degree=3, bezier=0, rebuild=1):
    if not positions:
        positions = [pm.xform(p, q=1, ws=1, t=1) for p in pm.selected()]
    elif type(positions[0]) == pm.nodetypes.Transform:
        positions = [pm.xform(p, q=1, ws=1, t=1) for p in positions]

    if len(positions) < (degree + 1):
        return 'Please supply at least 4 points'

    # create the curve
    knots = range(len(positions) + degree - 1)

    crv = pm.curve(p=positions, k=knots, d=degree)
    if name:
        crv.rename(name)
    if rebuild:
        rebuildUniform(crv, degree)
    return crv

def curveBetweenPoints(start, end, numPoints=4, name=None, degree=3, rebuild=1):
    '''
    Creates a nurbs curve containing CVs between start and end
    Args:
        start: (PyNode) or (float3) start node or position
        end: (PyNode) or (float3) end node or position
        numPoints: (int) number of cvs on curve
        name: (string) name of the curve
        degree: degree of the curve

    Returns:
        the newly created curve node
    '''
    points = mathOps.getPointsAlongVector(start, end, numPoints)
    return curveThroughPoints(name, points, degree=degree, rebuild=rebuild)


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
            if type(wuo) == pm.general.Attribute:
                wuo.connect(mp.worldUpMatrix)
            else:
                wuo.worldMatrix[0].connect(mp.worldUpMatrix)
        if type(wu) == pm.general.Attribute:
            wu.connect(mp.worldUpVector)
        else:
            mp.worldUpVector.set(wu)
    return mp

def createCurveInfo(crv, name=None):
    node = pm.createNode('curveInfo')
    if name:
        node.rename(name)
    crv.worldSpace[0].connect(node.inputCurve)
    return node

def createPointOnCurve(crv, name=None):
    node = pm.createNode('pointOnCurveInfo')
    if name:
        node.rename(name)
    crv.worldSpace[0].connect(node.inputCurve)
    return node

def createCurveLength(crv, name=None):
    node = createCurveInfo(crv)
    if name:
        node.rename(name)
    return node

def getCurveLength(crv):
    node = createCurveLength(crv)
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

def nodesAlongCurve(start, end, divs, crv, name, frontAxis='x', upAxis='y', upVec=(0, 1, 0), wut=2,
                    startParam=0, endParam=1):
    '''
    Creates motionPath nodes along a curve. Up vectors are blended between start and end matrices
    Args:
        start: (pm.datatypes.Matrix) start matrix for up vectoring
        end: (pm.datatypes.Matrix) end matrix for up vectoring
        crv: (nurbsCurve) the curve along which to place nodes
        divs: (int) number of motionPath nodes to create
        name: (string) base name for newly created nodes
        frontAxis: (string) axis along which to aim
        upAxis: (string) axis to align to upVector
        upVec: (float3) matrix axis to use as upVector
        startParam: (float) point on the curve where the first node should sit
        endParam: (float) point on the curve where the last node should sit
    Returns:
        [motionPath] list of newly created motionPath nodes
    '''
    mps = []
    for i in range(divs):
        num = str(i+1).zfill(2)
        param = (((endParam - startParam) / (divs-1))*i) + startParam
        mp = createMotionPathNode(crv, param, frontAxis, upAxis, name='%s_%s_mp' % (name, num), wu=upVec, wut=wut)
        blend = transform.blendMatrices(start, end, name='%s_%s_up_mtx' % (name, num), weight=param)
        blend.outputMatrix.connect(mp.worldUpMatrix)
        mps.append(mp)
    return mps

def createNearestPointOnCurve(crv, point, name=None):
    '''
    Creates a nearestPointOnCurve node with the specified curve connected.
    Args:
        crv: (pm.PyNode(nurbsCurve)) the curve to get the nearest point on
        point: (pm.general.Attribute) The position to use as input
        name: (str) name of the newly created node
    Returns:
        (pm.PyNode(nearestPointOnCurve)) The newly created node
    '''
    node = pm.createNode('nearestPointOnCurve')
    if name:
        node.rename(name)
    if type(point) == pm.general.Attribute:
        point.connect(node.inPosition)
    else:
        node.inPosition.set(point)
    crv.worldSpace[0].connect(node.inputCurve)
    return node

def getNearestPointOnCurve(crv, point):
    '''
    Creates a temp nearestPointOnCurve node with the specified curve connected. Samples it's parameter value
    and then deletes it
    Args:
        crv: (pm.PyNode(nurbsCurve)) the curve to get the nearest point on
        point: (pm.general.Attribute) The position to use as input
        name: (str) name of the newly created node
    Returns:
        (float) the parameter value of the nearest point on the curve
    '''
    node = pm.createNode('nearestPointOnCurve')
    if type(point) == pm.nodetypes.Transform:
        point = pm.datatypes.Matrix(point.worldMatrix[0].get()).translate.get()
    node.inPosition.set(point)
    crv.worldSpace[0].connect(node.inputCurve)
    param = node.result.parameter.get()
    pm.delete(node)
    return param
