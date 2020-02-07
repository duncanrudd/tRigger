import pymel.core as pm
import pymel.core.datatypes as dt
import pymel.util as pmUtil
import math
from tRigger.core import transform


# --------------------------------------------------------------------------------
# Arithmetic
# --------------------------------------------------------------------------------

def addScalar(inputs, name=None, operation=1):
    node = pm.createNode('plusMinusAverage')
    node.operation.set(operation)
    if name:
        node.rename(name)
    for index, input in zip(range(len(inputs)), inputs):
        if type(input) == pm.general.Attribute:
            input.connect(node.input1D[index])
        else:
            node.input1D[index].set(input)
    return node

def subtractScalar(inputs, name=None):
    return addScalar(inputs, name, operation=2)

def addVector(inputs, name=None, operation=1):
    node = pm.createNode('plusMinusAverage')
    node.operation.set(operation)
    if name:
        node.rename(name)
    for index, input in zip(range(len(inputs)), inputs):
        if type(input) == pm.general.Attribute:
            input.connect(node.input3D[index])
        else:
            node.input3D[index].set(input)
    return node

def subtractVector(inputs, name=None):
    return addVector(inputs, name, operation=2)

def averageVector(inputs, name=None):
    return addVector(inputs, name, operation=3)

def reverse(input, name=None):
    node = pm.createNode('reverse')
    if name:
        node.rename(name)
    if type(input) == pm.general.Attribute:
        input.connect(node.inputX)
    else:
        node.inputX.set(input)
    return node

def addAngles(inputA, inputB, name=None):
    '''
    creates an animBlendNodeAdditiveDA node and connects input1 and input2
    '''
    node = pm.createNode('animBlendNodeAdditiveDA')
    if name:
        node.rename(name)
    if type(inputA) == pm.general.Attribute:
        inputA.connect(node.inputA)
    else:
        node.inputA.set(inputA)

    if type(inputB) == pm.general.Attribute:
        inputB.connect(node.inputB)
    else:
        node.inputB.set(inputB)
    return node

def blendAngles(inputA, inputB, weightA, weightB, name=None):
    '''
    calls addAngles function above and adds options to control weights of inputs
    '''
    node = addAngles(inputA, inputB, name)
    if type(weightA) == pm.general.Attribute:
        weightA.connect(node.weightA)
    else:
        node.weightA.set(weightA)

    if type(weightB) == pm.general.Attribute:
        weightB.connect(node.weightB)
    else:
        node.weightB.set(weightB)
    return node

def pairBlend(translateA=None, translateB=None, rotateA=None, rotateB=None, weight=0.5, quatBlend=0, name=None):
    node = pm.createNode('pairBlend')
    if name:
        node.rename(name)
    if translateA and translateB:
        if type(translateA) == pm.general.Attribute:
            translateA.connect(node.inTranslate1)
        else:
            node.inTranslate1.set(translateA)

        if type(translateB) == pm.general.Attribute:
            translateB.connect(node.inTranslate2)
        else:
            node.inTranslate2.set(translateB)

    if rotateA and rotateB:
        if type(rotateA) == pm.general.Attribute:
            rotateA.connect(node.inRotate1)
        else:
            node.inRotate1.set(rotateA)

        if type(rotateB) == pm.general.Attribute:
            rotateB.connect(node.inRotate2)
        else:
            node.inRotate2.set(rotateB)
        if quatBlend:
            node.rotInterpolation.set(1)
    if type(weight) == pm.general.Attribute:
        weight.connect(node.weight)
    else:
        node.weight.set(weight)
    return node

def multiplyAngleByScalar(input, weight, name=None):
    '''
    creates an animBlendNodeAdditiveDA node and connects input1 with weightA as multiplier
    '''
    node = pm.createNode('animBlendNodeAdditiveDA')
    if name:
        node.rename(name)
    if type(input) == pm.general.Attribute:
        input.connect(node.inputA)
    else:
        node.inputA.set(input)
    if type(weight) == pm.general.Attribute:
        weight.connect(node.weightA)
    else:
        node.weightA.set(weight)

    return node

def getAngleBetweenVectors(v1, v2, vUp, degrees=1):
    '''
    returns the signed angle between v1 and v2
    '''
    if not type(v1) == dt.Vector:
        v1 = dt.Vector(v1)
    if not type(v2) == dt.Vector:
        v2 = dt.Vector(v2)
    if not type(vUp) == dt.Vector:
        vUp = dt.Vector(vUp)
    angle = v1.angle(v2)
    dot = vUp.dot(v2)
    if dot < 0:
        angle *= -1
    if degrees:
        angle = pmUtil.degrees(angle)
    return angle

def multiply(input1, input2, name=None):
    mult = pm.createNode('multDoubleLinear')
    if name:
        mult.rename(name)
    if type(input1) == pm.general.Attribute:
        val = input1.get()
        connect=True
    else:
        val = input1
        connect=False

    if connect:
        input1.connect(mult.input1)
    else:
        mult.input1.set(input1)

    if type(input2) == pm.general.Attribute:
        val = input2.get()
        connect=True
    else:
        val = input2
        connect=False

    if connect:
        input2.connect(mult.input2)
    else:
        mult.input2.set(input2)
    return mult

def multiplyVector(input1, input2, name=None, operation=1):
    md = pm.createNode('multiplyDivide', name=name)
    md.operation.set(operation)

    val = 0.0
    connect = False

    if type(input1) == pm.general.Attribute:
        val = input1.get()
        connect = True
    else:
        val = input1
        connect = False

    if type(val) == pm.datatypes.Vector or type(val) == tuple:
        if connect:
            input1.connect(md.input1)
        else:
            md.input1.set(input1)
    else:
        if connect:
            input1.connect(md.input1X)
        else:
            md.input1X.set(input1)

    if type(input2) == pm.general.Attribute:
        val = input2.get()
        connect = True
    else:
        val = input2
        connect = False

    if type(val) == pm.datatypes.Vector or type(val) == tuple:
        if connect:
            input2.connect(md.input2)
        else:
            md.input2.set(input2)
    else:
        if connect:
            input2.connect(md.input2X)
        else:
            md.input2X.set(input2)

    return md

def divide(input1, input2, name=None):
    return multiplyVector(input1, input2, name, operation=2)

def power(input1, input2, name=None):
    return multiplyVector(input1, input2, name, operation=3)

def multiplyRotationByScalar(input1, factor, name=None):
    node = pm.createNode('animBlendNodeAdditiveRotation')
    if name:
        node.rename(name)
    if type(input1) == pm.general.Attribute:
        input1.connect(node.inputA)
    else:
        node.inputA.set(input1)
    if type(factor) == pm.general.Attribute:
        factor.connect(node.weightA)
    else:
        node.weightA.set(factor)
    return node

def angleBetween(input1, input2, name=None):
    node = pm.createNode('angleBetween')
    if name:
        node.rename(name)
    if type(input1) == pm.general.Attribute:
        input1.connect(node.vector1)
    else:
        node.vector1.set(input1)
    if type(input2) == pm.general.Attribute:
        input2.connect(node.vector2)
    else:
        node.vector2.set(input2)
    return node

def distance(start, end, name):
    node = pm.createNode('distanceBetween')
    if name:
        node.rename(name)
    start.worldMatrix[0].connect(node.inMatrix1)
    end.worldMatrix[0].connect(node.inMatrix2)

    return node

def convert(input, factor, name=None):
    node = pm.createNode('unitConversion')
    if name:
        node.rename(name)
    if type(input) == pm.general.Attribute:
        input.connect(node.input)
    else:
        node.input.set(input)
    if type(factor) == pm.general.Attribute:
        factor.connect(node.conversionFactor)
    else:
        node.conversionFactor.set(factor)
    return node


# -----------------------------------------------------------------------
# MATRIX OPS
# -----------------------------------------------------------------------

axisDict = {'x': 0, 'y': 1, 'z': 2, '-x': 0, '-y': 1, '-z': 2}

def invertHandedness(mtx):
    '''
    Multiplies a mtx scaled to -1 in x by the supplied matrix
    Args:
        mtx: (pymel.datatypes.Matrix) The matrix whose handedness to invert
    Returns:
        (pymel.datatypes.Matrix) The same as input mtx but with inverted handedness
    '''
    return pm.datatypes.Matrix((-1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0)) * mtx

def createInverseHandedMatrix(mtx, composeMtx=None, name=None):
    '''
    Composes a matrix with scale -1, 1, 1 and multiplies it by the input matrix
    Args:
        mtx: (pm.general.Attribute) the matrix to invert
        composeMtx: (pm.PyNode(composeMatrix)) If supplied, will use it instead of creating a new composeMatrix node
    Returns:
        (pm.Pynode(multMatrix)) the newly created multMatrix node
    '''
    if not composeMtx:
        composeMtx = createComposeMatrix(inputScale=(-1, 1, 1), name='%s_negScale_mtx' % name)
    multMtx = multiplyMatrices([composeMtx.outputMatrix, mtx], name='%s_inverseHanded_mtx' % name)
    return multMtx

def getInverseHandedMatrix(mtx):
    '''
    returns a negatively scaled version of mtx on the x axis
    Args:
        mtx: (pm.datatypes.MAtrix) the matrix to negatively scale

    Returns:
        (pm.datatypes.Matrix) the inversely scaled matrix
    '''
    negMtx = pm.datatypes.Matrix((-1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0))
    return negMtx * mtx


def getMatrixAxisAsVector(mtx, axis):
    '''
    returns a vector representing the supplied axis of mtx
    '''
    if type(mtx) == pm.general.Attribute:
        mtx = mtx.get()
    row = None
    if 'x' in axis:
        row = dt.Vector((mtx.a00, mtx.a01, mtx.a02))
    elif 'y' in axis:
        row = dt.Vector((mtx.a10, mtx.a11, mtx.a12))
    else:
        row = dt.Vector((mtx.a20, mtx.a21, mtx.a22))
    return row

def multiplyMatrices(mtxList, name=None):
    '''
    Creates a multMatrix node and connects each mtx in mtxList in order
    Returns newly created multMatrix node
    '''
    mtx = pm.createNode('multMatrix')
    if name:
        mtx.rename(name)
    for i in range(len(mtxList)):
        mtxList[i].connect(mtx.matrixIn[i])
    return mtx

def decomposeMatrix(mtx, recycle=1, name=None):
    node = [node for node in pm.listConnections(mtx)
            if type(node) == pm.nodetypes.DecomposeMatrix
            and recycle]
    if node:
        node = node[0]
    if not node:
        node = pm.createNode('decomposeMatrix')
        if name:
            node.rename(name)
        mtx.connect(node.inputMatrix)
    return node

def getTransformedPoint(point, matrix):
    point = dt.VectorN(point[0], point[1], point[2], 1.0)
    if type(matrix) == type([]):
        matrix = dt.Matrix(matrix)
    point = point * matrix
    return dt.Vector(point[0], point[1], point[2])

def createTransformedPoint(point, matrix, name=None):
    node = pm.createNode('vectorProduct')
    if name:
        node.rename(name)
    if type(point) == pm.general.Attribute:
        point.connect(node.input1)
    else:
        node.input1.set(point)
    matrix.connect(node.matrix)
    node.operation.set(4)
    return node

def createMatrixAxisVector(matrix, axis, name=None):
    node = pm.createNode('vectorProduct')
    if name:
        node.rename(name)
    matrix.connect(node.matrix)
    if type(axis) == pm.general.Attribute:
        axis.connect(node.input1)
    else:
        node.input1.set(axis)
    node.operation.set(3)
    return node

def createComposeMatrix(inputTranslate=(0,0,0), inputRotate=(0,0,0), inputScale=(1,1,1), name=None):
    node = pm.createNode('composeMatrix')
    if name:
        node.rename(name)

    if type(inputTranslate) == pm.general.Attribute:
        inputTranslate.connect(node.inputTranslate)
    else:
        node.inputTranslate.set(inputTranslate)

    if type(inputRotate) == pm.general.Attribute:
        inputRotate.connect(node.inputRotate)
    else:
        node.inputRotate.set(inputRotate)

    if type(inputScale) == pm.general.Attribute:
        inputScale.connect(node.inputScale)
    else:
        node.inputScale.set(inputScale)
    return node

def composeMatrixFromMatrix(mtx, name=None):
    '''
    decomposes mtx by creating a temp decomposeMatrixNode then deleting it
    Args:
        mtx: (pm.datatypes.Matrix) the input matrix
        name: (string) the name for the new composeMatrix node

    Returns:
        (pm.PyNode(composeMatrix)) the newly created composeMatrix node
    '''
    d = decomposeMatrix(mtx, recycle=0)
    t = tuple(d.outputTranslate.get())
    r = tuple(d.outputRotate.get())
    s = tuple(d.outputScale.get())
    pm.delete(d)
    return createComposeMatrix(t, r, s, name=name)

def inverseMatrix(matrix, name=None):
    node = pm.createNode('inverseMatrix')
    if name:
        node.rename(name)
    matrix.connect(node.inputMatrix)
    return node

def vectors2Mtx44(vec1, vec2, vec3, vec4=[0, 0, 0], name=''):
    node = pm.createNode('fourByFourMatrix')
    if name:
        node.rename(name)

    if type(vec1) == pm.general.Attribute:
        vec1.children()[0].connect(node.in00)
        vec1.children()[1].connect(node.in01)
        vec1.children()[2].connect(node.in02)
    else:
        node.in00.set(vec1[0])
        node.in01.set(vec1[1])
        node.in02.set(vec1[2])

    if type(vec2) == pm.general.Attribute:
        vec2.children()[0].connect(node.in10)
        vec2.children()[1].connect(node.in11)
        vec2.children()[2].connect(node.in12)
    else:
        node.in10.set(vec2[0])
        node.in11.set(vec2[1])
        node.in12.set(vec2[2])

    if type(vec3) == pm.general.Attribute:
        vec3.children()[0].connect(node.in20)
        vec3.children()[1].connect(node.in21)
        vec3.children()[2].connect(node.in22)
    else:
        node.in20.set(vec3[0])
        node.in21.set(vec3[1])
        node.in22.set(vec3[2])

    if type(vec3) == pm.general.Attribute:
        vec4.children()[0].connect(node.in30)
        vec4.children()[1].connect(node.in31)
        vec4.children()[2].connect(node.in32)
    else:
        node.in30.set(vec4[0])
        node.in31.set(vec4[1])
        node.in32.set(vec4[2])

    node.in33.set(1)

    return node

def connectSrt(src, dest, s=1, r=1, t=1):
    if s:
        src.outputScale.connect(dest.s)
    if r:
        src.outputRotate.connect(dest.r)
    if t:
        src.outputTranslate.connect(dest.t)


# -----------------------------------------------------------------------
# VECTOR OPS
# -----------------------------------------------------------------------

def getStartAndEnd(start=None, end=None):
    '''
    Takes either two pynodes, two vectors, two matrix attrs or two selected nodes and returns their positions
    '''
    startPos, endPos = None, None
    if not start or not end:
        if len(pm.selected()) == 2:
            startPos = pm.xform(pm.selected()[0], translation=True, query=True, ws=True)
            endPos = pm.xform(pm.selected()[1], translation=True, query=True, ws=True)
    else:
        if type(start) == pm.general.Attribute:
            print 'START - ATTRIBUTE'
            if type(start.get()) == pm.datatypes.Matrix:
                startPos = start.get().translate.get()
            else:
                startPos = start.get()
        elif pm.nodetypes.Transform in type(start).__mro__:
            startPos = pm.xform(start, translation=True, query=True, ws=True)
        else:
            startPos = start

        if type(end) == pm.general.Attribute:
            if type(end.get()) == pm.datatypes.Matrix:
                endPos = end.get().translate.get()
            else:
                endPos = end.get()
        elif pm.nodetypes.Transform in type(end).__mro__:
            endPos = pm.xform(end, translation=True, query=True, ws=True)
        else:
            endPos = end

    if not startPos or not endPos:
        return (None, None)
    else:
        return startPos, endPos

def getDistance(start, end):
    '''
    Calculates distance between two Transforms using magnitude
    '''

    def mag(numbers):
        num = 0
        for eachNumber in numbers:
            num += math.pow(eachNumber, 2)

        mag = math.sqrt(num)
        return mag

    startPos, endPos = getStartAndEnd(start, end)

    if not startPos or not endPos:
        return 'getDistance: Cannot determine start and end points'

    calc = []
    calc.append(startPos[0] - endPos[0])
    calc.append(startPos[1] - endPos[1])
    calc.append(startPos[2] - endPos[2])

    return mag(calc)

def normalize(vector, name=''):
    node = pm.createNode('vectorProduct')
    if name:
        node.rename(name)
    vector.connect(node.input1)
    node.normalizeOutput.set(1)
    node.operation.set(0)

    return node

def createCrossProduct(vec1, vec2, normalize=1, name=''):
    node = pm.createNode('vectorProduct')
    if name:
        node.rename(name)
    vec1.connect(node.input1)
    vec2.connect(node.input2)
    node.operation.set(2)
    node.normalizeOutput.set(normalize)

    return node

def createDotProduct(vec1, vec2, normalize=1, name=''):
    node = pm.createNode('vectorProduct')
    if name:
        node.rename(name)
    vec1.connect(node.input1)
    vec2.connect(node.input2)
    node.operation.set(1)
    node.normalizeOutput.set(normalize)

    return node

def addTerms(vec1, vec2):
    return tuple([vec1[i] + vec2[i] for i in range(len(vec1))])

def subtractTerms(vec1, vec2):
    return tuple([vec1[i] - vec2[i] for i in range(len(vec1))])

def multiplyTerms(vec1, vec2):
    return tuple([vec1[i] * vec2[i] for i in range(len(vec1))])

def getPointsAlongVector(start, end, divs):
    '''
    Returns a list of points between start and end of length divs
    Args:
        start: (PyNode) or (float3) start node or position
        end: (PyNode) or (float3) end node or position
        divs: (int) number of points to return

    Returns:
        ([float3]) list of points
    '''
    start, end = getStartAndEnd(start, end)
    startVec, endVec = pm.datatypes.Vector(start), pm.datatypes.Vector(end)
    masterVec = endVec-startVec
    segVec = masterVec / (divs-1)
    points = []
    for i in range(divs):
        points.append((segVec*i)+startVec)
    return points


# -----------------------------------------------------------------------
# MISC
# -----------------------------------------------------------------------
def clamp(input, min, max, name=None):
    node = pm.createNode('clamp')
    if name:
        node.rename(name)
    input.connect(node.inputR)
    if type(min) == pm.general.Attribute:
        min.connect(node.minR)
    else:
        node.minR.set(min)
    if type(max) == pm.general.Attribute:
        max.connect(node.maxR)
    else:
        node.maxR.set(max)
    return node

def blendScalarAttrs(attr1, attr2, blend, name=None):
    node = pm.createNode('blendTwoAttr')
    if name:
        node.rename(name)
    if type(attr1) == pm.general.Attribute:
        attr1.connect(node.input[0])
    else:
        node.input[0].set(attr1)
    if type(attr2) == pm.general.Attribute:
        attr2.connect(node.input[1])
    else:
        node.input[1].set(attr2)
    if type(blend) == pm.general.Attribute:
        blend.connect(node.attributesBlender)
    else:
        node.attributesBlender.set(blend)
    return node

def remap(input, oldMin, oldMax, min, max, name=None):
    node = pm.createNode('setRange')
    if name:
        node.rename(name)
    if type(input) == pm.general.Attribute:
        input.connect(node.valueX)
    else:
        node.valueX.set(input)
    if type(oldMax) == pm.general.Attribute:
        oldMax.connect(node.oldMaxX)
    else:
        node.oldMaxX.set(oldMax)
    if type(oldMin) == pm.general.Attribute:
        oldMin.connect(node.oldMinX)
    else:
        node.oldMinX.set(oldMin)
    if type(max) == pm.general.Attribute:
        max.connect(node.maxX)
    else:
        node.maxX.set(max)
    if type(min) == pm.general.Attribute:
        min.connect(node.minX)
    else:
        node.minX.set(min)
    return node

def isolateRotationOnAxis(rotationAttr, axis, name=''):
    quatNode = pm.createNode('eulerToQuat')
    if name:
        quatNode.rename('%s_quat2Euler' % name)
    eulerNode = pm.createNode('quatToEuler')
    if name:
        eulerNode.rename('%s_euler2Quat' % name)
    rotationAttr.connect(quatNode.inputRotate)
    sourceAttr = pm.Attribute('%s.outputQuat%s' % (quatNode.name(), axis.upper()))
    destAttr = pm.Attribute('%s.inputQuat%s' % (eulerNode.name(), axis.upper()))
    sourceAttr.connect(destAttr)
    quatNode.outputQuatW.connect(eulerNode.inputQuatW)

    if 'Y' in axis.upper():
        eulerNode.inputRotateOrder.set(4)
    elif 'Z' in axis.upper():
        eulerNode.inputRotateOrder.set(2)


    return[quatNode, eulerNode]

