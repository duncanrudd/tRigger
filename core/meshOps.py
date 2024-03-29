import pymel.core as pm
import maya.api.OpenMaya as om
import math
import tRigger.core.mathOps as mathOps
import tRigger.core.attribute as attribute
reload(mathOps)

def testIfInsideMesh(targMesh, point=(0.0, 0.0, 0.0), dir=(0.0, 0.0, 1.0)):
    sel = om.MSelectionList()

    #replace torus with arbitrary shape name
    sel.add(targMesh.name())
    dag = sel.getDagPath(0)

    mesh = om.MFnMesh(dag)

    point = om.MFloatPoint(*point)
    dir = om.MFloatVector(*dir)

    farray = mesh.allIntersections(point, dir, om.MSpace.kWorld, 10000, 0)[0]
    return len(farray)%2 == 1


def meshAlongCurve(crv, upVec=(1, 0, 0), sides=8, radius=3.0, taper = 1.0):
    print 'creating mesh'
    angleStep = math.pi / (sides*0.5)
    seedPoints = [pm.datatypes.Vector(0, math.sin(-angleStep*i), math.cos(-angleStep*i)) for i in range(sides)]
    vertices = []

    # Build points along curve
    sel = om.MSelectionList()
    sel.add(crv.name())
    dag = sel.getDagPath(0)
    crvFn = om.MFnNurbsCurve(dag)
    layers = crvFn.numCVs

    mtx = pm.datatypes.Matrix()
    angle = mathOps.angleBetween((1, 0, 0), (1, 0, 0), name='angle_temp')
    compose = mathOps.createComposeMatrix(name='mtx_temp')
    for layer in range(layers):
        # get tangent vector on curve at param
        param = (crv.maxValue.get() / (layers-1))*layer
        layerRadius = pow(taper, layer)
        tangent = crvFn.tangent(param, om.MSpace.kWorld)
        p = crvFn.getPointAtParam(param, om.MSpace.kWorld)
        if layer > 0:
            t = mtx.translate.get()
            worldTangent = pm.datatypes.Vector(tangent[0] + t[0], tangent[1] + t[1], tangent[2] + t[2])
            localTangent = mathOps.getTransformedPoint(worldTangent, mtx.inverse())
            angle.vector2.set(localTangent)
            compose.inputRotate.set(angle.euler.get())
            compose.inputTranslate.set(mathOps.getTransformedPoint(p, mtx.inverse()))
            compose.inputScale.set(layerRadius, layerRadius, layerRadius)
            mtx = compose.outputMatrix.get() * mtx
            localTangent.normalize()
            dispY = (1 - localTangent[0])*localTangent[1]
            dispZ = (1 - localTangent[0])*localTangent[2]
        else:
            # take cross products
            sideVec = tangent ^ om.MVector(upVec)
            sideVec.normalize()
            upVec = sideVec ^ tangent

            upVec.normalize()

            # Build matrix to transform seed points into
            mtx = pm.datatypes.Matrix(tangent[0]*radius, tangent[1]*radius, tangent[2]*radius, 0,
                                      upVec[0]*radius, upVec[1]*radius, upVec[2]*radius, 0,
                                      sideVec[0]*radius, sideVec[1]*radius, sideVec[2]*radius, 0,
                                      p[0], p[1], p[2], 1)
            dispY = 0
            dispZ = 0

        # loc = pm.spaceLocator()
        # loc.offsetParentMatrix.set(mtx)
        for point in seedPoints:
            dispVal = ((point[1]*dispY) + (point[2]*dispZ))
            p = pm.datatypes.Vector(dispVal, point[1], point[2])
            vertices.append(om.MPoint(mathOps.getTransformedPoint(p, mtx)))


    # Create mesh
    meshFn = om.MFnMesh()
    for layer in range(layers-1):
        for i in range(sides):
            faceArray = om.MPointArray()

            # Specify 4 verts per face
            faceArray.append(vertices[i+(layer*sides)])
            faceArray.append(vertices[(i+1)%sides+(layer*sides)])
            faceArray.append(vertices[(i+1)%sides+((layer+1)*sides)])
            faceArray.append(vertices[i+((layer+1)*sides)])

            meshFn.addPolygon(faceArray, 1, .001)
    print 'mesh created'

def returnClosestUV(src, targ):
    shape = pm.listRelatives(src, c=1, s=1)[0]
    if type(targ) == pm.nodetypes.Transform:
        pos = targ.worldMatrix[0].get().translate.get()
    else:
        pos = targ

    if pm.nodeType(shape) == 'mesh':
        reader = pm.createNode('closestPointOnMesh')
        reader.inPosition.set(pos)
        shape.worldMesh[0].connect(reader.inMesh)
    elif pm.nodeType(shape) == 'nurbsSurface':
        reader = pm.createNode('closestPointOnSurface')
        reader.inPosition.set(pos)
        shape.worldSpace[0].connect(reader.inputSurface)

    uv = [reader.parameterU.get(), reader.parameterV.get()]

    pm.delete(reader)
    return (uv)


def pinToMesh(mesh, srcPos):
    '''
    Check if a uvPin node exists on mesh. creates one if not
    Args:
        mesh: geometry to pin to
        srcPos: node or position from which to sample UVs

    Returns:
        matrix output attribute from uvPin node
    '''
    shapes = pm.listRelatives(mesh, c=1, s=1)
    try:
        pin = pm.listConnections(shapes[0], d=1, s=0, type='uvPin')[0]
    except:
        pin = pm.createNode('uvPin')
        shapes[0].worldMesh[0].connect(pin.deformedGeometry)
        if len(shapes) > 0:
            shapes[1].worldMesh[0].connect(pin.originalGeometry)

    uv = returnClosestUV(mesh, srcPos)
    slot = pin.coordinate.evaluateNumElements()
    pin.coordinate[slot].coordinateU.set(uv[0])
    pin.coordinate[slot].coordinateV.set(uv[1])

    return pin.outputMatrix[slot]

