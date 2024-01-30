import pymel.core as pm

def align( node=None, target=None, translate=True, orient=True, scale=False, parent=0 ):
    '''
    sets the translation and / or orientation of node to match target
    If optional parent flag is set to true. Will also parent to the target node
    '''

    # Validate that the correct arguments have been supplied
    if not node or not target:
        # If node and target aren't explicitly supplied, check for a valid selection to use
        sel = pm.selected()
        if len(sel) == 2:
            node, target = sel[0], sel[1]
        else:
            return 'Align: Cannot determine nodes to align'

    targetMatrix = pm.xform( target, q=True, ws=1, matrix=True )
    nodeMatrix = pm.xform( node, q=True, ws=1, matrix=True )

    nodeScale = node.s.get()

    if translate and orient:
        pm.xform(node, ws=1, matrix=targetMatrix)
    elif translate:
        # set row4 x y z to row4 of targetMatrix
        nodeMatrix[12:-1] = targetMatrix[ 12:-1]
        pm.xform(node, ws=1, matrix=nodeMatrix)
    elif orient:
        # set rows 1-3 to rows 1-3 of nodeMatrix
        targetMatrix[12:-1] = nodeMatrix[12:-1]
        pm.xform(node, ws=1, matrix=targetMatrix)

    if not scale:
        node.s.set(nodeScale)

    if parent:
        if not node.getParent() == target:
            node.setParent(target)

def getMatrixFromPos(pos):
    '''
    :param pos: position vector
    :return: pm.datatypes.Matrix object with its translation set to pos
    '''
    mtx = pm.datatypes.Matrix()
    mtx.translate = pos
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

def connectSrt(src, dest, s=1, r=1, t=1):
    if s:
        src.outputScale.connect(dest.s)
    if r:
        src.outputRotate.connect(dest.r)
    if t:
        src.outputTranslate.connect(dest.t)

def pmMtx2fourFourMtx(mtx, name=None):
    node = pm.createNode('fourByFourMatrix')
    if name:
        node.rename(name)
    mtxList = mtx.tolist()

    node.in00.set(mtxList[0][0])
    node.in01.set(mtxList[0][1])
    node.in02.set(mtxList[0][2])

    node.in10.set(mtxList[1][0])
    node.in11.set(mtxList[1][1])
    node.in12.set(mtxList[1][2])

    node.in20.set(mtxList[2][0])
    node.in21.set(mtxList[2][1])
    node.in22.set(mtxList[2][2])

    node.in30.set(mtxList[3][0])
    node.in31.set(mtxList[3][1])
    node.in32.set(mtxList[3][2])

    return node

def multiplyMatrices(mtxList, name=None):
    node = pm.createNode('multMatrix')
    if name:
        node.rename(name)
    for i, mtx in enumerate(mtxList):
        mtx.connect(node.matrixIn[i])
    return node

def createAimMatrix(input, target, name=None):
    '''
    Creates an aimMatrix node. Connects input to input matrix and output to primary target matrix
    Args:
        input: (pyNode or pymel.nodetypes.Attribute) The input node or an input matrix attribute
        target: (pyNode or pymel.nodetypes.Attribute) The target node or an input matrix attribute
        name: (string) name of the new aimMatrix node
    Returns:
        (pyNode) The new aimMatrix node

    '''
    node = pm.createNode('aimMatrix')
    if name:
        node.rename(name)
    # Coerce input and output to be matrix attributes
    if pm.nodetypes.Transform in type(input).__mro__:
        input = input.worldMatrix[0]
    if pm.nodetypes.Transform in type(target).__mro__:
        target = target.worldMatrix[0]
    input.connect(node.inputMatrix)
    target.connect(node.primaryTargetMatrix)
    return node

def createNonRollMatrix(input, target, axis='x', name=None):
    '''
    Creates a non rolling aim matrix for use in isolating twist.
    Args:
        input: (pyNode or pymel.nodetypes.Attribute) The input node or an input matrix attribute
        target: (pyNode or pymel.nodetypes.Attribute) The target node or an input matrix attribute
        axis: (string) the axis to aim along
        name: ([string]) names of the new aimMatrix nodes

    Returns:
        ([pynode]) the two new aimMatrix nodes
    '''
    lockVec = (0, 1, 0)
    lockVec2 = (0, 0, 1)
    aimVec = (1, 0, 0)
    if 'y' in axis:
        lockVec = (1, 0, 0)
        aimVec = (0, 1, 0)
    elif 'z' in axis:
        lockVec2 = (1, 0, 0)
        aimVec = (0, 0, 1)
    if '-' in axis:
        aimVec = tuple([aimVec[i] * -1 for i in range(len(aimVec))])

    name1, name2 = None, None
    if name:
        name1, name2 = name[0], name[1]
    node1 = createAimMatrix(input, target, name1)
    node1.primaryMode.set(0)
    node1.primaryInputAxis.set(lockVec)
    node1.secondaryMode.set(1)
    node1.secondaryInputAxis.set(aimVec)
    conn = pm.listConnections(node1.primaryTargetMatrix, p=1, d=0)[0]
    conn.connect(node1.secondaryTargetMatrix)
    conn.disconnect(node1.primaryTargetMatrix)

    node2 = createAimMatrix(node1.outputMatrix, target, name2)
    node2.primaryInputAxis.set(aimVec)

    return [node1, node2]

def bakeSrtToOffsetParentMtx(node):
    '''
    Zeros out the channel box srt attributes and bakes the values into the offsetParentMatrix attribute
    Args:
        node: (pyNode) te node to bake
    '''
    parent = node.getParent()
    if parent:
        node.offsetParentMatrix.set(node.worldMatrix[0].get() * parent.worldInverseMatrix.get())
    else:
        node.offsetParentMatrix.set(node.worldMatrix[0].get())
    node.t.set((0, 0, 0))
    node.r.set((0, 0, 0))
    node.s.set((1, 1, 1))

def bakeOffsetParentMtxToSrt(node):
    '''
    Zeros out the offsetParentMatrix and bakes the values into the channel box srt attributes
    Args:
        node: (pyNode) the node to bake
    '''
    mtx = node.worldMatrix[0].get()
    node.offsetParentMatrix.set(pm.datatypes.Matrix())
    pm.xform(node, ws=1, m=mtx)

def blend_T_R_matrices(t_mtx, r_mtx, name=None):
    '''
    Creates a blend Matrix node with T and R matrices as inputs. Result is the translate and scale from T_mtx and the
    rotate and shear from r_mtx
    Args:
        t_mtx: (pm.datattypes.Matrix) the input translate matrix
        r_mtx: (pm.datattypes.Matrix) the input rotate matrix
        name: (string) name of the new node

    Returns:
        (pyNode) the newly create blend matrix node
    '''
    node = pm.createNode('blendMatrix')
    if name:
        node.rename(name)
    if type(t_mtx) == pm.general.Attribute:
        t_mtx.connect(node.inputMatrix)
    else:
        node.inputMatrix.set(t_mtx)
    if type(r_mtx) == pm.general.Attribute:
        r_mtx.connect(node.target[0].targetMatrix)
    else:
        node.target[0].targetMatrix.set(r_mtx)
    node.target[0].weight.set(1)
    node.target[0].useTranslate.set(0)
    node.target[0].useScale.set(0)

    return node

def blendMatrices(mtx1, mtx2, name=None, weight=0.5):
    '''
    Creates a blend Matrix node with mtx1 and mtx2 matrices as inputs.
    Blends between them based on weight value
    Args:
        mtx1: (pm.datattypes.Matrix) the 1st input matrix
        mtx2: (pm.datattypes.Matrix) the 2nd input matrix
        name: (string) name of the new node

    Returns:
        (pyNode) the newly create blend matrix node
    '''
    node = pm.createNode('blendMatrix')
    if name:
        node.rename(name)
    if type(mtx1) == pm.general.Attribute:
        mtx1.connect(node.inputMatrix)
    else:
        node.inputMatrix.set(mtx1)
    if type(mtx2) == pm.general.Attribute:
        mtx2.connect(node.target[0].targetMatrix)
    else:
        node.target[0].targetMatrix.set(mtx2)
    if type(weight) == pm.general.Attribute:
        weight.connect(node.target[0].weight)
    else:
        node.target[0].weight.set(weight)

    return node

def getBlendedMatrix(targetList):
    '''
    Returns average of transforms in targetList
    Args:
        targetList: ([pyNode]) or ([pymel.datatype.Matrix]) List of transforms to align to
    '''
    blend = pm.createNode('blendMatrix')
    for index, targ in enumerate(targetList):
        if pm.nodetypes.Transform in type(targ).__mro__:
            targ = targ.worldMatrix[0].get()
        elif type(targ) == pm.general.Attribute:
            targ = targ.get()
        if index == 0:
            blend.inputMatrix.set(targ)
        else:
            blend.target[index].targetMatrix.set(targ)
            blend.target[index].weight.set(1.0 / (index + 1))
    mtx = blend.outputMatrix.get()
    pm.delete(blend)
    return mtx

def getOppositeMatrix(mtx):
    '''
    Calculates a matrix mirrored on the x axis such that handedness remains unchanged
    Args:
        mtx: (pm.datatypes.Matrix) the matrix to mirror

    Returns:
        (pm.datatypes.Matrix) the mirrored matrix
    '''
    # invert x position
    mtx.a30 *= -1

    # invert z and y components of x axis
    mtx.a01 *= -1
    mtx.a02 *= -1

    # invert x component of y and z axes
    mtx.a10 *= -1
    mtx.a20 *= -1

    return mtx

def getMirrorMatrix(mtx):
    '''
    Calculates a matrix mirrored on the x axis such that the orientation is mirrored
    Args:
        mtx: (pm.datatypes.Matrix) the matrix to mirror

    Returns:
        (pm.datatypes.Matrix) the mirrored matrix
    '''
    # invert x position
    mtx.a30 *= -1

    # invert x and y components of z axis
    mtx.a20 *= -1
    mtx.a21 *= -1

    # invert y and z components of y axis
    mtx.a11 *= -1
    mtx.a12 *= -1

    # invert x component of x axis
    mtx.a00 *= -1

    return mtx

def list2Mtx(list):
    '''
    Takes a list (e.g.the return value of pm.xform() command and converts it into a pm.datatypes.Matrix
    Args:
        list: ([float]) the list which should be converted

    Returns:
        (pm.datatypes.Matrix) the matrix resulting from the conversion
    '''
    return pm.datatypes.Matrix((list[0], list[1], list[2]), (list[4], list[5], list[6]),
                               (list[8], list[9], list[10]), (list[12], list[13], list[14]))

def mtx2List(mtx):
    '''
    Takes a pymel.datatypes.Matrix and turns it into a flat list which is serializable to json
    Args:
        mtx: (pm.datatypes.Matrix) the matrix to convert to a list

    Returns:
        (list) the list representing the matrix
    '''
    return [mtx[0][0], mtx[0][1], mtx[0][2], mtx[0][3],
            mtx[1][0], mtx[1][1], mtx[1][2], mtx[1][3],
            mtx[2][0], mtx[2][1], mtx[2][2], mtx[2][3],
            mtx[3][0], mtx[3][1], mtx[3][2], mtx[3][3]]

def invertRotateOrder(node, name=None):
    '''
    Creates a choice node with inputs corresponding to the reverse of the standard rotate order dropdown.
    Node.rotateOrder is connected to the choice node's selector attribute. Use to negate a parent transforms rotation.
    Args:
        node: (pm.PyNode) the node whose rotateOrder we want to reverse
        name: (str) the name to give the newly created choice node

    Returns:
        (pm.PyNode) The newly created choice node
    '''
    choice = pm.createNode('choice')
    if name:
        choice.rename(name)
    choice.input[0].set(5)
    choice.input[1].set(3)
    choice.input[2].set(4)
    choice.input[3].set(1)
    choice.input[4].set(2)
    choice.input[5].set(0)
    node.rotateOrder.connect(choice.selector)
    return choice










