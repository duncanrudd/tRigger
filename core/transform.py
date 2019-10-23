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


