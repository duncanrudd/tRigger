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