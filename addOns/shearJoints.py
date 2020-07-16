import pymel.core as pm
import math
from tRigger.core import mathOps, dag
'''
Add to any joint in a hierarchy
Derives dominant axis and applies shear on other two axes base on angle between the joint and its parent
Creates two new joints and their associated shearing transforms and parents them under the target joint
'''
def addShearJoints(targetJoint):
    name=targetJoint.name().replace('_jnt', '_shear')

    # determine dominant axis
    pos = targetJoint.t.get() + targetJoint.offsetParentMatrix.get().translate.get()
    axis=(1, 0, 0)
    if math.fabs(pos[1]) > math.fabs(pos[0]) and math.fabs(pos[1]) > math.fabs(pos[2]):
        axis=(0, 1, 0)
    elif math.fabs(pos[2]) > math.fabs(pos[0]) and math.fabs(pos[2]) > math.fabs(pos[1]):
        axis=(0, 0, 1)

    child = pm.listRelatives(targetJoint, type='joint', c=1)[0]
    parent = targetJoint.getParent()
    targetInverseMtx = mathOps.inverseMatrix(targetJoint.offsetParentMatrix, name=name+'_targ_inverse_mtx')
    parentAimMtx = pm.createNode('aimMatrix', name=name + '_parent_aim_mtx')
    targetInverseMtx.outputMatrix.connect(parentAimMtx.primaryTargetMatrix)
    parentAimMtx.primaryInputAxisX.set(-1)

    childAimMtx = pm.createNode('aimMatrix', name=name + '_child_aim_mtx')
    child.offsetParentMatrix.connect(childAimMtx.primaryTargetMatrix)

    parentAxis = mathOps.createMatrixAxisVector(parentAimMtx.outputMatrix, (1, 0, 0), name=name+'_parent_axis')
    childAxis = mathOps.createMatrixAxisVector(childAimMtx.outputMatrix, (1, 0, 0), name=name+'_child_axis')
    angle = mathOps.angleBetween(parentAxis.output, childAxis.output)

    parentSrt = dag.addChild(targetJoint, 'group', name=name+'_parent_srt')
    parentSrt.tx.set(mathOps.getDistance(targetJoint, parent)*-.1)
    childSrt = dag.addChild(targetJoint, 'group', name=name+'_child_srt')
    childSrt.tx.set(mathOps.getDistance(targetJoint, child)*.1)

    parentJoint = dag.addChild(parentSrt, 'joint', name=name + '_parent_jnt')
    childJoint = dag.addChild(childSrt, 'joint', name=name + '_child_jnt')

    parentAimMtx.outputMatrix.connect(parentSrt.offsetParentMatrix)
    childAimMtx.outputMatrix.connect(childSrt.offsetParentMatrix)

    shearY = mathOps.convert(angle.euler.eulerY, 1, name=name+'_shearY')
    shearY.output.connect(parentSrt.shearXZ)
    shearZ = mathOps.convert(angle.euler.eulerZ, -1, name=name + '_shearZ')
    shearZ.output.connect(parentSrt.shearXY)

    childShearY = mathOps.convert(angle.euler.eulerY, -1, name=name + '_child_shearY')
    childShearY.output.connect(childSrt.shearXZ)
    childShearZ = mathOps.convert(angle.euler.eulerZ, 1, name=name + '_child_shearZ')
    childShearZ.output.connect(childSrt.shearXY)


