from tRigger.core import mathOps, attribute

import pymel.core as pm

def defineCollision(obj1, obj2, params, name, axis='y'):
    '''
    Compares the transforms of obj1 and obj2. If obj1 goes beyond obj2 on specified axis a blended transform is computed
    for output1 and output2. Otherwise output1=obj1.transform and output2=obj2.transform.
    :param obj1: first comparison object
    :param obj2: second comparison object
    :param params: node on which to expose outputs and config attributes
    :param name: prefix for newly created nodes
    :param axis: primary axis for comparision
    :return:
    '''
    if not params.hasAttr('push_amount'):
        pm.addAttr(params, ln='push_amount', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
    if not params.hasAttr('push_midpoint'):
        attribute.addFloatAttr(params, 'push_midpoint', minValue=0.0, maxValue=1.0)
    if not params.hasAttr('secondary_push_mult'):
        attribute.addFloatAttr(params, 'secondary_push_mult', minValue=-1, maxValue=1.0)

    startPushAttr = attribute.addFloatAttr(params, '%s_start_push' % name, k=0)
    endPushAttr = attribute.addFloatAttr(params, '%s_end_push' % name, k=0)
    secondaryPushAttr = attribute.addFloatAttr(params, '%s_secondary_push' % name, k=0)

    mtxMult = mathOps.multiplyMatrices([obj1.worldMatrix[0], obj2.worldInverseMatrix[0]], name='%s_localMtx' % name)
    d = mathOps.decomposeMatrix(mtxMult.matrixSum, name='%s_localMtxToSrt' % name)
    cond = pm.createNode('condition', name='%s_isColliding' % name)
    attr = pm.Attribute('%s.outputTranslate%s' % (d.name(), axis.upper()))
    attr.connect(cond.firstTerm)
    cond.operation.set(2)

    midpointBlend = mathOps.multiply(params.push_midpoint, cond.outColorR, name='%s_midpointBlend' % name)
    startPushBlend = mathOps.multiply(midpointBlend.output, attr, name='%s_startPushBlend' % name)
    startPushAmount = mathOps.multiply(startPushBlend.output, params.push_amount, '%s_startPushAmount' % name)
    startPushAmount.output.connect(startPushAttr)
    endPushPma = mathOps.subtractScalar([startPushBlend.output, attr], name='%s_endPush' % name)
    endPushBlend = mathOps.multiply(endPushPma.output1D, cond.outColorR, name='%s_endPushBlend' % name)
    endPushAmount = mathOps.multiply(endPushBlend.output, params.push_amount, '%s_endPushAmount' % name)
    endPushAmount.output.connect(endPushAttr)

    secondaryPush = mathOps.multiply(attr, cond.outColorR, name='%s_secondaryPush' % name)
    secondaryPushBlend = mathOps.multiply(secondaryPush.output, params.secondary_push_mult,
                                          name='%s_secondaryPushBlend' % name)
    secondaryPushBlend.output.connect(secondaryPushAttr)

    return {'startPushAttr': startPushAttr, 'endPushAttr': endPushAttr, 'secondaryPushAttr': secondaryPushAttr}
