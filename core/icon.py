import pymel.core as pm

def getShape(node):
    '''
    returns the first shape under the given transform
    '''
    shapes = getShapes(node)
    if shapes:
        return shapes[0]
    else:
        return None

def getShapes(node):
    '''
    returns all shapes under the given transform
    '''
    shapes = pm.listRelatives(node, c=1, s=1)
    return shapes

def boxIcon(size=20.0, name=''):
    '''
    Creates a box shaped nurbs curve

    '''
    pos = size * 0.5
    neg = pos * -1
    points = [(neg, pos, neg), (neg, neg, neg), (neg, neg, pos), (neg, pos, pos),
              (neg, pos, neg), (pos, pos, neg), (pos, neg, neg), (neg, neg, neg),
              (neg, neg, pos), (pos, neg, pos), (pos, neg, neg), (pos, pos, neg),
              (pos, pos, pos), (neg, pos, pos), (pos, pos, pos), (pos, neg, pos)]

    knots = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]

    icon = pm.curve(degree=1, p=points, k=knots, name=name)

    return icon


def crossIcon(size=20.0, name=''):
    '''
    Creates a locator shaped nurbs curve

    '''
    pos = size * 0.5
    neg = pos * -1
    points = [(0, 0, neg), (0, 0, pos), (0, 0, 0), (0, pos, 0),
              (0, neg, 0), (0, 0, 0), (pos, 0, 0), (neg, 0, 0)]

    knots = [1, 2, 3, 4, 5, 6, 7, 8]

    icon = pm.curve(degree=1, p=points, k=knots, name=name)

    return icon

def crossBoxIcon(size=20.0, name=''):
    '''
    Creates a locator shaped nurbs curve and a box shaped nurbs curve

    '''
    icon = crossIcon(size, name)
    box = boxIcon(size*.5, name)

    boxShape = getShape(box)
    pm.parent(boxShape, icon, s=1, r=1)
    pm.delete(box)

    return icon