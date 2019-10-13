import pymel.core as pm

def setColour(node, colour=None):
    '''
    takes a node and enables the drawing overrides.
    'Colour' specifies either an integer for the required color or a string corresponding to a key in colorDict
    if nodelist is not supplied, will attempt to work on selected nodes.

    '''
    if not colour:
        raise RuntimeError, 'color not specified. You must supply either a string or integer.'
    colourDict = {
                   'center': 14,    # green
                   'right': 13,     # red
                   'left': 6,       # blue
                   'red': 13,
                   'blue': 6,
                   'yellow': 17,
                   'green': 14,
                   'purple': 9,
                   'C': 14,         # green
                   'R': 13,         # red
                   'L': 6,          # blue
                   'cyan': 18,
                   'magenta': 9,
                  }


    if type(colour) == type('hello') or type(colour) == type(u'hello'):
        colour = colourDict[colour]

    node.overrideEnabled.set(1)
    node.overrideColor.set(colour)

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

def boxIcon(size=20.0, name='', colour=None):
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

    if colour:
        setColour(icon, colour)

    return icon

def ballIcon(radius=20.0, name='', colour=None):
    icon = pm.circle(name=name, r=radius, ch=0, o=1, s=8, nr=(0, 1, 0))[0]
    dup1 = pm.circle(name=name, r=radius, ch=0, o=1, s=8, nr=(1, 0, 0))[0]
    dup2 = pm.circle(name=name, r=radius, ch=0, o=1, s=8, nr=(0, 0, 1))[0]
    shape = pm.listRelatives(dup1, shapes=True)[0]
    pm.parent(shape, icon, s=1, r=1)
    shape = pm.listRelatives(dup2, shapes=True)[0]
    pm.parent(shape, icon, s=1, r=1)
    pm.delete([dup1, dup2])

    if colour:
        setColour(icon, colour)

    return icon


def crossIcon(size=20.0, name='', colour=None):
    '''
    Creates a locator shaped nurbs curve

    '''
    pos = size * 0.5
    neg = pos * -1
    points = [(0, 0, neg), (0, 0, pos), (0, 0, 0), (0, pos, 0),
              (0, neg, 0), (0, 0, 0), (pos, 0, 0), (neg, 0, 0)]

    knots = [1, 2, 3, 4, 5, 6, 7, 8]

    icon = pm.curve(degree=1, p=points, k=knots, name=name)

    if colour:
        setColour(icon, colour)

    return icon

def crossBoxIcon(size=20.0, name='', colour=None):
    '''
    Creates a locator shaped nurbs curve and a box shaped nurbs curve

    '''
    icon = crossIcon(size, name)
    box = boxIcon(size*.5, name)

    boxShape = getShape(box)
    pm.parent(boxShape, icon, s=1, r=1)
    pm.delete(box)

    if colour:
        setColour(icon, colour)

    return icon

def crossBallIcon(size=20.0, name='', colour=None):
    '''
    Creates a locator shaped nurbs curve and a ball shaped nurbs curve

    '''
    icon = crossIcon(size, name)
    ball = ballIcon(size*.25, name)

    ballShapes = getShapes(ball)
    for shape in ballShapes:
        pm.parent(shape, icon, s=1, r=1)
    pm.delete(ball)

    if colour:
        setColour(icon, colour)

    return icon

def squarePointIcon(size=20.0, name='', colour=None):
    '''
    creates a square shaped nurbs curve with a point at the front
    '''
    pos = size * 0.5
    neg = pos * -1
    points = [(0, neg*.05, pos), (0, 0, pos*1.1), (0, pos*.05, pos), (0, 0, pos),
              (pos, 0, pos), (pos, 0, neg), (neg, 0, neg), (neg, 0, pos), (neg*.05, 0, pos)]

    knots = [i for i in range(len(points))]

    icon = pm.curve(degree=1, p=points, k=knots, name=name)

    if colour:
        setColour(icon, colour)

    return icon