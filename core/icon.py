import pymel.core as pm
import math

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


def setColourRGB(node, colour, offsets={'_fk_': .3, '_ik_': -.3}):
    rgb = ("R", "G", "B")

    node.overrideEnabled.set(1)
    node.overrideRGBColors.set(1)

    for channel, value in zip(rgb, colour):
        for key in offsets.keys():
            if key in node.name():
                value += offsets[key]
        pm.Attribute('%s.overrideColor%s' % (node.name(), channel)).set(value)

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

def circlePointIcon(size=20.0, name='', colour=None):
    '''
    creates a circle shaped nurbs curve with a point at the front
    '''
    radius = size*0.5
    points = []
    for i in range(24):
        interval = math.radians(15*(i+1))
        points.append((math.sin(interval)*radius, 0, math.cos(interval)*radius))
    points.append((0, radius*.05, radius))
    points.append((0, 0, radius*1.1))
    points.append((0, radius*-.05, radius))

    knots = [i for i in range(len(points))]

    icon = pm.curve(degree=1, p=points, k=knots, name=name)

    if colour:
        setColour(icon, colour)

    return icon

def pringleIcon(size=20.0, name='', colour=None):
    radius = size*0.5
    icon = pm.circle(radius=radius, r=radius, ch=0, o=1, s=8, nr=(0, 1, 0))[0]
    if name:
        icon.rename(name)
    pm.select([icon.cv[i] for i in [3, 7]])
    pm.move((0, size*.6, 0), r=1)
    pm.select([icon.cv[i] for i in [0, 2, 4, 6]])
    pm.move((0, size*0.4, 0), r=1)
    pm.select([icon.cv[i] for i in [5, 1]])
    pm.move((0, size*0.2, 0), r=1)

    if colour:
        setColour(icon, colour)

    return icon

def triNorthIcon(size=20, name='', colour=None):
    radius = size*0.5
    points = [(0, radius*1.5, 0),
              (radius, 0, 0),
              (-radius, 0, 0),
              (0, radius*1.5, 0)]
    icon = pm.curve(p=points, k=[1, 2, 3, 4], degree=1)
    if name:
        icon.rename(name)
    if colour:
        setColour(icon, colour)
    return icon

def triSouthIcon(size=20, name='', colour=None):
    icon = triNorthIcon(size, name, colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((0, 0, 180), r=1)
    pm.select(icon)
    return icon

def triEastIcon(size=20, name='', colour=None):
    icon = triNorthIcon(size, name, colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((0, 0, 90), r=1)
    pm.select(icon)
    return icon

def triWestIcon(size=20, name='', colour=None):
    icon = triNorthIcon(size, name, colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((0, 0, -90), r=1)
    pm.select(icon)
    return icon

def gearIcon(size=10, name='', colour=None):
    radius = size*0.5
    points = []
    teeth = 10
    step = 6.2832 / teeth
    for i in range(teeth):
        angle = i * step
        angle += step * 0.25
        points.append((math.sin(angle)*radius, math.cos(angle)*radius, 0))
        points.append((math.sin(angle)*(radius*.8), math.cos(angle)*(radius*.8), 0))
        angle += step*.5
        points.append((math.sin(angle)*(radius*.8), math.cos(angle)*(radius*.8), 0))
        points.append((math.sin(angle)*radius, math.cos(angle)*radius, 0))
    points.append(points[0])

    icon = pm.curve(p=points, k=range(len(points)), degree=1)
    if name:
        icon.rename(name)
    if colour:
        setColour(icon, colour)
    return icon

def arrowForwardIcon(size=10, name='', colour=None):
    points = [(0, 0, 0), (0, size*-.2, 0), (0, size*-.2, size*.6), (0, size*-.4, size*.6), (0, 0, size),
              (0, size*.4, size*.6), (0, size*.2, size*.6), (0, size*.2, 0), (0, 0, 0)]
    knots = range(len(points))
    icon = pm.curve(p=points, k=range(len(points)), degree=1)
    if name:
        icon.rename(name)
    if colour:
        setColour(icon, colour)
    return icon

def arrowUpIcon(size=10, name='', colour=None):
    icon = arrowForwardIcon(size=size, name=name, colour=colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((-90, 0, 0), r=1)
    pm.select(icon)
    return icon

def arrowDownIcon(size=10, name='', colour=None):
    icon = arrowForwardIcon(size=size, name=name, colour=colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((90, 0, 0), r=1)
    pm.select(icon)
    return icon

def arrowBackIcon(size=10, name='', colour=None):
    icon = arrowForwardIcon(size=size, name=name, colour=colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((-180, 0, 0), r=1)
    pm.select(icon)
    return icon

def arrowLeftIcon(size=10, name='', colour=None):
    icon = arrowForwardIcon(size=size, name=name, colour=colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((0, -90, 0), r=1)
    pm.select(icon)
    return icon

def arrowRightIcon(size=10, name='', colour=None):
    icon = arrowForwardIcon(size=size, name=name, colour=colour)
    pm.select('%s.cv[*]' % getShape(icon).name())
    pm.rotate((0, 90, 0), r=1)
    pm.select(icon)
    return icon

