import pymel.core as pm

from tRigger.core import icon, attribute, dag, transform
reload(attribute)
reload(icon)
reload(transform)

class TGuide(object):
    def __init__(self):
        self.root = pm.createNode('transform', name='guide')
        self.controllers = dag.addChild(self.root, 'group', name='controllers')
        attribute.addBoolAttr(self.root, 'is_tGuide')

class TGuideBaseComponent(object):
    def __init__(self, name, guideType, side='C', index='0'):
        self.guide_name = name
        self.guide_type = guideType
        self.guide_side = side
        self.guide_index = self.requestIndex(index)
        self.root = self.addGuideRoot(self.getGuideParent())
        self.locs = [self.root]
        attribute.addStringAttr(self.root, 'guide_type', guideType)
        attribute.addStringAttr(self.root, 'guide_name', name)
        attribute.addStringAttr(self.root, 'guide_side', side)
        attribute.addIntAttr(self.root, 'guide_index', self.guide_index)

    def addGuideRoot(self, parent):
        root = icon.crossBoxIcon(name=self.getName('root'), colour='red')
        attribute.addBoolAttr(root, 'is_tGuide_root')
        if parent:
            root.setParent(parent)
            transform.align(root, parent)
        return root

    def addGuideLoc(self, name, mtx, parent=None):
        loc = icon.crossBallIcon(name=name, colour='yellow')
        attribute.addBoolAttr(loc, 'is_tGuide_loc')
        if not parent:
            parent = self.root
        loc.setParent(parent)
        pm.xform(loc, m=mtx, ws=0)
        self.locs.append(loc)
        return loc

    def getName(self, name):
        return '%s_%s%s_%s_guide' % (self.guide_name, self.guide_side, self.guide_index, name)

    def requestIndex(self, index):
        guideNodes = [node for node in pm.ls(type='transform') if node.hasAttr('guide_type')]
        guideIndices = [node.guide_index.get() for node in guideNodes if node.guide_type.get() == self.guide_type]
        if not index in guideIndices:
            return index
        else:
            return max(guideIndices) + 1

    def getGuideParent(self):
        parent = None
        sel = pm.selected()
        if sel:
            if sel[0].hasAttr('is_tGuide_root') or sel[0].hasAttr('is_tGuide_loc'):
                parent = sel[0]
        else:
            try:
                parent = pm.PyNode('guide')
            except:
                guide = TGuide()
                parent = guide.root
        return parent

def validateGuideRoot(guideRoot):
    if not guideRoot or not guideRoot.hasAttr('is_tGuide'):
        sel = pm.selected()
        if sel:
            if sel[0].hasAttr('is_tGuide'):
                guideRoot = sel[0]
    return guideRoot

def compileGuide(guideRoot=None):
    '''
    :param guideRoot: Instance of TGuide class.
    :return: Dictionary representing all the guide components ready to be built
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'
    compRoots = findComponentRoots(guideRoot)
    if not compRoots:
        return 'Specified Guide contains no valid components'
    guideDict = {}
    for root in compRoots:
        guideDict[root.name()] = {}
    print '\n'.join(guideDict.keys())

def findComponentRoots(guideRoot=None):
    '''
    :param guideRoot: Instance of TGuide class.
    :return: List of component root nodes in specified guide hierarchy
    '''
    guideRoot = validateGuideRoot(guideRoot)
    if not guideRoot:
        return 'Please supply a valid TGuide root node'

    compRoots = [node for node in pm.listRelatives(guideRoot, c=1, ad=1, s=0) if node.hasAttr('guide_type')]
    return compRoots