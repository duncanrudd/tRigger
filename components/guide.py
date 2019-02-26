import pymel.core as pm

from tRigger.core import icon, attribute, dag
reload(attribute)

class TGuide(object):
    def __init__(self):
        self.root = pm.createNode('transform', name='guide')
        self.controllers = dag.addChild(self.root, 'group', name='controllers')
        attribute.addBoolAttr(self.root, 'is_tGuide')

class TGuideBaseComponent(object):
    def __init__(self, name, guideType, side='C', index='0'):
        self.parent = self.getGuideParent()
        self.guide_name = name
        self.guide_type = guideType
        self.guide_side = side
        self.guide_index = self.requestIndex(index)
        self.root = self.addGuideRoot()
        self.root.setParent(self.parent)
        attribute.addStringAttr(self.root, 'guide_type', guideType)
        attribute.addStringAttr(self.root, 'guide_name', name)
        attribute.addStringAttr(self.root, 'guide_side', side)
        attribute.addIntAttr(self.root, 'guide_index', self.guide_index)

    def addGuideRoot(self):
        root = icon.crossBoxIcon(name=self.getName('root'))
        attribute.addBoolAttr(root, 'is_tGuide_node')
        return root

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
            if sel[0].hasAttr('is_tGuide_node'):
                parent = sel[0]
        else:
            try:
                parent = pm.PyNode('guide')
            except:
                guide = TGuide()
                parent = guide.root
        return parent