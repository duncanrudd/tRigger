import pymel.core as pm
from maya.api import OpenMaya as om2
import os

from tRigger.core import icon, attribute, dag, transform, curve
reload(attribute)
reload(icon)
reload(transform)
reload(curve)

class TGuide(object):
    def __init__(self):
        self.root = pm.createNode('transform', name='guide')
        self.controllers = dag.addChild(self.root, 'group', name='controllers')
        attribute.addBoolAttr(self.root, 'is_tGuide')
        attribute.addColourAttr(self.root, 'centre_colour')
        attribute.addColourAttr(self.root, 'left_colour')
        attribute.addColourAttr(self.root, 'right_colour')

class TGuideBaseComponent(object):
    def __init__(self, name, guideType, side='C', index='0', fromDagNode=0):
        self.guide_name = name
        self.guide_type = guideType
        self.guide_side = side
        self.params = ['guide_name', 'guide_side', 'guide_index']
        if not fromDagNode:
            self.guide_index = self.requestIndex(index)
            self.root = self.addGuideRoot(self.requestGuideParent())
            attribute.addStringAttr(self.root, 'guide_name', name)
            attribute.addStringAttr(self.root, 'guide_side', side)
            attribute.addStringAttr(self.root, 'guide_type', guideType)
            attribute.addIntAttr(self.root, 'guide_index', self.guide_index)
        else:
            self.root = fromDagNode
            self.guide_index = index
        self.locs = [self.root]
        self.installCallbacks()

    def installCallbacks(self):
        try:
            attribute.removeAttributeCallback(self.root, self.guide_name_CB)
            self.guide_name_CB = None
        except:
            pass
        self.guide_name_CB = attribute.addCallbackToAttr(self.root, 'guide_name', self.guide_name_callback)

        print 'Callbacks Installed'

    def guide_name_callback(self, msg, plug1, plug2, payload):
        if msg == 2056:
            mfn_dep = om2.MFnDependencyNode(plug1.node())
            plugNames = ['guide_name', 'guide_side', 'guide_index']
            interestingPlugs = [mfn_dep.findPlug(plug, False) for plug in plugNames]
            rename = False
            if plug1 in interestingPlugs:
                self.guide_name = self.root.guide_name.get()
                self.guide_side = self.root.guide_side.get()
                validIndex = self.requestIndex(self.root.guide_index.get(), excludeSelf=1)
                if not validIndex == self.root.guide_index.get():
                    self.root.guide_index.set(validIndex)
                self.guide_index = self.root.guide_index.get()
                self.renameGuide()


# -------------------------------------------------------------------
    # Add Objects to Guide
    # -------------------------------------------------------------------
    def addGuideRoot(self, parent):
        root = icon.crossBoxIcon(name=self.getName('root'), colour='red', size=6)
        attribute.addBoolAttr(root, 'is_tGuide_root')
        if parent:
            root.setParent(parent)
            transform.align(root, parent)
        self.addSpaceSwitchAttr(root)
        return root

    def addGuideLoc(self, name, mtx, parent=None, size=5, colour='yellow', locType='loc'):
        loc = icon.crossBallIcon(name=name, colour=colour, size=size)
        attribute.addBoolAttr(loc, 'is_tGuide_%s' % locType)
        if not parent:
            parent = self.root
        loc.setParent(parent)
        pm.xform(loc, m=mtx, ws=0)
        self.locs.append(loc)
        return loc

    def addSpaceSwitchAttr(self, node):
        attribute.addStringAttr(node, 'spaces', '')
        attribute.addBoolAttr(node, 'splitTranslateAndRotate')

    def addGuideUpNode(self, axis='y'):
        loc = self.addGuideLoc(self.getName('upNode'),
                               pm.datatypes.Matrix(),
                               self.root)
        icon.setColour(loc, 'cyan')
        pm.setAttr('%s.t%s' % (loc.name(), axis), 5.0)
        self.addGuideCurve([self.root, loc, self.locs[1]], name='upCrv', degree=1)
        return loc

    def addGuideCurve(self, nodes, name, degree=1):
        crv = curve.curveThroughPoints(self.getName(name), nodes, degree)
        curve.driveCurve(crv, nodes)
        crv.inheritsTransform.set(0)
        crv.setParent(self.root)
        pm.xform(crv, ws=1, m=pm.datatypes.Matrix())
        dag.setDisplayType(crv, 'template')
        return crv

    # -------------------------------------------------------------------
    # Get information about the Guide
    # -------------------------------------------------------------------
    def getName(self, name):
        return '%s_%s%s_%s_guide' % (self.guide_name, self.guide_side, self.guide_index, name)

    def requestIndex(self, index, excludeSelf=0):
        guideNodes = [node for node in pm.ls(type='transform') if node.hasAttr('guide_type')]
        if excludeSelf:
            guideNodes.remove(self.root)
        guideIndices = [node.guide_index.get() for node in guideNodes if node.guide_name.get() == self.guide_name and node.guide_side.get() == self.guide_side]
        if not index in guideIndices:
            return index
        else:
            return max(guideIndices) + 1

    def requestGuideParent(self):
        parent = None
        sel = pm.selected()
        if sel:
            if sel[0].hasAttr('is_tGuide_root') or sel[0].hasAttr('is_tGuide_loc') or sel[0].hasAttr('is_tGuide_div'):
                parent = sel[0]
        else:
            try:
                parent = pm.PyNode('guide')
            except:
                guide = TGuide()
                parent = guide.root
        return parent

    def getGuideParent(self):
        parent = self.root.getParent()
        if parent:
            if parent.hasAttr('is_tGuide_root') or parent.hasAttr('is_tGuide_loc') or parent.hasAttr('is_tGuide_div'):
                return parent
            else:
                return None

    def getGuideLocs(self, root, locType='loc'):
        locs = []
        if locType == 'loc':
            locs.append(root)
        guide_id = '%s_%s%s' % (self.guide_name, self.guide_side, self.guide_index)
        childLocs = [node for node in pm.listRelatives(root, c=1, ad=1, s=0)
                     if node.hasAttr('is_tGuide_%s' % locType)
                     and guide_id in node.name()]
        print guide_id
        return locs + childLocs

    def getRequiredInputs(self):
        switchNodes = [node for node in self.locs if node.hasAttr('spaces')]
        inputs = []
        for node in switchNodes:
            spaces = [space.split(':')[1] for space in node.spaces.get().split(',')]
            for space in spaces:
                if not space in inputs:
                    inputs.append(space)
        return inputs

    def getAllGuideNodes(self):
        '''
        Returns all nodes that are part of the component guide
        Returns:(list)
        '''


    def renameGuide(self):
        '''
        Called via a callback whenever the component name, side or index is changed.
        This function renames all component nodes to reflect the changes.
        Returns:
        '''
        searchString = '_'.join(self.root.name().split('_')[:2])
        print searchString
        nodes = pm.ls('%s*' % searchString, type='transform')
        prefix = '%s_%s%s' % (self.guide_name, self.guide_side, self.guide_index)
        for node in nodes:
            name = '_'.join(node.name().split('_')[2:-1])
            node.rename('%s_%s_guide' % (prefix, name))






