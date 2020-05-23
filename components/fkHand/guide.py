import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute
from maya.api import OpenMaya as om2
reload(guide)


class TFkHandGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_segments=6, skew_start=2, num_digits=4, thumb=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'fkHand', guide_side, guide_index, fromDagNode=fromDagNode)
        for param in ['num_segments', 'num_digits', 'thumb']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_digits', num_digits)
            attribute.addIntAttr(self.root, 'num_segments', num_segments)
            attribute.addIntAttr(self.root, 'skew_start', skew_start)
            attribute.addBoolAttr(self.root, 'thumb', thumb)
            attribute.addBoolAttr(self.root, 'add_joint')
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.num_segments = self.root.num_segments.get()
            self.num_digits = self.root.num_digits.get()
            self.skew_start = self.root.skew_start.get()
            self.thumb = self.root.thumb.get()
        self.installComponentCallbacks()

    def installComponentCallbacks(self):
        try:
            attribute.removeAttributeCallback(self.root, self.num_segments_CB)
            self.num_segments_CB = None
        except:
            pass
        self.num_segments_CB = attribute.addCallbackToAttr(self.root, 'num_segments_CB', self.num_segments_callback)
        guide.TGuideBaseComponent.installCallbacks(self)

    def num_segments_callback(self, msg, plug1, plug2, payload):
        if msg == 2056:
            mfn_dep = om2.MFnDependencyNode(plug1.node())
            plugNames = ['num_segments', 'num_digits', 'thumb']
            interestingPlugs = [mfn_dep.findPlug(plug, False) for plug in plugNames]
            if plug1 in interestingPlugs:
                sel = pm.selected()
                self.num_segments = self.root.num_segments.get()
                self.num_digits = self.root.num_digits.get()
                self.thumb = self.root.thumb.get()
                self.addLocs()
                pm.select(sel)
                print('callback fired')

    def addLocs(self):
        self.num_segments = self.root.num_segments.get()
        self.num_digits = self.root.num_digits.get()
        self.thumb = self.root.thumb.get()
        locs = self.locs
        locs.remove(self.root)
        locsDict = {}
        for loc in locs:
            locsDict[loc.name()] = pm.xform(loc, q=1, m=1)
        toDelete = [node for node in self.locs if not node == self.root]
        crvs = [node for node in pm.listRelatives(self.root, c=1, type='transform') if 'crv' in node.name()]
        pm.delete(toDelete)
        pm.delete(crvs)
        self.locs = [self.root]
        for d in range(self.num_digits + self.thumb):
            baseNum = str(d+1).zfill(2)
            fingLocs = [self.root]
            segs = self.num_segments
            prefix = baseNum
            if self.thumb and d == (self.num_digits + self.thumb - 1):
                prefix = 'thumb'
                segs -= 1
            for i in range(segs):
                zOffset = d*3 - (1.5*(self.num_digits-1))
                if i != 0:
                    zOffset = 0
                num = str(i+1).zfill(2)
                mtx = transform.getMatrixFromPos((6, 0, zOffset))
                loc = self.addGuideLoc(self.getName('%s_%s' % (prefix, num)), mtx, fingLocs[-1])
                if loc.name() in locsDict.keys():
                    pm.xform(loc, m=locsDict[loc.name()])
                fingLocs.append(loc)
            self.crv = self.addGuideCurve(fingLocs, name=self.guide_name + '_%s_crv' % prefix, degree=1)
        self.locs = self.getGuideLocs(self.root)


def instantiateFromDagNode(dagNode):
    return TFkHandGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_segments.get(),
                         dagNode.skew_start.get(),
                         dagNode.num_digits.get(),
                         dagNode.thumb.get(),
                         fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TFkHandGuide(**kwargs)

def updateGuide(guideRoot):
    if not guideRoot.hasAttr('skew_start'):
        attribute.addIntAttr(guideRoot, 'skew_start', 2)
