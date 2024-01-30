import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute
from maya.api import OpenMaya as om2
reload(guide)


class TFkDistributedGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_segments=6, sub_segments=3, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'fkDistributed', guide_side, guide_index, fromDagNode=fromDagNode)
        for param in ['num_segments', 'sub_segments']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_segments', num_segments)
            attribute.addIntAttr(self.root, 'sub_segments', sub_segments)
            attribute.addBoolAttr(self.root, 'add_joint')
            self.crv = None
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.num_segments = self.root.num_segments.get()
            self.sub_segments = self.root.sub_segments.get()

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
            plugNames = ['num_segments']
            interestingPlugs = [mfn_dep.findPlug(plug, False) for plug in plugNames]
            if plug1 in interestingPlugs:
                sel = pm.selected()
                self.num_segments = self.root.num_segments.get()
                self.addLocs()
                pm.select(sel)
                print('callback fired')

    def addLocs(self):
        self.num_segments = self.root.num_segments.get()
        locs = [loc for loc in self.locs if not loc == self.root]
        locsDict = {}
        for loc in locs:
            locsDict[loc.name()] = pm.xform(loc, q=1, m=1)
        toDelete = [node for node in self.locs if not node == self.root]
        pm.delete(toDelete)
        if self.crv:
            pm.delete(self.crv)
        self.locs = [self.root]

        for i in range(self.num_segments):
            num = str(i+1).zfill(2)
            mtx = transform.getMatrixFromPos((6, 0, 0))
            loc = self.addGuideLoc(self.getName(num), mtx, self.locs[-1])
            if loc.name() in locsDict.keys():
                pm.xform(loc, m=locsDict[loc.name()])
        self.crv = self.addGuideCurve(self.locs, name=self.getName('crv'), degree=1)
        self.locs = self.getGuideLocs(self.root)


def instantiateFromDagNode(dagNode):
    return TFkDistributedGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_segments.get(),
                         dagNode.sub_segments.get(),
                         fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TFkDistributedGuide(**kwargs)
