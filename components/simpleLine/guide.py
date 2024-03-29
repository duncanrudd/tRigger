import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute
from maya.api import OpenMaya as om2
reload(guide)


class TSimpleLineGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_segments=6, num_joints=8, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'simpleLine', guide_side, guide_index, fromDagNode=fromDagNode)
        for param in ['num_segments', 'num_joints']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_segments', num_segments)
            attribute.addIntAttr(self.root, 'num_joints', num_segments)
            attribute.addBoolAttr(self.root, 'add_joint')
            self.crv = None
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.num_segments = self.root.num_segments.get()
            self.num_joints = self.root.num_joints.get()
            self.crv = pm.PyNode(self.getName(self.getName('crv')))

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
        locs = self.locs
        locs.remove(self.root)
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
            mtx = transform.getMatrixFromPos((6*i, 0, 0))
            loc = self.addGuideLoc(self.getName(num), mtx, self.root)
            if loc.name() in locsDict.keys():
                pm.xform(loc, m=locsDict[loc.name()])
        self.crv = self.addGuideCurve(self.locs[1:], name=self.getName('crv'), degree=3)
        self.locs = self.getGuideLocs(self.root)


def instantiateFromDagNode(dagNode):
    return TSimpleLineGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_segments.get(),
                         dagNode.num_joints.get(),
                         fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TSimpleLineGuide(**kwargs)
