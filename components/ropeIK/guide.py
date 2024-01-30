import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute
from maya.api import OpenMaya as om2
reload(guide)


class TRopeIKGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_segments=6, num_divisions=12, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'ropeIK', guide_side, guide_index, fromDagNode=fromDagNode)
        for param in ['num_segments', 'num_divisions']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_segments', num_segments)
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            attribute.addBoolAttr(self.root, 'add_joint')
            self.crv = None
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.num_segments = self.root.num_segments.get()
            self.num_divisions = self.root.num_divisions.get()
            self.crv = pm.PyNode(self.getName(self.getName('crv')))


    def addLocs(self):
        for i in range(4):
            num = str(i+1).zfill(2)
            mtx = transform.getMatrixFromPos((6*i, 0, 0))
            loc = self.addGuideLoc(self.getName(num), mtx, self.root)
            if i == 3:
                self.addSpaceSwitchAttr(loc)
        self.crv = self.addGuideCurve(self.locs, name=self.getName('crv'), degree=3)
        self.locs = self.getGuideLocs(self.root)


def instantiateFromDagNode(dagNode):
    return TRopeIKGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_segments.get(),
                         dagNode.num_divisions.get(),
                         fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TRopeIKGuide(**kwargs)
