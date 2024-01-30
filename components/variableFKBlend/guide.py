import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute, curve, mathOps
from maya.api import OpenMaya as om2
reload(guide)
reload(mathOps)

axisDict = {'x': pm.datatypes.Vector(10, 0, 0),
            'y': pm.datatypes.Vector(0, 10, 0),
            'z': pm.datatypes.Vector(0, 0, 10),
            }

class TVariableFKBlendGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_divisions=4, num_ctrls=4, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'variableFKBlend', guide_side, guide_index,
                                           fromDagNode=fromDagNode)
        self.num_divisions = num_divisions
        self.num_ctrls = num_ctrls
        self.divisionLocs = []
        self.ctrlLocs = []
        for param in ['num_divisions', 'num_ctrls']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            attribute.addIntAttr(self.root, 'num_ctrls', value=num_ctrls, minValue=4)
            self.addLocs()
            attribute.addBoolAttr(self.root, 'add_joint')
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.crv = pm.PyNode(self.getName('crv'))

    def addLocs(self):
        for i in range(3):
            xform = transform.getMatrixFromPos((0, 5*(i+1), 0))
            num = str(i+1).zfill(2)
            self.addGuideLoc(self.getName(num), xform, self.root)

        self.crv = self.addGuideCurve(self.locs, name='crv', degree=2)

        self.parentLoc = self.addGuideLoc(self.getName('parent'), self.root.worldMatrix[0].get(), self.root, size=2)

        self.locs = self.getGuideLocs(self.root)

        self.addSpaceSwitchAttr(self.parentLoc)

def instantiateFromDagNode(dagNode):
    return TVariableFKBlendGuide(dagNode.guide_name.get(),
                           dagNode.guide_side.get(),
                           dagNode.guide_index.get(),
                           dagNode.num_divisions.get(),
                           dagNode.num_ctrls.get(),
                           fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TVariableFKBlendGuide(**kwargs)

