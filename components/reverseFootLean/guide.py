import pymel.core as pm
from maya.api import OpenMaya as om2
from tRigger.core import transform
from tRigger.components import guide, attribute
reload(guide)

class TReverseFootLeanGuide(guide.TGuideBaseComponent):
    '''
    A simple reverse foot component
    '''
    def __init__(self, guide_name='', guide_side='C', guide_index=0, add_joint=1, attr_driven=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'reverseFootLean',
                                           guide_side, guide_index, fromDagNode=fromDagNode)
        self.add_joint = add_joint
        self.attr_driven = attr_driven
        for param in ['add_joint', 'attr_driven']:
            self.params.append(param)
            self.attr_driven = attr_driven
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint')
            attribute.addBoolAttr(self.root, 'attr_driven')
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)

    def addLocs(self):
        xform = pm.datatypes.Matrix()
        
        self.heelLoc = self.addGuideLoc(self.getName('heel'), xform, self.root)
        self.heelLoc.t.set((-2, -5, 0))
        self.addSpaceSwitchAttr(self.heelLoc)
        
        self.ballLoc = self.addGuideLoc(self.getName('ball'), xform, self.root)
        self.ballLoc.t.set((8, -5, 0))
        
        self.tipLoc = self.addGuideLoc(self.getName('tip'), xform, self.root)
        self.tipLoc.t.set((12, -5, 0))
        
        self.toeLoc = self.addGuideLoc(self.getName('toe'), xform, self.root)
        self.toeLoc.t.set((8, -3, 0))
        
        self.outerLoc = self.addGuideLoc(self.getName('outer'), xform, self.root)
        self.outerLoc.t.set((4, -5, -2.5))
        
        self.innerLoc = self.addGuideLoc(self.getName('inner'), xform, self.root)
        self.innerLoc.t.set((4, -5, 2.5))

        self.leanLoc = self.addGuideLoc(self.getName('lean'), xform, self.root)
        self.leanLoc.t.set((4, -5, 0))
        for loc in self.locs:
            if loc != self.root:
                loc.ry.set(180.0)


def instantiateFromDagNode(dagNode):
    return TReverseFootLeanGuide(dagNode.guide_name.get(),
                            dagNode.guide_side.get(),
                            dagNode.guide_index.get(),
                            add_joint=dagNode.add_joint.get(),
                            attr_driven=dagNode.attr_driven.get(),
                            fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TReverseFootLeanGuide(**kwargs)
