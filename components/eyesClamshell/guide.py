import pymel.core as pm
import math

from tRigger.components import guide, attribute, transform
reload(guide)

class TEyesClamshellGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, add_joint=1, local_rig=1, lidSpans=0, ballSpans=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'eyesClamshell', guide_side, guide_index, fromDagNode=fromDagNode)
        self.add_joint = add_joint
        self.local_rig = local_rig
        self.lidSpans = lidSpans
        self.ballSpans = ballSpans
        for param in ['add_joint', 'local_rig', 'lidSpans', 'ballSpans']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
            attribute.addBoolAttr(self.root, 'local_rig', local_rig)
            attribute.addIntAttr(self.root, 'lidSpans', lidSpans)
            attribute.addIntAttr(self.root, 'ballSpans', ballSpans)
            self.addLocs()
        else:
            self.root.add_joint.set(self.add_joint)
            self.root.local_rig.set(self.local_rig)
            self.root.lidSpans.set(self.lidSpans)
            self.root.ballSpans.set(self.ballSpans)
            self.locs = self.getGuideLocs(fromDagNode)

    def addLocs(self):
        xform = transform.getMatrixFromPos((0, 0, 20))
        self.aimLoc = self.addGuideLoc(self.getName('aim'), xform, self.root, size=5)
        xform = transform.getMatrixFromPos((0, 0, 0))
        self.upper = self.addGuideLoc(self.getName('upperRadius'), xform, self.root)
        self.upper.ty.set(5)
        self.lower = self.addGuideLoc(self.getName('lowerRadius'), xform, self.root)
        self.lower.ty.set(-5)
        self.eyeball = self.addGuideLoc(self.getName('eyeballRadius'), xform, self.root)
        self.eyeball.tz.set(4)
        self.addSpaceSwitchAttr(self.aimLoc)



def instantiateFromDagNode(dagNode):
    return TEyesClamshellGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         add_joint=dagNode.add_joint.get(),
                         local_rig=dagNode.local_rig.get(),
                         lidSpans=dagNode.lidSpans.get(),
                         ballSpans=dagNode.ballSpans.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TEyesClamshellGuide(**kwargs)

