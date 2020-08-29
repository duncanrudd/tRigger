import pymel.core as pm
import math

from tRigger.components import guide, attribute, transform
reload(guide)

class TEyesGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, add_joint=1, local_rig=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'eyes', guide_side, guide_index, fromDagNode=fromDagNode)
        self.add_joint = add_joint
        self.local_rig = local_rig
        for param in ['add_joint', 'local_rig']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
            attribute.addBoolAttr(self.root, 'local_rig', local_rig)
            self.addLocs()
        else:
            self.root.add_joint.set(self.add_joint)
            self.root.local_rig.set(self.local_rig)
            self.locs = self.getGuideLocs(fromDagNode)

    def addLocs(self):
        xform = transform.getMatrixFromPos((0, 0, 20))
        self.aimLoc = self.addGuideLoc(self.getName('aim'), xform, self.root, size=5)
        self.addSpaceSwitchAttr(self.aimLoc)



def instantiateFromDagNode(dagNode):
    return TEyesGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         add_joint=dagNode.add_joint.get(),
                         local_rig=dagNode.local_rig.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TEyesGuide(**kwargs)

