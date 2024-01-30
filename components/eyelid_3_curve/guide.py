import pymel.core as pm
import math

from tRigger.components import guide, attribute, transform
reload(guide)

class TEyelid_3_curveGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, add_joint=1, local_rig=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'eyelid_3_curve', guide_side, guide_index, fromDagNode=fromDagNode)
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
        radius = 10
        xform = transform.getMatrixFromPos((0, 0, radius*5))
        self.aimLoc = self.addGuideLoc(self.getName('aim'), xform, self.root, size=radius*.25)
        self.addSpaceSwitchAttr(self.aimLoc)



def instantiateFromDagNode(dagNode):
    return TEyelid_3_curveGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         add_joint=dagNode.add_joint.get(),
                         local_rig=dagNode.local_rig.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TEyelid_3_curveGuide(**kwargs)

