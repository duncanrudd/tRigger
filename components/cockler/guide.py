import pymel.core as pm
from tRigger.core import transform
from tRigger.components import guide, attribute
reload(guide)

class TCocklerGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, add_joint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'cockler', guide_side, guide_index, fromDagNode=fromDagNode)
        self.add_joint = add_joint
        for param in ['add_joint']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
            self.addLocs()
        else:
            self.root.add_joint.set(self.add_joint)
            self.locs = self.getGuideLocs(self.root)

    def addLocs(self):
        xform = transform.getMatrixFromPos((-5, 0, 0))
        innerLoc = self.addGuideLoc(self.getName('inner'), xform, self.root)

        xform = transform.getMatrixFromPos((5, 0, 0))
        outerLoc = self.addGuideLoc(self.getName('outer'), xform, self.root)

        xform = transform.getMatrixFromPos((0, 0, 5))
        frontLoc = self.addGuideLoc(self.getName('front'), xform, self.root)

        xform = transform.getMatrixFromPos((0, 0, -5))
        backLoc = self.addGuideLoc(self.getName('back'), xform, self.root)
        self.locs = self.getGuideLocs(self.root)


def instantiateFromDagNode(dagNode):
    return TCocklerGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         add_joint=dagNode.add_joint.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TCocklerGuide(**kwargs)
