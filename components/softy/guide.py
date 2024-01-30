import pymel.core as pm

from tRigger.components import guide, attribute
reload(guide)

class TSoftyGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, add_joint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'softy', guide_side, guide_index, fromDagNode=fromDagNode)
        self.add_joint = add_joint
        for param in ['add_joint']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
        else:
            self.root.add_joint.set(self.add_joint)

def instantiateFromDagNode(dagNode):
    return TSoftyGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         add_joint=dagNode.add_joint.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TSoftyGuide(**kwargs)
