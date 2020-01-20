import pymel.core as pm

from tRigger.components import guide, attribute
reload(guide)

class TControlGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_ctrls=3, add_joint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'control', guide_side, guide_index, fromDagNode=fromDagNode)
        self.num_ctrls = num_ctrls
        for param in ['num_ctrls']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_ctrls', num_ctrls)
            attribute.addBoolAttr(self.root, 'add_joint')

def instantiateFromDagNode(dagNode):
    return TControlGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         num_ctrls=dagNode.num_ctrls.get(),
                         add_joint=dagNode.add_joint.get(),
                         fromDagNode=dagNode)
