import pymel.core as pm

from tRigger.components import guide, attribute
reload(guide)

class TControlGuide(guide.TGuideBaseComponent):
    def __init__(self, name, side='C', index=0, numCtrls=3, addJoint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, name, 'control', side, index,  fromDagNode=fromDagNode)
        self.num_ctrls = numCtrls
        for param in ['num_ctrls']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_ctrls', numCtrls)
            attribute.addBoolAttr(self.root, 'add_joint')

def instantiateFromDagNode(dagNode):
    return TControlGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         numCtrls=dagNode.num_ctrls.get(),
                         addJoint=dagNode.add_joint.get(),
                         fromDagNode=dagNode)
