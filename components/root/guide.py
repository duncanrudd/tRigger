import pymel.core as pm

from tRigger.components import guide
reload(guide)

class TRootGuide(guide.TGuideBaseComponent):
    def __init__(self, name, side='C', index=0,  fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, name, 'root', side, index,  fromDagNode=fromDagNode)

def instantiateFromDagNode(dagNode):
    return TRootGuide(dagNode.guide_name.get(),
                      dagNode.guide_side.get(),
                      dagNode.guide_index.get(),
                      fromDagNode=dagNode)
