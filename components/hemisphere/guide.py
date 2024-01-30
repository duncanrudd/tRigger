import pymel.core as pm

from tRigger.components import guide, attribute
reload(guide)

class THemisphereGuide(guide.TGuideBaseComponent):
    '''
    A simple fk shoulder controller with an offsetable orbit control
    '''
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_spans=5, add_joint=1, local_rig=1, flip=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'hemisphere', guide_side, guide_index, fromDagNode=fromDagNode)
        self.num_spans = num_spans
        self.add_joint = add_joint
        self.local_rig = local_rig
        self.flip = flip
        for param in ['num_spans', 'add_joint', 'local_rig', 'flip']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_spans')
            attribute.addBoolAttr(self.root, 'add_joint')
            attribute.addBoolAttr(self.root, 'local_rig')
            attribute.addBoolAttr(self.root, 'flip')
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)

    def addLocs(self):
        self.locs = [self.root]
        xform = self.root.worldMatrix[0].get()
        start = self.addGuideLoc(self.getName('start'), xform, self.root)
        start.tx.set(-5)

def instantiateFromDagNode(dagNode):
    return THemisphereGuide(dagNode.guide_name.get(),
                            dagNode.guide_side.get(),
                            dagNode.guide_index.get(),
                            num_spans=dagNode.num_spans.get(),
                            add_joint=dagNode.add_joint.get(),
                            local_rig=dagNode.local_rig.get(),
                            flip=dagNode.flip.get(),
                            fromDagNode=dagNode)

def buildGuide(**kwargs):
    return THemisphereGuide(**kwargs)
