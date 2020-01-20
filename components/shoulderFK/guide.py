import pymel.core as pm

from tRigger.components import guide, attribute
reload(guide)

class TShoulderFKGuide(guide.TGuideBaseComponent):
    '''
    A simple fk shoulder controller with an offsetable orbit control
    '''
    def __init__(self, guide_name='', guide_side='C', guide_index=0, axis='x', up_axis='y', add_joint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'shoulderFK', guide_side, guide_index, fromDagNode=fromDagNode)
        self.axis = axis
        self.up_axis = up_axis
        for param in ['axis', 'up_axis', 'add_joint']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addStringAttr(self.root, 'axis', axis)
            attribute.addStringAttr(self.root, 'up_axis', up_axis)
            attribute.addBoolAttr(self.root, 'add_joint')
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)

    def addLocs(self):
        self.locs = [self.root]
        xform = self.root.worldMatrix[0].get()
        tip = self.addGuideLoc(self.getName('tip'), xform, self.root)
        tip.tx.set(10)
        orbit = self.addGuideLoc(self.getName('orbit'), xform, self.root)
        orbit.tx.set(12.5)
        self.crv = self.addGuideCurve([self.root, tip], name='crv', degree=1)
        self.orbit_crv = self.addGuideCurve([self.root, orbit], name='orbit_crv', degree=1)
        self.upNode = self.addGuideUpNode(self.up_axis)

def instantiateFromDagNode(dagNode):
    return TShoulderFKGuide(dagNode.guide_name.get(),
                            dagNode.guide_side.get(),
                            dagNode.guide_index.get(),
                            dagNode.axis.get(),
                            dagNode.up_axis.get(),
                            add_joint=dagNode.add_joint.get(),
                            fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TShoulderFKGuide(**kwargs)
