import pymel.core as pm
import math

from tRigger.components import guide, attribute, transform
reload(guide)

class TEyelidGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_ctrls=3, add_joint=1, local_rig=1,
                 bias_tweak_ctrls=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'eyelid', guide_side, guide_index, fromDagNode=fromDagNode)
        self.num_ctrls = num_ctrls
        self.add_joint = add_joint
        self.local_rig = local_rig
        self.bias_tweak_ctrls = bias_tweak_ctrls
        for param in ['num_ctrls', 'add_joint', 'local_rig', 'bias_tweak_ctrls']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_ctrls', num_ctrls)
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
            attribute.addBoolAttr(self.root, 'local_rig', local_rig)
            attribute.addBoolAttr(self.root, 'bias_tweak_ctrls', value=False)
            self.addLocs()
        else:
            self.root.num_ctrls.set(self.num_ctrls)
            self.root.add_joint.set(self.add_joint)
            self.root.local_rig.set(self.local_rig)
            self.root.bias_tweak_ctrls.set(self.bias_tweak_ctrls)
            self.locs = self.getGuideLocs(fromDagNode)
            self.upperCrv = pm.PyNode(self.getName('upper_crv'))
            self.lowerCrv = pm.PyNode(self.getName('lower_crv'))

    def addLocs(self):
        radius = 10
        self.upperLocs = []
        self.lowerLocs = []
        angles = [math.pi*.333 + (math.pi*.5), math.pi*.167 + (math.pi*.5), (math.pi*.5), math.pi*-.167 + (math.pi*.5), math.pi*-.333 + (math.pi*.5)]
        for i in range(5):
            num = str(i+1).zfill(2)
            aimVec = pm.datatypes.Vector(math.cos(angles[i]), 0, math.sin(angles[i]))
            pos = aimVec * radius
            upVec = pm.datatypes.Vector(0, 1, 0)
            sideVec = upVec.cross(aimVec)
            xform = pm.datatypes.Matrix(sideVec, upVec, aimVec, pos)
            loc = self.addGuideLoc(self.getName('upper_%s' % num), xform, self.root, size=radius*.25)
            self.upperLocs.append(loc)
            if 0 < i < 4:
                loc = self.addGuideLoc(self.getName('lower_%s' % num), xform, self.root, size=radius*.25, colour='blue')
                self.lowerLocs.append(loc)
        xform = transform.getMatrixFromPos((0, 0, radius*5))
        self.aimLoc = self.addGuideLoc(self.getName('aim'), xform, self.root, size=radius*.25)
        self.addSpaceSwitchAttr(self.aimLoc)
        self.upperCrv = self.addGuideCurve(self.upperLocs, name='upper_crv', degree=2)
        self.lowerCrv = self.addGuideCurve([self.upperLocs[0]] + self.lowerLocs + [self.upperLocs[-1]], name='lower_crv', degree=2)



def instantiateFromDagNode(dagNode):
    return TEyelidGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         num_ctrls=dagNode.num_ctrls.get(),
                         add_joint=dagNode.add_joint.get(),
                         local_rig=dagNode.local_rig.get(),
                         bias_tweak_ctrls=dagNode.bias_tweak_ctrls.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TEyelidGuide(**kwargs)

def updateGuide(guideRoot):
    if not guideRoot.hasAttr('bias_tweak_ctrls'):
        attribute.addBoolAttr(guideRoot, 'bias_tweak_ctrls', value=False)

