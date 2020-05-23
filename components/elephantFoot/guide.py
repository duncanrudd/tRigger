import pymel.core as pm

from tRigger.components import guide, attribute, transform
reload(guide)

class TElephantFootGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_ctrls=8, add_joint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'elephantFoot', guide_side, guide_index, fromDagNode=fromDagNode)
        self.num_ctrls = num_ctrls
        self.add_joint = add_joint
        for param in ['num_ctrls', 'add_joint']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_ctrls', num_ctrls)
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
            self.addLocs()
        else:
            self.root.num_ctrls.set(self.num_ctrls)
            self.root.add_joint.set(self.add_joint)
            self.locs = self.getGuideLocs(self.root)

    def addLocs(self):
        mtx = pm.datatypes.Matrix()
        for index, locName in enumerate(['inner', 'outer']):
            loc = pm.circle(name=self.getName(locName), radius=(index+1)*2, normal=(0, 1, 0))[0]
            attribute.addBoolAttr(loc, 'is_tGuide_loc')
            loc.setParent(self.root)
            pm.xform(loc, m=mtx, ws=0)
            self.locs.append(loc)
            attrList = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz', 'sx', 'sy', 'sz']
            attribute.channelControl(nodeList=[loc], attrList=attrList)
        self.addGuideLoc(self.getName('footRoll'), mtx)
        mtx = transform.getMatrixFromPos((0, -2, 0))
        self.addGuideLoc(self.getName('floor'), mtx)


def instantiateFromDagNode(dagNode):
    return TElephantFootGuide(dagNode.guide_name.get(),
                              dagNode.guide_side.get(),
                              dagNode.guide_index.get(),
                              num_ctrls=dagNode.num_ctrls.get(),
                              add_joint=dagNode.add_joint.get(),
                              fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TElephantFootGuide(**kwargs)
