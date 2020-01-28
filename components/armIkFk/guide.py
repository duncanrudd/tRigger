import pymel.core as pm

from tRigger.components import guide, attribute, transform
reload(guide)

class TArmIkFkGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_divisions=5, add_joint=1,
                 sleeve=0, world_aligned_ik_ctrl=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'armIkFk', guide_side, guide_index, fromDagNode=fromDagNode)
        for param in ['num_divisions', 'add_joint', 'world_aligned_ik_ctrl', 'sleeve']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
            attribute.addBoolAttr(self.root, 'world_aligned_ik_ctrl', world_aligned_ik_ctrl)
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            attribute.addEnumAttr(self.root, 'sleeve', ['none', 'upper', 'full', 'lower', 'split'])
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
        self.num_divisions = self.root.num_divisions.get()
        self.add_joint = self.root.add_joint.get()
        self.world_aligned_ik_ctrl = self.root.world_aligned_ik_ctrl.get()
        self.sleeve = self.root.sleeve.get()

    def addLocs(self):
        points = [(x, 0, 0) for x in [0, 0, 20, 25]]
        points.append((10, 0, -10))

        for index, point in enumerate(points):
            num = str(index+1).zfill(2)
            xform = transform.getMatrixFromPos(point)
            self.addGuideLoc(self.getName(num), xform, self.root)
        self.locs[2].setParent(self.locs[1])
        self.locs[3].setParent(self.locs[1])
        self.locs[4].setParent(self.locs[3])
        self.locs[5].setParent(self.locs[1])

        self.addGuideCurve([self.locs[x] for x in [1, 2, 3]], name='crv')
        self.addGuideCurve([self.locs[x] for x in [2, 5]], name='pole_crv')

        aimMtx = pm.createNode('aimMatrix', name=self.getName('aim_mtx'))
        aimMtx.secondaryInputAxis.set((0, 0, -1))
        aimMtx.secondaryMode.set(1)
        self.locs[1].worldMatrix[0].connect(aimMtx.inputMatrix)
        self.locs[3].worldMatrix[0].connect(aimMtx.primary.primaryTargetMatrix)
        self.locs[5].worldMatrix[0].connect(aimMtx.secondary.secondaryTargetMatrix)

        blendMtx = pm.createNode('blendMatrix', name=self.getName('blend_matrix'))
        aimMtx.outputMatrix.connect(blendMtx.inputMatrix)
        self.locs[3].worldMatrix[0].connect(blendMtx.target[0].targetMatrix)
        blendMtx.target[0].useRotate.set(0)
        blendMtx.target[0].weight.set(0.5)

        self.locs[2].inheritsTransform.set(0)

        blendMtx.outputMatrix.connect(self.locs[2].offsetParentMatrix)

        for node in [self.locs[1], self.locs[3], self.locs[5]]:
            self.addSpaceSwitchAttr(node)

        self.locs = self.getGuideLocs(self.root)

def instantiateFromDagNode(dagNode):
    return TArmIkFkGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_divisions.get(),
                         add_joint=dagNode.add_joint.get(),
                         world_aligned_ik_ctrl=dagNode.world_aligned_ik_ctrl.get(),
                         sleeve=dagNode.sleeve.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TArmIkFkGuide(**kwargs)
