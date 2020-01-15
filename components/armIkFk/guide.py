import pymel.core as pm

from tRigger.components import guide, attribute, transform
reload(guide)

class TArmIkFkGuide(guide.TGuideBaseComponent):
    def __init__(self, name, side='C', index=0, divisions=5, addJoint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, name, 'armIkFk', side, index, fromDagNode=fromDagNode)
        self.num_divisions = divisions
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint')
            attribute.addBoolAttr(self.root, 'world_aligned_ik_ctrl')
            attribute.addIntAttr(self.root, 'num_divisions', divisions)
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)

    def addLocs(self):
        points = [(x, 0, 0) for x in [0, 0, 20, 25]]
        points.append((10, 0, -10))

        for index, point in enumerate(points):
            num = str(index+1).zfill(2)
            xform = transform.getMatrixFromPos(point) * self.root.worldMatrix[0].get()
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

def instantiateFromDagNode(dagNode):
    return TArmIkFkGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_divisions.get(),
                         addJoint=dagNode.add_joint.get(),
                         fromDagNode=dagNode)
