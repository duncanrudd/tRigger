import pymel.core as pm
import math

from tRigger.components import guide
from tRigger.core import transform, attribute, curve, mathOps
from maya.api import OpenMaya as om2
reload(guide)
reload(mathOps)

class TMouth04Guide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_divisions=9, num_ctrls=1, corner_start=.25,
                 local_rig=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'mouth04', guide_side, guide_index,
                                           fromDagNode=fromDagNode)
        self.num_divisions = num_divisions
        self.num_ctrls = num_ctrls
        self.corner_start = corner_start
        self.local_rig = local_rig
        self.divisionLocs = []
        self.ctrlLocs = []
        for param in ['num_divisions', 'num_ctrls', 'corner_start', 'local_rig']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            attribute.addIntAttr(self.root, 'num_ctrls', value=num_ctrls, minValue=1)
            attribute.addFloatAttr(self.root, 'corner_start', value=corner_start, minValue=0, maxValue=0.49)
            attribute.addBoolAttr(self.root, 'local_rig', local_rig)
            self.addLocs()
            attribute.addBoolAttr(self.root, 'add_joint')
            self.cornerStartRev = mathOps.reverse(self.root.corner_start, name=self.getName('cornerStart_reverse'))
            self.addCtrls()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.crv = pm.PyNode(self.getName('crv'))
            self.upperCrv = pm.PyNode(self.getName('upper_ctrlCrv'))
            self.lowerCrv = pm.PyNode(self.getName('lower_ctrlCrv'))
            self.cornerStartRev = pm.PyNode(self.getName('cornerStart_reverse'))
            self.jawLoc = pm.PyNode(self.getName('jaw'))
            self.root.local_rig.set(self.local_rig)

    def addLocs(self):
        for i in range(7):
            xform = transform.getMatrixFromPos(((i*2)-6, 0, 0))
            num = str(i+1).zfill(2)
            self.addGuideLoc(self.getName(num), xform, self.root, size=2)

        self.crv = self.addGuideCurve(self.locs[1:], name='crv', degree=3)
        self.upperRebuildCrv = pm.createNode('rebuildCurve', name=self.getName('upperRebuildCrv'))
        self.upperRebuildCrv.degree.set(3)
        self.upperRebuildCrv.keepControlPoints.set(1)
        self.crv.worldSpace[0].connect(self.upperRebuildCrv.inputCurve)

        self.jawLoc = self.addGuideLoc(self.getName('jaw'), self.root.worldMatrix[0].get(), self.root, size=2)

        self.locs = self.getGuideLocs(self.root)

        self.addSpaceSwitchAttr(self.locs[3])
        self.addSpaceSwitchAttr(self.jawLoc)

    def addCtrls(self):
        totalCtrls = 4 + (4 * self.num_ctrls)

        self.upperMPs = []
        self.lowerMPs = []
        self.upperLocs = []
        self.lowerLocs = []

        # upper and corners
        for i in range((totalCtrls / 2) + 1):
            num = str(i + 1).zfill(2)
            param = (1.0 / (totalCtrls / 2)) * i
            if param <= 0.5:
                param = 0.5 * (1.0 - (math.cos((math.pi / 2.0) * (param * 2))))
            else:
                param = (0.5 * (math.sin((math.pi / 2.0) * ((param - 0.5) * 2)))) + 0.5
            paramRemap = mathOps.remap(param, 0, 1, self.root.corner_start, self.cornerStartRev.outputX,
                                       name=self.getName('upper_%s_param' % num))
            mp = curve.createMotionPathNode(self.crv, uValue=paramRemap.outValueX, wut=2, wuo=self.root,
                                            name='%s_upper_%s_ctrl_mp' % (self.guide_name, num))
            mtx = pm.datatypes.Matrix()
            self.upperMPs.append(mp)
            loc = self.addGuideLoc(self.getName('upper_ctrl_%s' % num), mtx, self.root,
                                   colour='green', size=1, locType='upperCtrl')
            loc.inheritsTransform.set(0)
            mtx = mathOps.createComposeMatrix(inputTranslate=mp.allCoordinates, inputRotate=mp.rotate,
                                              name=self.getName('upper_ctrlMtx_%s' % num))
            mtx.outputMatrix.connect(loc.offsetParentMatrix)
            self.upperLocs.append(loc)
            self.ctrlLocs.append(loc)

        # lower
        for i in range((totalCtrls / 2) - 1):
            num = str(i + 1).zfill(2)
            param = (1.0 / (totalCtrls / 2)) * (i + 1)
            if param <= 0.5:
                param = 0.5 * (1.0 - (math.cos((math.pi / 2.0) * (param * 2))))
            else:
                param = (0.5 * (math.sin((math.pi / 2.0) * ((param - 0.5) * 2)))) + 0.5
            paramRemap = mathOps.remap(param, 0, 1, self.root.corner_start, self.cornerStartRev.outputX,
                                       name=self.getName('lower_%s_param' % num))
            mp = curve.createMotionPathNode(self.crv, uValue=paramRemap.outValueX, wut=2, wuo=self.root,
                                            name='%s_lower_%s_ctrl_mp' % (self.guide_name, num))
            mtx = pm.datatypes.Matrix()
            self.lowerMPs.append(mp)
            loc = self.addGuideLoc(self.getName('lower_ctrl_%s' % num), mtx, self.root,
                                   colour='green', size=1, locType='lowerCtrl')
            loc.inheritsTransform.set(0)
            mtx = mathOps.createComposeMatrix(inputTranslate=mp.allCoordinates, inputRotate=mp.rotate,
                                              name=self.getName('lower_ctrlMtx_%s' % num))
            mtx.outputMatrix.connect(loc.offsetParentMatrix)
            self.lowerLocs.append(loc)
            self.ctrlLocs.append(loc)

        ctrls = [self.upperLocs[0]] + self.upperLocs + [self.upperLocs[-1]]
        self.upperCrv = self.addGuideCurve(ctrls, name='upper_ctrlCrv', degree=2)

        ctrls = [self.upperLocs[0] for i in range(2)] + self.lowerLocs + [self.upperLocs[-1] for i in range(2)]
        self.lowerCrv = self.addGuideCurve(ctrls, name='lower_ctrlCrv', degree=2)

def instantiateFromDagNode(dagNode):
    return TMouth04Guide(dagNode.guide_name.get(),
                           dagNode.guide_side.get(),
                           dagNode.guide_index.get(),
                           dagNode.num_divisions.get(),
                           dagNode.num_ctrls.get(),
                           dagNode.corner_start.get(),
                           local_rig=dagNode.local_rig.get(),
                           fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TMouth04Guide(**kwargs)

