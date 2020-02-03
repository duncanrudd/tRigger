import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute, curve, mathOps
from maya.api import OpenMaya as om2
reload(guide)
reload(mathOps)

class TMouth01Guide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_divisions=4, num_ctrls=1, corner_start=.25,
                 fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'mouth01', guide_side, guide_index,
                                           fromDagNode=fromDagNode)
        self.num_divisions = num_divisions
        self.num_ctrls = num_ctrls
        self.corner_start = corner_start
        self.divisionLocs = []
        self.ctrlLocs = []
        for param in ['num_divisions', 'num_ctrls', 'corner_start', 'jaw']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            attribute.addIntAttr(self.root, 'num_ctrls', value=num_ctrls, minValue=1)
            attribute.addFloatAttr(self.root, 'corner_start', value=corner_start, minValue=0, maxValue=0.49)
            attribute.addStringAttr(self.root, 'jaw')
            self.addLocs()
            attribute.addBoolAttr(self.root, 'add_joint')
            self.cornerStartRev = mathOps.reverse(self.root.corner_start, name=self.getName('cornerStart_reverse'))
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.divisionLocs = self.getGuideLocs(fromDagNode, locType='div')
            self.ctrlLocs = self.getGuideLocs(fromDagNode, locType='ctrl')
            self.crv = pm.PyNode(self.getName('crv'))
            self.upperCrv = pm.PyNode(self.getName('upper_ctrlCrv'))
            self.lowerCrv = pm.PyNode(self.getName('lower_ctrlCrv'))
            self.cornerStartRev = pm.PyNode(self.getName('cornerStart_reverse'))
            self.upperRebuildCrv = pm.PyNode(self.getName('upperRebuildCrv'))
            self.lowerRebuildCrv = pm.PyNode(self.getName('lowerRebuildCrv'))
        self.installComponentCallbacks()
        if not fromDagNode:
            self.addCtrls()
            self.addDivisions()

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
        
        self.lowerRebuildCrv = pm.createNode('rebuildCurve', name=self.getName('lowerRebuildCrv'))
        self.lowerRebuildCrv.degree.set(3)
        self.lowerRebuildCrv.keepControlPoints.set(1)
        self.crv.worldSpace[0].connect(self.lowerRebuildCrv.inputCurve)
         

        self.locs = self.getGuideLocs(self.root)
        # self.crv.visibility.set(0)
        self.addSpaceSwitchAttr(self.locs[3])

    def installComponentCallbacks(self):
        try:
            attribute.removeAttributeCallback(self.root, self.num_divisions_CB)
            attribute.removeAttributeCallback(self.root, self.num_ctrls_CB)
            self.num_divisions_CB = None
            self.num_ctrls_CB = None
        except:
            pass
        self.num_divisions_CB = attribute.addCallbackToAttr(self.root, 'num_divisions', self.num_divisions_callback)
        self.num_ctrls_CB = attribute.addCallbackToAttr(self.root, 'num_ctrls', self.num_ctrls_callback)

    def num_ctrls_callback(self, msg, plug1, plug2, payload):
        if msg == 2056:
            mfn_dep = om2.MFnDependencyNode(plug1.node())
            if mfn_dep.findPlug('num_ctrls', False) == plug1:
                self.num_ctrls = self.root.num_ctrls.get()
                sel = pm.selected()
                self.addCtrls()
                pm.select(sel)
                print('callback fired')

    # FK CTRL LOCS
    def addCtrls(self):
        hrcDict = {}
        for node in self.ctrlLocs:
            children = pm.listRelatives(node, children=1, type='transform')
            hrcDict[node.name()] = children
            for child in children:
                child.setParent(self.root)
        try:
            pm.lockNode(self.cornerStartRev, lock=1)
            self.crv.worldSpace[0].connect(self.upperRebuildCrv.inputCurve, f=1)
            self.crv.worldSpace[0].connect(self.lowerRebuildCrv.inputCurve, f=1)
            pm.delete([self.upperCrv, self.lowerCrv, self.circleCrv])
        except:
            pass
        pm.delete(self.ctrlLocs)
        self.ctrlLocs = []
        totalCtrls = 4 + (4*self.num_ctrls)
        
        self.upperMPs = []
        self.lowerMPs = []
        self.upperLocs = []
        self.lowerLocs = []
        
        # upper and corners
        for i in range((totalCtrls/2)+1):
            num = str(i+1).zfill(2)
            param = (1.0 / (totalCtrls/2)) * i
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
        for i in range((totalCtrls/2)-1):
            num = str(i+1).zfill(2)
            param = (1.0 / (totalCtrls/2)) * (i+1)
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

        for key in hrcDict:
            try:
                for child in hrcDict[key]:
                    child.setParent(key)
            except:
                print 'Unable to reparent: %s to %s' % (child.name(), key)

        ctrls = [self.upperLocs[0]] + self.upperLocs + [self.upperLocs[-1]]
        self.upperCrv = self.addGuideCurve(ctrls, name='upper_ctrlCrv', degree=2)
        self.upperCrv.worldSpace[0].connect(self.upperRebuildCrv.inputCurve, f=1)

        ctrls = [self.upperLocs[0] for i in range(2)] + self.lowerLocs + [self.upperLocs[-1] for i in range(2)]
        self.lowerCrv = self.addGuideCurve(ctrls, name='lower_ctrlCrv', degree=2)
        self.lowerCrv.worldSpace[0].connect(self.lowerRebuildCrv.inputCurve, f=1)

        points = [(i, 0, 0) for i in range(totalCtrls)]
        points.append(points[0])
        points.append(points[1])
        knots = range(totalCtrls+3)
        self.circleCrv = pm.curve(per=True, p=points, k=knots, d=2, name=self.getName('circleCrv'))
        ctrls = self.upperLocs + [self.lowerLocs[len(self.lowerLocs)-(i+1)] for i in range(len(self.lowerLocs))]
        for index, ctrl in enumerate(ctrls):
            d = mathOps.decomposeMatrix(ctrl.worldMatrix[0])
            d.outputTranslate.connect(self.circleCrv.controlPoints[index])
        self.circleCrv.setParent(self.root)
        self.circleCrv.inheritsTransform.set(0)
        pm.lockNode(self.cornerStartRev, lock=0)

    def num_divisions_callback(self, msg, plug1, plug2, payload):
        if msg == 2056:
            mfn_dep = om2.MFnDependencyNode(plug1.node())
            if mfn_dep.findPlug('num_divisions', False) == plug1:
                self.num_divisions = self.root.num_divisions.get()
                sel = pm.selected()
                self.addDivisions()
                pm.select(sel)
                print('callback fired')

    # JOINT LOCS
    def addDivisions(self):
        hrcDict = {}
        for node in self.divisionLocs:
            children = pm.listRelatives(node, children=1, type='transform')
            hrcDict[node.name()] = children
            for child in children:
                child.setParent(self.root)
        pm.lockNode(self.upperRebuildCrv, lock=1)
        pm.delete([node for node in self.divisionLocs])
        self.divisionLocs = []
        for i in range(self.num_divisions):
            num = str(i+1).zfill(2)
            param = (1.0 / (self.num_divisions-1)) * i
            mp = curve.createMotionPathNode(self.crv, uValue=param, name='%s_%s_mp' % (self.guide_name, num))
            self.upperRebuildCrv.outputCurve.connect(mp.geometryPath, f=1)
            mtx = transform.getMatrixFromPos(mp.allCoordinates.get())
            loc = self.addGuideLoc(self.getName('div_%s' % num), mtx, self.root, colour='blue', size=0.5, locType='div')
            loc.inheritsTransform.set(0)
            mp.allCoordinates.connect(loc.t)
            self.divisionLocs.append(loc)
        pm.lockNode(self.upperRebuildCrv, lock=0)
        for key in hrcDict:
            try:
                for child in hrcDict[key]:
                    child.setParent(key)
            except:
                print 'Unable to reparent: %s to %s' % (child.name(), key)

def instantiateFromDagNode(dagNode):
    return TMouth01Guide(dagNode.guide_name.get(),
                           dagNode.guide_side.get(),
                           dagNode.guide_index.get(),
                           dagNode.num_divisions.get(),
                           dagNode.num_ctrls.get(),
                           dagNode.corner_start.get(),
                           fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TMouth01Guide(**kwargs)

