import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute, curve, mathOps
from maya.api import OpenMaya as om2
reload(guide)
reload(mathOps)

axisDict = {'x': pm.datatypes.Vector(10, 0, 0),
            'y': pm.datatypes.Vector(0, 10, 0),
            'z': pm.datatypes.Vector(0, 0, 10),
            }

class TNeck02Guide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_divisions=4, num_ctrls=3, aimer=1, ik_start_ctrl=0, sCurve=0, side_rail=0,
                 fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'neck02', guide_side, guide_index, fromDagNode=fromDagNode)
        self.num_divisions = num_divisions
        self.num_ctrls = num_ctrls
        self.divisionLocs = []
        self.ctrlLocs = []
        self.aimer=aimer
        self.ik_start_ctrl = ik_start_ctrl
        self.sCurve = sCurve
        self.side_rail = side_rail
        for param in ['num_divisions', 'num_ctrls', 'aimer', 'ik_start_ctrl', 'sCurve', 'side_rail']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            attribute.addIntAttr(self.root, 'num_ctrls', minValue=3, value=num_ctrls)
            attribute.addBoolAttr(self.root, 'aimer', aimer)
            attribute.addBoolAttr(self.root, 'ik_start_ctrl', ik_start_ctrl)
            attribute.addBoolAttr(self.root, 'sCurve', sCurve)
            attribute.addBoolAttr(self.root, 'side_rail', sCurve)
            self.addLocs()
            attribute.addBoolAttr(self.root, 'add_joint')
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.divisionLocs = self.getGuideLocs(fromDagNode, locType='div')
            self.ctrlLocs = self.getGuideLocs(fromDagNode, locType='ctrl')
            self.crv = pm.PyNode(self.getName('crv'))
            self.ctrlCrv = pm.PyNode(self.getName('ctrlCrv'))
            self.rebuildCrv = pm.PyNode(self.getName('rebuildCrv'))
        self.installComponentCallbacks()
        if not fromDagNode:
            self.addCtrls()
            self.addDivisions()

    def addLocs(self):
        for i in range(3):
            xform = transform.getMatrixFromPos((0, 5*(i+1), 0))
            num = str(i+1).zfill(2)
            self.addGuideLoc(self.getName(num), xform, self.root)

        self.crv = self.addGuideCurve(self.locs, name='crv', degree=2)
        self.rebuildCrv = pm.createNode('rebuildCurve', name=self.getName('rebuildCrv'))
        self.rebuildCrv.degree.set(2)
        self.rebuildCrv.keepControlPoints.set(1)
        self.crv.worldSpace[0].connect(self.rebuildCrv.inputCurve)
        # Aim loc
        xform = transform.getMatrixFromPos((0, 15, 10))
        self.addGuideLoc(self.getName('aim'), xform, self.root)
        self.aimCrv = self.addGuideCurve([self.locs[-1], self.locs[-2]], name='aim_crv', degree=1)
        # Tip loc
        xform = transform.getMatrixFromPos((0, 20, 0))
        self.addGuideLoc(self.getName('tip'), xform, self.root)

        self.locs = self.getGuideLocs(self.root)
        self.addSpaceSwitchAttr(self.locs[4])
        self.crv.visibility.set(0)
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
            self.crv.worldSpace[0].connect(self.rebuildCrv.inputCurve, f=1)
            pm.delete(self.ctrlCrv)
        except:
            pass
        pm.delete(self.ctrlLocs)
        self.ctrlLocs = []

        for i in range(self.num_ctrls):
            num = str(i+1).zfill(2)
            param = (1.0 / (self.num_ctrls-1)) * i
            mp = curve.createMotionPathNode(self.crv, uValue=param, name='%s_%s_ctrl_mp' % (self.guide_name, num))
            mtx = transform.getMatrixFromPos(mp.allCoordinates.get())
            loc = self.addGuideLoc(self.getName('ctrl_%s' % num), mtx, self.root, colour='green', size=3, locType='ctrl')
            loc.inheritsTransform.set(0)
            mp.allCoordinates.connect(loc.t)
            self.ctrlLocs.append(loc)
        for key in hrcDict:
            try:
                for child in hrcDict[key]:
                    child.setParent(key)
            except:
                print 'Unable to reparent: %s to %s' % (child.name(), key)
        self.ctrlCrv = self.addGuideCurve(self.ctrlLocs, name='ctrlCrv', degree=2)
        self.ctrlCrv.worldSpace[0].connect(self.rebuildCrv.inputCurve, f=1)

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
        pm.lockNode(self.rebuildCrv, lock=1)
        pm.delete([node for node in self.divisionLocs])
        self.divisionLocs = []
        for i in range(self.num_divisions):
            num = str(i+1).zfill(2)
            param = (1.0 / (self.num_divisions-1)) * i
            mp = curve.createMotionPathNode(self.crv, uValue=param, name='%s_%s_mp' % (self.guide_name, num))
            self.rebuildCrv.outputCurve.connect(mp.geometryPath, f=1)
            mtx = transform.getMatrixFromPos(mp.allCoordinates.get())
            loc = self.addGuideLoc(self.getName('div_%s' % num), mtx, self.root, colour='blue', size=2, locType='div')
            loc.inheritsTransform.set(0)
            mp.allCoordinates.connect(loc.t)
            self.divisionLocs.append(loc)
        pm.lockNode(self.rebuildCrv, lock=0)
        for key in hrcDict:
            try:
                for child in hrcDict[key]:
                    child.setParent(key)
            except:
                print 'Unable to reparent: %s to %s' % (child.name(), key)

def instantiateFromDagNode(dagNode):
    return TNeck02Guide(dagNode.guide_name.get(),
                        dagNode.guide_side.get(),
                        dagNode.guide_index.get(),
                        dagNode.num_divisions.get(),
                        dagNode.num_ctrls.get(),
                        dagNode.aimer.get(),
                        dagNode.ik_start_ctrl.get(),
                        dagNode.sCurve.get(),
                        dagNode.side_rail.get(),
                        fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TNeck02Guide(**kwargs)

def updateGuide(guideRoot):
    if not guideRoot.hasAttr('ik_start_ctrl'):
        attribute.addBoolAttr(guideRoot, 'ik_start_ctrl', 0)
    if not guideRoot.hasAttr('sCurve'):
        attribute.addBoolAttr(guideRoot, 'sCurve', 0)
    if not guideRoot.hasAttr('side_rail'):
        attribute.addBoolAttr(guideRoot, 'side_rail', 0)

