import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, curve, attribute
from maya.api import OpenMaya as om2
reload(guide)

class TArcGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_divisions=3, add_joint=1, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'arc', guide_side, guide_index, fromDagNode=fromDagNode)
        self.num_divisions = num_divisions
        self.add_joint = add_joint
        self.divisionLocs = []
        for param in ['num_divisions', 'add_joint']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            attribute.addBoolAttr(self.root, 'add_joint', add_joint)
            self.addLocs()
            self.addDivs()
        else:
            self.root.num_divisions.set(self.num_divisions)
            self.root.add_joint.set(self.add_joint)
            self.locs = self.getGuideLocs(fromDagNode)
            self.divisionLocs = self.getGuideLocs(fromDagNode, locType='div')
            self.crv = pm.PyNode(self.getName('crv'))
        self.installComponentCallbacks()

    def installComponentCallbacks(self):
        try:
            attribute.removeAttributeCallback(self.root, self.num_divisions_CB)
            self.num_divisions_CB = None
        except:
            pass
        self.num_divisions_CB = attribute.addCallbackToAttr(self.root, 'num_divisions_CB', self.num_divisions_callback)
        guide.TGuideBaseComponent.installCallbacks(self)

    def num_divisions_callback(self, msg, plug1, plug2, payload):
        if msg == 2056:
            mfn_dep = om2.MFnDependencyNode(plug1.node())
            plugNames = ['num_divisions']
            interestingPlugs = [mfn_dep.findPlug(plug, False) for plug in plugNames]
            if plug1 in interestingPlugs:
                sel = pm.selected()
                self.num_divisions = self.root.num_divisions.get()
                self.addDivs()
                pm.select(sel)
                print('callback fired')

    def addDivs(self):
        hrcDict = {}
        for node in self.divisionLocs:
            children = pm.listRelatives(node, children=1, type='transform')
            hrcDict[node.name()] = children
            for child in children:
                child.setParent(self.root)
        pm.delete([node for node in self.divisionLocs])
        self.divisionLocs = []
        for i in range(self.num_divisions):
            num = str(i + 1).zfill(2)
            param = (1.0 / (self.num_divisions - 1)) * i
            mp = curve.createMotionPathNode(self.crv, uValue=param, name='%s_%s_mp' % (self.guide_name, num))
            mtx = transform.getMatrixFromPos(mp.allCoordinates.get())
            loc = self.addGuideLoc(self.getName('div_%s' % num), mtx, self.root, colour='blue', size=2, locType='div')
            loc.inheritsTransform.set(0)
            mp.allCoordinates.connect(loc.t)
            self.divisionLocs.append(loc)
        for key in hrcDict:
            try:
                for child in hrcDict[key]:
                    child.setParent(key)
            except:
                print
                'Unable to reparent: %s to %s' % (child.name(), key)

    def addLocs(self):
        mtx = transform.getMatrixFromPos((0, 0, 0))
        startLoc = self.addGuideLoc(self.getName('start'), mtx, self.root)
        self.addSpaceSwitchAttr(startLoc)
        mtx = transform.getMatrixFromPos((6, 0, 0))
        midLoc = self.addGuideLoc(self.getName('mid'), mtx, self.root)
        endLoc = self.addGuideLoc(self.getName('end'), mtx, self.root)
        self.addSpaceSwitchAttr(endLoc)
        midLoc.inheritsTransform.set(0)
        midLoc.t.set((0, 0, 0))
        midMtx = transform.blendMatrices(startLoc.worldMatrix[0], endLoc.worldMatrix[0],
                                         name=self.getName('mid_guide_mtx'))
        midMtx.outputMatrix.connect(midLoc.offsetParentMatrix)
        self.crv = self.addGuideCurve(self.locs[1:], name='crv', degree=2)

def instantiateFromDagNode(dagNode):
    return TArcGuide(dagNode.guide_name.get(),
                     dagNode.guide_side.get(),
                     dagNode.guide_index.get(),
                     num_divisions=dagNode.num_divisions.get(),
                     add_joint=dagNode.add_joint.get(),
                     fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TArcGuide(**kwargs)
