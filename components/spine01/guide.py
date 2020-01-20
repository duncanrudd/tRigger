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

class TSpine01Guide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, num_divisions=4, axis='y', up_axis='z',
                 mid_point=0.5, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'spine01', guide_side, guide_index, fromDagNode=fromDagNode)
        self.axis = axis
        self.up_axis = up_axis
        self.num_divisions = num_divisions
        self.mid_point = mid_point
        self.divisionLocs = []
        for param in ['num_divisions', 'axis', 'up_axis', 'mid_point']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_divisions', num_divisions)
            midAttr = attribute.addFloatAttr(self.root, 'mid_point', mid_point, minValue=0.0, maxValue=1.0)
            attribute.addStringAttr(self.root, 'axis', axis)
            attribute.addStringAttr(self.root, 'up_axis', up_axis)
            self.addLocs()
            attribute.addBoolAttr(self.root, 'add_joint')
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.divisionLocs = self.getGuideLocs(fromDagNode, locType='div')
            self.crv = pm.PyNode(self.getName('crv'))
        self.installComponentCallbacks()
        self.addDivisions()

    def addLocs(self):
        toDelete = [node for node in self.locs if not node == self.root]
        try:
            toDelete.append(self.crv)
            toDelete.append(self.upNode)
        except:
            pass
        pm.delete(toDelete)
        self.locs = [self.root]
        for i in range(4):
            num = str(i+1).zfill(2)
            mtx = transform.getMatrixFromPos(axisDict[self.axis])
            self.addGuideLoc(self.getName(num), mtx, self.locs[-1])
        self.crv = self.addGuideCurve(self.locs, name='crv', degree=3)
        self.upNode = self.addGuideUpNode(self.up_axis)
        self.locs[4].setParent(self.root)
        self.locs[3].setParent(self.locs[4])
        self.locs[2].setParent(self.root)
        lowerTangentScaled = mathOps.multiplyVector(self.locs[1].t, (1.5, 1.5, 1.5),
                                                   name=self.getName('lower_tangent_scaled'))
        lowerTangentPoint = mathOps.createTransformedPoint(lowerTangentScaled.output, self.root.worldMatrix[0],
                                                           name=self.getName('lower_tangent_point'))
        upperTangentScaled = mathOps.multiplyVector(self.locs[3].t, (1.5, 1.5, 1.5),
                                                    name=self.getName('upper_tangent_scaled'))
        upperTangentPoint = mathOps.createTransformedPoint(upperTangentScaled.output, self.locs[4].worldMatrix[0],
                                                           name=self.getName('upper_tangent_point'))
        midBlendPoint = mathOps.pairBlend(translateA=lowerTangentPoint.output, translateB=upperTangentPoint.output,
                                          name=self.getName('mid_blend_point'))
        midPoint = mathOps.createTransformedPoint(midBlendPoint.outTranslate, self.root.worldInverseMatrix[0],
                                                  name=self.getName('mid_point'))
        midPoint.output.connect(self.locs[2].t)

        self.locs = self.getGuideLocs(self.root)


    def installComponentCallbacks(self):
        try:
            attribute.removeAttributeCallback(self.root, self.num_divisions_CB)
            self.num_divisions_CB = None
        except:
            pass
        self.num_divisions_CB = attribute.addCallbackToAttr(self.root, 'num_divisions', self.num_divisions_callback)

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
        pm.delete([node for node in self.divisionLocs])
        self.divisionLocs = []
        totalDivs = (self.num_divisions*2)-1
        for i in range(totalDivs):
            num = str(i+1).zfill(2)
            param = (1.0 / (self.num_divisions-1)) * i
            if i >= self.num_divisions:
                param -= 1
                paramNode = mathOps.remap(param, 0, 1, self.root.mid_point, 1, name=self.getName('param_%s' % num))
            else:
                paramNode = mathOps.remap(param, 0, 1, 0, self.root.mid_point, name=self.getName('param_%s' % num))
            mp = curve.createMotionPathNode(self.crv, uValue=paramNode.outValueX, name='%s_%s_mp' % (self.guide_name, num))
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
                print 'Unable to reparent: %s to %s' % (child.name(), key)

def instantiateFromDagNode(dagNode):
    return TSpine01Guide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_divisions.get(),
                         dagNode.axis.get(),
                         dagNode.up_axis.get(),
                         fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TSpine01Guide(**kwargs)

