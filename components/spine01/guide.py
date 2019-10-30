import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute, curve, mathOps
reload(guide)

axisDict = {'x': pm.datatypes.Vector(10, 0, 0),
            'y': pm.datatypes.Vector(0, 10, 0),
            'z': pm.datatypes.Vector(0, 0, 10),
            }

class TSpine01Guide(guide.TGuideBaseComponent):
    def __init__(self, name, side='C', index=0, segments=2, divisions=4, axis='y', upAxis='z', fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, name, 'spine01', side, index, fromDagNode=fromDagNode)
        self.axis = axis
        self.up_axis = upAxis
        self.num_segments = segments
        self.num_divisions = divisions
        self.mid_point = 0.5
        self.segmentLocs = []
        for param in ['num_segments', 'num_divisions', 'axis', 'up_axis', 'mid_point']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_segments', segments)
            attribute.addIntAttr(self.root, 'num_divisions', divisions)
            midAttr = attribute.addFloatAttr(self.root, 'mid_point', 0.5, minValue=0.0, maxValue=1.0)
            attribute.addStringAttr(self.root, 'axis', axis)
            attribute.addStringAttr(self.root, 'up_axis', upAxis)
            self.addLocs()
            self.addSegments()
            attribute.addBoolAttr(self.root, 'add_joint')
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.segmentLocs = self.getGuideLocs(fromDagNode, locType='div')
            self.crv = pm.PyNode(self.getName('crv'))

    def addLocs(self):
        toDelete = [node for node in self.locs if not node == self.root]
        try:
            toDelete.append(self.crv)
            toDelete.append(self.upNode)
        except:
            pass
        pm.delete(toDelete)
        self.locs = [self.root]
        for i in range(3):
            num = str(i+1).zfill(2)
            mtx = transform.getMatrixFromPos(axisDict[self.axis])
            self.addGuideLoc(self.getName(num), mtx, self.locs[-1])
        self.crv = self.addGuideCurve(self.locs, name=self.getName('crv'), degree=3)
        self.upNode = self.addGuideUpNode(self.up_axis)

    # JOINT LOCS
    def addSegments(self):
        pm.delete([node for node in self.segmentLocs])
        self.segmentLocs = []
        totalDivs = (self.num_divisions*self.num_segments)-(self.num_segments-1)
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
            self.segmentLocs.append(loc)

def instantiateFromDagNode(dagNode):
    return TSpine01Guide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_segments.get(),
                         dagNode.num_divisions.get(),
                         dagNode.axis.get(),
                         dagNode.up_axis.get(),
                         fromDagNode=dagNode)
