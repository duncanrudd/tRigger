import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute
reload(guide)

axisDict = {'x': pm.datatypes.Vector(10, 0, 0),
            'y': pm.datatypes.Vector(0, 10, 0),
            'z': pm.datatypes.Vector(0, 0, 10),
            }

class TFkChainGuide(guide.TGuideBaseComponent):
    def __init__(self, name, side='C', index=0, segments=4, axis='x', upAxis='y', fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, name, 'fkChain', side, index, fromDagNode=fromDagNode)
        self.axis = axis
        self.up_axis = upAxis
        self.num_segments = segments
        for param in ['num_segments', 'axis', 'up_axis']:
            self.params.append(param)
        if not fromDagNode:
            attribute.addIntAttr(self.root, 'num_segments', segments)
            attribute.addStringAttr(self.root, 'axis', axis)
            attribute.addStringAttr(self.root, 'up_axis', upAxis)
            self.addLocs(segments)
        else:
            self.locs = self.getGuideLocs(fromDagNode)

    def addLocs(self, segments):
        toDelete = [node for node in self.locs if not node == self.root]
        try:
            toDelete.append(self.crv)
            toDelete.append(self.upNode)
        except:
            pass
        pm.delete(toDelete)
        self.locs = [self.root]
        self.num_segments = segments
        self.root.num_segments.set(segments)
        for i in range(segments):
            num = str(i+1).zfill(2)
            mtx = transform.getMatrixFromPos(axisDict[self.axis])
            self.addGuideLoc(self.getName(num), mtx, self.locs[-1])
        self.crv = self.addGuideCurve(self.locs, name=self.guide_name + '_crv', degree=1)
        self.upNode = self.addGuideUpNode(self.up_axis)

def instantiateFromDagNode(dagNode):
    return TFkChainGuide(dagNode.guide_name.get(),
                         dagNode.guide_side.get(),
                         dagNode.guide_index.get(),
                         dagNode.num_segments.get(),
                         dagNode.axis.get(),
                         dagNode.up_axis.get(),
                         fromDagNode=dagNode)
