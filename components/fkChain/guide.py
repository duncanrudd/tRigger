import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform
reload(guide)

class TFkChainGuide(guide.TGuideBaseComponent):
    def __init__(self, name, side='C', index=0, segments=4, axis='x', upAxis='y'):
        guide.TGuideBaseComponent.__init__(self, name, 'fkChain', side, index)

        axisDict = {'x': pm.datatypes.Vector(10, 0, 0),
                    'y': pm.datatypes.Vector(0, 10, 0),
                    'z': pm.datatypes.Vector(0, 0, 10),
                    }

        for i in range(segments+1):
            num = str(i+1).zfill(2)
            mtx = transform.getMatrixFromPos(axisDict[axis])
            self.addGuideLoc(self.getName(num), mtx, self.locs[-1])
