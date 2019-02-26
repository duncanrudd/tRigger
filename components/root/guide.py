import pymel.core as pm

from tRigger.components import guide
reload(guide)

class TRootGuide(guide.TGuideBaseComponent):
    def __init__(self, name, side='C', index=0):
        guide.TGuideBaseComponent.__init__(self, name, 'root', side, index)