from tRigger import components
from tRigger.core import attribute, dag
reload(components)

import pymel.core as pm

class TSpine01(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'spine01')
        print 'Created Spine01 Component: %s' % self.comp_name

    def addObjects(self, guide):
        # FK controls - base, lower upper
        xform = pm.xform(pm.PyNode(guide.root), q=1, ws=1, m=1)
        self.base_ctrl = self.addCtrl(shape='squarePoint', size=20.0,
                                      name=self.getName('fk_base'), xform=xform, parent=self.base_srt, buffer=1)
        xform = pm.xform(pm.PyNode(guide.divisionLocs[guide.num_divisions-1]), q=1, ws=1, m=1)
        self.lower_ctrl = self.addCtrl(shape='squarePoint', size=15.0,
                                       name=self.getName('fk_base'), xform=xform, parent=self.base_ctrl, buffer=1)
        xform = pm.xform(pm.PyNode(guide.divisionLocs[len(guide.divisionLocs)-1]), q=1, ws=1, m=1)
        self.upper_ctrl = self.addCtrl(shape='squarePoint', size=15.0,
                                       name=self.getName('fk_base'), xform=xform, parent=self.lower_ctrl, buffer=1)

        self.divs = []
        for i, div in enumerate(guide.divisionLocs):
            num = str(i+1).zfill(2)
            mtx = pm.xform(div, q=1, ws=1, m=1)
            parent = self.rig
            if i > 0:
                parent = self.divs[i-1]
            node = dag.addChild(parent, 'group', name=self.getName('div_%s_driven_srt' % num))
            pm.xform(node, ws=1, m=mtx)
            buffer = dag.addParent(node, 'group', name=self.getName('div_%s_buffer_srt' % num))
            self.mapToGuideLocs(node, div)
            self.divs.append(node)


    def addSystems(self):
        pass



def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TSpine01(guide)
