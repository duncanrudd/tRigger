from tRigger import components
from tRigger.core import attribute, transform, dag, mathOps
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)

import pymel.core as pm

class TShoulderFK(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'control')
        print 'Created ShoulderFK Component: %s' % self.comp_name

    def addObjects(self, guide):

        # Build initial matrix
        start, end = mathOps.getStartAndEnd(guide.root, guide.locs[1])
        aimVec = (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
        start, end = mathOps.getStartAndEnd(guide.root, guide.locs[3])
        upVecTemp = (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
        sideVec = aimVec.cross(upVecTemp).normal()
        upVec = sideVec.cross(aimVec).normal()
        startPos = pm.xform(guide.root, q=1, ws=1, t=1)

        xform = pm.datatypes.Matrix(aimVec, upVec, sideVec, startPos)
        self.base_srt.offsetParentMatrix.set(xform)
        self.addCtrl(shape='circlePoint', size=20.0,
                     name=self.getName('fk'), xform=xform, parent=self.base_srt)
        self.mapToGuideLocs(self.controls_list[0], guide.locs[0])

        xform = guide.locs[2].worldMatrix[0].get()
        self.addCtrl(shape='circlePoint', size=10.0,
                     name=self.getName('orbit'), xform=xform, parent=self.controls_list[0])
        self.mapToGuideLocs(self.controls_list[0], guide.locs[0])
        self.mapToGuideLocs(self.controls_list[1], guide.locs[2])

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('jnt'))
            self.joints_list.append({'joint': j, 'driver': self.controls_list[0]})
            self.mapJointToGuideLocs(j, guide.locs[0])
        components.TBaseComponent.addObjects(self, guide)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TShoulderFK(guide)

