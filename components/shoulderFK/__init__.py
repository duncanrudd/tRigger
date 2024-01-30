from tRigger import components
from tRigger.core import attribute, transform, dag, mathOps, icon
import pymel.core as pm
reload(components)
reload(transform)
reload(mathOps)

import pymel.core as pm

class TShoulderFK(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'shoulderFK')
        print 'Created ShoulderFK Component: %s' % self.comp_name

    def addObjects(self, guide):

        # Build initial matrix
        start, end = mathOps.getStartAndEnd(guide.root, guide.locs[1])
        aimVec = (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
        if self.comp_side == 'R':
            aimVec = (pm.datatypes.Vector(start) - pm.datatypes.Vector(end)).normal()
        start, end = mathOps.getStartAndEnd(guide.root, guide.locs[3])
        upVecTemp = (pm.datatypes.Vector(end) - pm.datatypes.Vector(start)).normal()
        sideVec = aimVec.cross(upVecTemp).normal()
        upVec = sideVec.cross(aimVec).normal()
        startPos = pm.xform(guide.root, q=1, ws=1, t=1)

        # place base srt and invert sx if side == R
        xform = pm.datatypes.Matrix(aimVec, upVec, sideVec, startPos)
        if self.comp_side == 'R':
            xform = mathOps.getInverseHandedMatrix(xform)
        self.base_srt.offsetParentMatrix.set(xform)
        self.base_srt.t.set((0, 0, 0))
        self.base_srt.r.set((0, 0, 0))

        # Generate controls
        ctrlSize = mathOps.getDistance(guide.locs[0], guide.locs[2])*.25
        xform = self.base_srt.worldMatrix[0].get()
        self.fk_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize,
                                    name=self.getName('fk'), xform=xform, parent=self.base_srt,
                                    metaParent=self.base_srt)
        if self.comp_side == 'R':
            xform = mathOps.invertHandedness(xform)
            self.fk_out = dag.addChild(self.rig, 'group', name=self.getName('fk_out_srt'))
            self.fk_out.offsetParentMatrix.set(xform)
            self.mapToGuideLocs(self.fk_out, guide.locs[0])

            # Add mapping connection to control - used when mapping controller tags
            self.copyGuideMapping(self.fk_ctrl, self.fk_out)
        else:
            self.mapToGuideLocs(self.controls_list[1], guide.locs[0])

        xform = guide.locs[2].worldMatrix[0].get()
        if self.comp_side == 'R':
            self.orbit_out = dag.addChild(self.rig, 'group', name=self.getName('orbit_out_srt'))
            self.orbit_out.offsetParentMatrix.set(xform)
            xform = mathOps.invertHandedness(xform)
        self.orbit_ctrl = self.addCtrl(shape='circlePoint', size=ctrlSize*.5,
                                       name=self.getName('orbit'), xform=xform, parent=self.controls,
                                       metaParent=self.controls_list[1])
        if self.comp_side == 'R':
            self.mapToGuideLocs(self.orbit_out, guide.locs[2])
            self.copyGuideMapping(self.orbit_out, self.orbit_ctrl)
            # Add mapping connection to control - used when mapping controller tags
            self.mapToControl(self.orbit_ctrl, self.orbit_out)
        else:
            self.mapToGuideLocs(self.controls_list[2], guide.locs[2])

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('jnt'))
            driver = self.controls_list[1]
            if self.comp_side == 'R':
                driver = self.fk_out
            self.joints_list.append({'joint': j, 'driver': driver})
            self.mapJointToGuideLocs(j, guide.locs[0])

        components.TBaseComponent.addObjects(self, guide)

    def addSystems(self):
        if self.comp_side == 'R':
            negMtx = mathOps.createComposeMatrix(inputScale=(-1, 1, 1), name=self.getName('invertHandedness_mtx'))
            fk_mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, self.fk_ctrl.worldMatrix[0]],
                                              name=self.getName('fk_out_mtx'))
            fk_mtx.matrixSum.connect(self.fk_out.offsetParentMatrix)

            orbit_mtx = mathOps.multiplyMatrices([negMtx.outputMatrix, self.orbit_ctrl.worldMatrix[0]],
                                                  name=self.getName('orbit_out_mtx'))
            orbit_mtx.matrixSum.connect(self.orbit_out.offsetParentMatrix)

        # ---------------------------------
        # Internal spaces switching setup
        # ---------------------------------
        self.spaces['%s' % (self.orbit_ctrl.name())] = 'clavicle: %s.worldMatrix[0]' % self.fk_ctrl.name()

        # Attach params shape to base srt
        tempJoint = pm.createNode('joint')
        skn = pm.skinCluster(tempJoint, self.params)
        pm.skinCluster(skn, e=1, ai=self.base_srt, lw=1, wt=1)
        pm.delete(tempJoint)

    def finish(self):
        self.setColours(self.guide)

        nodeList = self.controls_list
        attrList = ['visibility']
        attribute.channelControl(nodeList=nodeList, attrList=attrList)

        nodes = [node for node in self.controls_list if not node == self.params]
        attribute.channelControl(nodeList=nodes, attrList=['rotateOrder'], keyable=1, lock=0)

        spaceAttrs = [attr for attr in ['orbit_ctrl_parent_space', 'orbit_ctrl_translate_space',
                                        'orbit_ctrl_rotate_space'] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.orbit_ctrl)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TShoulderFK(guide)

