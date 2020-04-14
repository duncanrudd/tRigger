from tRigger import components
from tRigger.core import attribute, transform, dag, icon, mathOps
import pymel.core as pm
reload(components)
reload(transform)

import pymel.core as pm

class TControl(components.TBaseComponent):
    def __init__(self, guide):
        self.guide = guide
        components.TBaseComponent.__init__(self, guide.guide_name, guide.guide_side, guide.guide_index, 'control')
        print 'Created Control Component: %s' % self.comp_name

    def addObjects(self, guide):
        self.invert = (self.guide.guide_side == 'R')
        xform = pm.PyNode(guide.root).worldMatrix[0].get()
        if self.invert:
            xform = mathOps.invertHandedness(xform)
        for i in range(guide.num_ctrls):
            parent = self.base_srt
            if i > 0:
                parent = self.controls_list[-1]
            num = str(i+1).zfill(2)
            self.addCtrl(shape='squarePoint', size=20.0-(i*3),
                         name=self.getName(num), xform=xform, parent=parent, metaParent=parent)

        driver = self.controls_list[-1]

        if self.invert:
            xform = mathOps.createInverseHandedMatrix(self.controls_list[-1].worldMatrix[0],
                                                      name=self.getName('out'))
            self.out_srt = dag.addChild(self.rig, 'group', name=self.getName('out_srt'))
            xform.matrixSum.connect(self.out_srt.offsetParentMatrix)
            self.mapToGuideLocs(self.out_srt, guide.locs[-1])
            driver = self.out_srt
        else:
            self.mapToGuideLocs(self.controls_list[-1], guide.locs[-1])

        if guide.root.add_joint.get():
            j = pm.createNode('joint', name=self.getName('jnt'))
            self.joints_list.append({'joint': j, 'driver': driver})
            self.mapJointToGuideLocs(j, guide.locs[-1])
        components.TBaseComponent.addObjects(self, guide)

        # Attach params shape to end srt
        pm.cluster(icon.getShapes(self.params), wn=[self.controls_list[-1], self.controls_list[-1]],
                   name=self.getName('params_cluster'))

    def finish(self):
        self.setColours(self.guide)

        attribute.channelControl(nodeList=self.controls_list, attrList=['rotateOrder'], keyable=1, lock=0)

        ctrlName = '_' + '_'.join(self.controls_list[-1].name().split('_')[-2:])

        spaceAttrs = [attr for attr in ['%s_parent_space' % ctrlName, '%s_translate_space' % ctrlName,
                                        '%s_rotate_space' % ctrlName] if pm.hasAttr(self.params, attr)]
        for attr in spaceAttrs:
            attribute.proxyAttribute(pm.Attribute('%s.%s' % (self.params.name(), attr)), self.controls_list[-1])

        attrList = ['visibility']
        attribute.channelControl(nodeList=self.controls_list, attrList=attrList)

def build(guide):
    '''
    Called when rig is built from guide. This function should be in every component module.
    Its job is to instantiate the component
    :return: The newly created component instance
    '''
    return TControl(guide)

