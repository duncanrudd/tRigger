import pymel.core as pm
from maya.api import OpenMaya as om2
from tRigger.core import transform
from tRigger.components import guide, attribute
reload(guide)

class TReverseFootGuide(guide.TGuideBaseComponent):
    '''
    A simple fk shoulder controller with an offsetable orbit control
    '''
    def __init__(self, guide_name='', guide_side='C', guide_index=0, add_joint=1,
                 tarsi_segs=2, attr_driven=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'reverseFoot',
                                           guide_side, guide_index, fromDagNode=fromDagNode)
        for param in ['add_joint', 'tarsi_segs', 'attr_driven']:
            self.params.append(param)
            self.tarsi_segs = tarsi_segs
            self.attr_driven = attr_driven
        if not fromDagNode:
            attribute.addBoolAttr(self.root, 'add_joint')
            attribute.addBoolAttr(self.root, 'attr_driven')
            attribute.addIntAttr(self.root, 'tarsi_segs', tarsi_segs)
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
        self.installComponentCallbacks()

    def installComponentCallbacks(self):
        try:
            attribute.removeAttributeCallback(self.root, self.tarsi_segs_CB)
            self.tarsi_segs_CB = None
        except:
            pass
        self.tarsi_segs_CB = attribute.addCallbackToAttr(self.root, 'tarsi_segs_CB', self.tarsi_segs_callback)

    def tarsi_segs_callback(self, msg, plug1, plug2, payload):
        if msg == 2056:
            mfn_dep = om2.MFnDependencyNode(plug1.node())
            if mfn_dep.findPlug('tarsi_segs', False) == plug1:
                sel = pm.selected()
                self.tarsi_segs = self.root.tarsi_segs.get()
                self.addTarsiLocs()
                pm.select(sel)
                print('callback fired')

    def addLocs(self):
        xform = pm.datatypes.Matrix()
        
        self.heelLoc = self.addGuideLoc(self.getName('heel'), xform, self.root)
        self.heelLoc.t.set((-2, -5, 0))
        self.addSpaceSwitchAttr(self.heelLoc)
        
        self.ballLoc = self.addGuideLoc(self.getName('ball'), xform, self.root)
        self.ballLoc.t.set((8, -5, 0))
        
        self.tipLoc = self.addGuideLoc(self.getName('tip'), xform, self.root)
        self.tipLoc.t.set((12, -5, 0))
        
        self.toeLoc = self.addGuideLoc(self.getName('toe'), xform, self.root)
        self.toeLoc.t.set((8, -3, 0))
        
        self.outerLoc = self.addGuideLoc(self.getName('outer'), xform, self.root)
        self.outerLoc.t.set((4, -5, -2.5))
        
        self.innerLoc = self.addGuideLoc(self.getName('inner'), xform, self.root)
        self.innerLoc.t.set((4, -5, 2.5))

        self.addTarsiLocs()

    def addTarsiLocs(self):
        toDelete = [loc for loc in self.locs if 'tarsi' in loc.name()]
        try:
            pm.delete(toDelete)
        except:
            pass
        xform = pm.datatypes.Matrix()
        for i in range(self.tarsi_segs):
            if i != 0:
                param = (1.0 / self.tarsi_segs) * i
                num = str(i).zfill(2)
                blend = transform.blendMatrices(self.locs[4].worldMatrix[0], self.root.worldMatrix[0],
                                                name=self.getName('tarsi_%s_blend_mtx') % num)
                blend.target[0].weight.set(param)
                loc = self.addGuideLoc(self.getName('tarsi_%s' % num), xform, self.root)
                blend.outputMatrix.connect(loc.offsetParentMatrix)
                loc.inheritsTransform.set(0)
        self.locs = self.getGuideLocs(self.root)

def instantiateFromDagNode(dagNode):
    return TReverseFootGuide(dagNode.guide_name.get(),
                            dagNode.guide_side.get(),
                            dagNode.guide_index.get(),
                            add_joint=dagNode.add_joint.get(),
                            tarsi_segs=dagNode.tarsi_segs.get(),
                            attr_driven=dagNode.attr_driven.get(),
                            fromDagNode=dagNode)

def buildGuide(**kwargs):
    return TReverseFootGuide(**kwargs)
