import pymel.core as pm

from tRigger.components import guide
from tRigger.core import transform, attribute
import json
reload(guide)

animAttrList = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz', 'ro', 'sx', 'sy', 'sz']


class TControlArrayGuide(guide.TGuideBaseComponent):
    def __init__(self, guide_name='', guide_side='C', guide_index=0, locDict='{}', local_rig=0, fromDagNode=0):
        guide.TGuideBaseComponent.__init__(self, guide_name, 'controlArray', guide_side, guide_index,
                                           fromDagNode=fromDagNode)
        self.local_rig = local_rig
        for param in ['locDict', 'local_rig']:
            self.params.append(param)

        if not fromDagNode:
            self.locDict = json.loads(locDict)
            attribute.addBoolAttr(self.root, 'add_joint')
            attribute.addBoolAttr(self.root, 'local_rig', local_rig)
            attribute.addStringAttr(self.root, 'locDict')
            self.addLocs()
        else:
            self.locs = self.getGuideLocs(fromDagNode)
            self.renameLocs()
            self.locDict = {}
            self.populateLocDict()
            self.root.locDict.set(json.dumps(self.locDict))
            self.root.local_rig.set(self.local_rig)

    def renameLocs(self):
        for index, loc in enumerate(self.locs[1:]):
            num = str(index+1).zfill(2)
            loc.rename(self.getName(num))

    def addLocs(self):

        def _addLocAttrs(loc):
            self.addSpaceSwitchAttr(loc)
            attribute.addStringAttr(loc, 'ctrl_name')
            attribute.addBoolAttr(loc, 'buffer')
            attribute.addBoolAttr(loc, 'invert')
            for attr in animAttrList:
                attribute.addBoolAttr(loc, 'enable_%s' % attr)

        if not self.locDict:
            loc = self.addGuideLoc(self.getName('01'), pm.datatypes.Matrix(), self.root)
            _addLocAttrs(loc)
            for attr in animAttrList:
                pm.Attribute('%s.enable_%s' % (loc.name(), attr)).set(1)
        else:
            for index in range(len(self.locDict.keys())):
                num = str(index+1).zfill(2)
                loc = self.addGuideLoc(self.getName(num), pm.datatypes.Matrix(), self.root)
            for loc in self.locs[1:]:
                key = loc.name().split('_')[2]
                parent = pm.PyNode(self.root.name().replace('root', self.locDict[key]['parent']))
                loc.setParent(parent)

            for key in self.locDict.keys():
                loc = pm.PyNode(self.root.name().replace('root', key))
                _addLocAttrs(loc)
                for attr in animAttrList:
                    if attr not in self.locDict[key]['anim_attrs']:
                        pm.Attribute('%s.enable_%s' % (loc.name(), attr)).set(0)
                loc.ctrl_name.set(self.locDict[key]['ctrl_name'])

    def populateLocDict(self):
        self.locDict = {}
        for index, loc in enumerate(self.locs[1:]):
            num = str(index+1).zfill(2)
            parentKey = loc.getParent().name().split('_')[2]
            self.locDict[num] = {'parent': parentKey,
                                 'anim_attrs': [attr for attr in animAttrList if
                                                pm.Attribute('%s.enable_%s' % (loc.name(), attr)).get()],
                                 'ctrl_name': loc.ctrl_name.get()}

def instantiateFromDagNode(dagNode):
    return TControlArrayGuide(dagNode.guide_name.get(),
                              dagNode.guide_side.get(),
                              dagNode.guide_index.get(),
                              local_rig = dagNode.local_rig.get(),
                              locDict = dagNode.locDict.get(),
                              fromDagNode=dagNode)


def buildGuide(**kwargs):
    return TControlArrayGuide(**kwargs)

# Mirroring this guide works. However, if a mirrored guide already exists. Changes to the hierarchy will not be
# propogated when the original guide is mirrored.
# Instead. Delete the mirrored guide and mirror the original again.
