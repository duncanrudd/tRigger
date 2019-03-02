#tRigger
from tRigger.core import attribute, dag
reload(attribute)

import pymel.core as pm


class TRig(object):
    def __init__(self, name):
        self.root = pm.createNode('transform', name=name)
        self.systems = dag.addChild(self.root, 'group', 'systems')
        self.geo = dag.addChild(self.root, 'group', 'geo')
        self.rig_name = name
        attribute.addStringAttr(self.root, 'rig_name', name)
        print 'New TRig instance created: %s' % name

class TBaseComponent(object):
    def __init__(self, name, side, index, compType):
        self.comp_name = name
        self.comp_type = compType
        self.comp_side = side
        self.comp_index = self.requestIndex(index)
        self.createGroups()
        attribute.addStringAttr(self.root, 'comp_type', 'root')
        attribute.addStringAttr(self.root, 'comp_name', name)
        attribute.addStringAttr(self.root, 'comp_side', side)
        attribute.addIntAttr(self.root, 'comp_index', self.comp_index)

    def createGroups(self):
        self.root = pm.createNode('transform', name=self.getName('comp'))
        self.input = dag.addChild(self.root, 'group', name=self.getName('input'))
        self.controls = dag.addChild(self.root, 'group', name=self.getName('controls'))
        self.rig = dag.addChild(self.root, 'group', name=self.getName('rig'))
        self.output = dag.addChild(self.root, 'group', name=self.getName('output'))


    def getName(self, name):
        return '%s_%s%s_%s' % (self.comp_name, self.comp_side, self.comp_index, name)

    def requestIndex(self, index):
        compNodes = [node for node in pm.ls(type='transform') if node.hasAttr('comp_type')]
        compIndices = [node.comp_index.get() for node in compNodes if node.comp_type.get() == self.comp_type.get()]
        if not index in compIndices:
            return index
        else:
            return max(compIndices) + 1

    def addObjects(self):
        # Overload this function in derived component classes to add
        # all required dag nodes for the component
        print 'Added Objects: %s' % self.comp_name

    def addAttributes(self):
        # Overload this function in derived component classes to add
        # any config or anim parameters to the component's host node
        print 'Added Attributes: %s' % self.comp_name

    def addSystems(self):
        # Overload this function in derived component classes to add
        # any solvers / rigging / connections required by the component
        print 'Added Systems: %s' % self.comp_name

    def addDeformers(self):
        # Overload this function in derived component classes to add
        # joints to the component's output - for use in skinning
        print 'Added Deformers: %s' % self.comp_name

    def addConnections(self):
        # Overload this function in derived component classes to add
        # connections between components - simple inheritance or space switching systems
        print 'Added Connections: %s' % self.comp_name

    def finish(self):
        # Overload this function in derived component classes to
        # complete the component setup. Lock attrs, hide unnecessary nodes etc.
        print 'Finished: %s' % self.comp_name
