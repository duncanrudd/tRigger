#tRigger
from tRigger.core import attribute, dag, icon, transform
reload(attribute)
reload(dag)
reload(icon)
reload(transform)

import pymel.core as pm


class TRig(object):
    def __init__(self, name):
        self.root = pm.createNode('transform', name=name)
        self.components = dag.addChild(self.root, 'group', 'components')
        self.joints = dag.addChild(self.root, 'group', 'joints')
        self.geo = dag.addChild(self.root, 'group', 'geo')
        self.rig_name = name
        attribute.addStringAttr(self.root, 'rig_name', name)
        print 'New TRig instance created: %s' % name

class TBaseComponent(object):
    def __init__(self, name, side, index, compType):
        self.comp_name = name
        self.comp_type = compType
        self.comp_side = side
        self.comp_index = index
        self.guideToRigMapping = {}
        self.guideToJointMapping = {}
        self.inputs = []
        self.outputs = []
        self.createGroups()
        attribute.addStringAttr(self.root, 'comp_type', 'root')
        attribute.addStringAttr(self.root, 'comp_name', name)
        attribute.addStringAttr(self.root, 'comp_side', side)
        attribute.addIntAttr(self.root, 'comp_index', self.comp_index)

        self.controls_list = []
        self.joints_list = []

    def createGroups(self):
        self.root = pm.createNode('transform', name=self.getName('comp'))
        self.input = dag.addChild(self.root, 'group', name=self.getName('input'))
        self.controls = dag.addChild(self.root, 'group', name=self.getName('controls'))
        self.rig = dag.addChild(self.root, 'group', name=self.getName('rig'))
        self.output = dag.addChild(self.root, 'group', name=self.getName('output'))
        self.deform = dag.addChild(self.root, 'group', name=self.getName('deform'))


    def getName(self, name):
        return '%s_%s%s_%s' % (self.comp_name, self.comp_side, self.comp_index, name)

    def requestIndex(self, index):
        compNodes = [node for node in pm.ls(type='transform') if node.hasAttr('comp_type')]
        compIndices = [node.comp_index.get() for node in compNodes if node.comp_type.get() == self.comp_type.get()]
        if not index in compIndices:
            return index
        else:
            return max(compIndices) + 1

    def addCtrl(self, shape, size, name, xform, parent=None):
        ctrl = eval('icon.%sIcon(%s, "%s")' % (shape, size, name))
        pm.xform(ctrl, ws=1, m=xform)
        if parent:
            ctrl.setParent(parent)
        else:
            ctrl.setParent(self.controls)
        self.controls_list.append(ctrl)

    def addOutput(self, node):
        conns = [conn for conn in pm.listConnections(self.output, p=1, c=1, s=1) if conn[1] == node.worldMatrix[0]]
        if conns:
            return conns[0][0]
        else:
            attr = attribute.addMatrixAttr(self.output, '%s_outMtx' % node.name())
            node.worldMatrix[0].connect(attr)
            return attr

    def connectToInput(self, input, node):
        targMtx = node.worldMatrix[0].get()
        mtxAttr = input
        suffix = input.name().split('.')[1].replace('_mtx', '')
        if input.get() != targMtx:
            offsetMtx = targMtx * input.get().inverse()
            offset = transform.pmMtx2fourFourMtx(offsetMtx, name=self.getName('%s_offsetMtx' % suffix))
            multMtx = transform.multiplyMatrices([offset.output, input], name=self.getName('%s_mtx' % suffix))
            mtxAttr = multMtx.matrixSum

        dm = transform.decomposeMatrix(mtxAttr, name=self.getName('%s_mtx2Srt' % suffix))
        transform.connectSrt(dm, self.controls_list[0].getParent())

    def mapToGuideLocs(self, rigNode, guideNode):
        if type(rigNode) == 'str' or type(rigNode) == 'unicode':
            rigNode, guideNode = pm.PyNode(rigNode), pm.PyNode(guideNode)
        self.guideToRigMapping[guideNode] = rigNode

    def mapJointToGuideLocs(self, jointNode, guideNode):
        if type(jointNode) == 'str' or type(jointNode) == 'unicode':
            jointNode, guideNode = pm.PyNode(jointNode), pm.PyNode(guideNode)
        self.guideToJointMapping[guideNode] = jointNode

    #----------------------------------------------------------------
    # BUILD ROUTINES
    #----------------------------------------------------------------
    def addObjects(self, guide):
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

    def addDeformers(self, rig, rObj):
        if self.guide.root.add_joint.get():
            parent = self.guide.getGuideParent()
            parentJoint = None
            if parent:
                while not parentJoint:
                    compObj = pm.PyNode('_'.join(parent.name().split('_')[:2]) + '_comp')
                    try:
                        parentJoint = rig[compObj.name()].guideToJointMapping[parent]
                    except:
                        parent = parent.getParent()
                        if parent.hasAttr('is_tGuide'):
                            parentJoint = rObj.joints
            else:
                parentJoint = rObj.joints
            self.joints_list[0]['joint'].setParent(parentJoint)

            # Create input node inside rig's joints group. This is where the connection from the component to drive
            # each joint comes in.
            input = dag.addChild(rObj.joints, 'group', self.getName('joints_input'))
            for j in self.joints_list:
                name = j['joint'].name()
                outAttr = attribute.addMatrixAttr(self.deform, '%s_out_mtx' % name)
                j['driver'].worldMatrix[0].connect(outAttr)
                inAttr = attribute.addMatrixAttr(input, '%s_in_mtx' % name)
                outAttr.connect(inAttr)
                mul = transform.multiplyMatrices([inAttr, j['joint'].getParent().worldInverseMatrix[0]],
                                                 name=self.getName('%s_mtx' % name))
                dm = transform.decomposeMatrix(mul.matrixSum, name=self.getName('%s_mtx2Srt' % name))
                transform.connectSrt(dm, j['joint'])
                j['joint'].jo.set(0,0,0)

        print 'Added Deformers: %s' % self.comp_name

    def addConnections(self, rig):
        parent = self.guide.getGuideParent()
        if parent:
            compObj = pm.PyNode('_'.join(parent.name().split('_')[:2]) + '_comp')
            parentObj = rig[compObj.name()].guideToRigMapping[parent]
            output = rig[compObj.name()].addOutput(parentObj)
            input = attribute.addMatrixAttr(self.input, 'parent_in_mtx')
            output.connect(input)
            self.connectToInput(input, self.controls_list[0].getParent())
            print 'Parent Object: %s' % parentObj.name()
        else:
            print 'No parent found: %s' % self.guide.root.name()
        print 'Added Connections: %s' % self.comp_name

    def finish(self):
        # Overload this function in derived component classes to
        # complete the component setup. Lock attrs, hide unnecessary nodes etc.
        print 'Finished: %s' % self.comp_name

