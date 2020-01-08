#tRigger
from tRigger.core import attribute, dag, icon, transform
reload(attribute)
reload(dag)
reload(icon)
reload(transform)

import pymel.core as pm


class TRig(object):
    '''
    Main class that contains components
    '''
    def __init__(self, name):
        self.root = pm.createNode('transform', name=name)
        self.components = dag.addChild(self.root, 'group', 'components')
        self.joints = dag.addChild(self.root, 'group', 'joints')
        self.geo = dag.addChild(self.root, 'group', 'geo')
        self.rig_name = name
        attribute.addStringAttr(self.root, 'rig_name', name)
        print 'New TRig instance created: %s' % name

class TBaseComponent(object):
    '''
    Class from which all rig components inherit
    '''
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
        '''
        Creates the basic dag structure for the component
        '''
        self.root = pm.createNode('transform', name=self.getName('comp'))
        self.input = dag.addChild(self.root, 'group', name=self.getName('input'))
        self.controls = dag.addChild(self.root, 'group', name=self.getName('controls'))
        self.params = dag.addChild(self.controls, 'group', name=self.getName('anim_params'))
        self.base_srt = dag.addChild(self.controls, 'group', name=self.getName('base_srt'))
        transform.align(self.base_srt, self.guide.root)
        self.rig = dag.addChild(self.root, 'group', name=self.getName('rig'))
        self.output = dag.addChild(self.root, 'group', name=self.getName('output'))
        self.deform = dag.addChild(self.root, 'group', name=self.getName('deform'))

    def getName(self, name):
        '''
        Returns: (string) valid name for new node prefixed with 'compName_compSide_compIndex_'
        '''
        return '%s_%s%s_%s' % (self.comp_name, self.comp_side, self.comp_index, name)

    def requestIndex(self, index):
        '''
        Returns: (Int) Next available component index
        '''
        compNodes = [node for node in pm.ls(type='transform') if node.hasAttr('comp_type')]
        compIndices = [node.comp_index.get() for node in compNodes if node.comp_type.get() == self.comp_type.get()]
        if not index in compIndices:
            return index
        else:
            return max(compIndices) + 1

    def addCtrl(self, shape, size, name, xform, parent=None, buffer=0):
        '''
        Creates an icon for use as a control object
        Args:
            shape: (string) What kind of icon to use for the control's nurbs curve
            size: (float) Size of the newly created icon
            name: (string) Nmae of the new control
            xform: (pymel.Datatypes.matrix) 4x4 Matrix describing the placement of the control
            parent: (pyNode) DAG transform under which to parent the control
            buffer: (bool) if true, creates an extra transform above the control and below parent

        Returns: (pyNode) The new control object

        '''
        ctrl = eval('icon.%sIcon(%s, "%s")' % (shape, size, '%s_ctrl' % name))
        if parent:
            ctrl.setParent(parent)
        else:
            ctrl.setParent(self.base_srt)
        self.controls_list.append(ctrl)
        if buffer:
            buffer = dag.addParent(ctrl, 'group', name='%s_buffer_srt' % name)
            if parent:
                buffer.offsetParentMatrix.set(xform * parent.worldInverseMatrix[0].get())
            else:
                buffer.offsetParentMatrix.set(xform)
            buffer.t.set((0, 0, 0))
            buffer.r.set((0, 0, 0))
        else:
            if parent:
                ctrl.offsetParentMatrix.set(xform * parent.worldInverseMatrix[0].get())
            else:
                ctrl.offsetParentMatrix.set(xform)
            ctrl.t.set((0, 0, 0))
            ctrl.r.set((0, 0, 0))
            ctrl.s.set((1, 1, 1))
        return ctrl

    def addOutput(self, node):
        '''
        Exposes the worldMatrix of a node within the component for use by another component. A matrix attr is added
        to the component's 'output' group
        Args:
            node: (pyNode) the node whose matrix needs to be exposed

        Returns: (pyMel.Attribute) the newly created matrix attribute

        '''
        conns = [conn for conn in pm.listConnections(self.output, p=1, c=1, s=1) if conn[1] == node.worldMatrix[0]]
        if conns:
            return conns[0][0]
        else:
            attr = attribute.addMatrixAttr(self.output, '%s_outMtx' % node.name())
            node.worldMatrix[0].connect(attr)
            return attr

    def connectToInput(self, input, node):
        '''
        Connects the offsetParentMatrix of a node to an input from another component. If there is an offset required,
        this is piped into multmatrix before being connected.
        Args:
            input: (pyMel.Attribute) Matrix attribute on the component's 'input' group to which node should be connected
            node: (pyNode) Node that should be driven by the input matrix

        Returns: None

        '''
        targMtx = node.worldMatrix[0].get()
        mtxAttr = input
        suffix = input.name().split('.')[1].replace('_mtx', '')
        if input.get() != targMtx:
            offsetMtx = targMtx * input.get().inverse()
            offset = transform.pmMtx2fourFourMtx(offsetMtx, name=self.getName('%s_offsetMtx' % suffix))
            multMtx = transform.multiplyMatrices([offset.output, input], name=self.getName('%s_mtx' % suffix))
            mtxAttr = multMtx.matrixSum

        mtxAttr.connect(node.offsetParentMatrix)
        node.t.set((0, 0, 0))
        node.r.set((0, 0, 0))

    def connectToMultiInputs(self, inputs, enumNames, node):
        '''
        Implements space switching on a node.
        Args:
            inputs: ([pymel.Datatypes.matrix]) matrix attributes to pick from
            enumNames: ([string]) Names to expose to the UI to pick from
            node: node whose space will be switched.

        Returns: None

        '''
        name = '_'.join(node.name().split('_')[2:])
        switch = pm.createNode('choice', name=self.getName('%s_spaceSwitch' % name))
        for i, input in enumerate(inputs):
            suffix = input.name().split('.')[1].replace('_mtx', '')
            targMtx = node.worldMatrix[0].get()
            offsetMtx = targMtx * input.get().inverse()
            offset = transform.pmMtx2fourFourMtx(offsetMtx,
                                                 name=self.getName('%s_%s_offsetMtx' % (suffix, name)))
            multMtx = transform.multiplyMatrices([offset.output, input],
                                                 name=self.getName('%s_%s_mtx' % (suffix, name)))
            multMtx.matrixSum.connect(switch.input[i])
        switchAttr = attribute.addEnumAttr(self.params, 'parent_space', [enum for enum in enumNames])
        switchAttr.connect(switch.selector)
        switch.output.connect(node.offsetParentMatrix)
        node.t.set((0, 0, 0))
        node.r.set((0, 0, 0))
        node.inheritsTransform.set(0)

    def connectToMultiInputsSplit(self, inputs, enumNames, node):
        '''
        Implements space switching on a node with separate choices for translate and rotate.
        Scale is coupled with translate.
        Args:
            translateInputs: ([pymel.Datatypes.matrix]) matrix attributes to pick from for translate
            rotateInputs: ([pymel.Datatypes.matrix]) matrix attributes to pick from for rotate
            translateEnumNames: ([string]) Names to expose to the UI to pick from for translate
            rotateEnumNames: ([string]) Names to expose to the UI to pick from for rotate
            node: node whose space will be switched.

        Returns: None

        '''
        name = '_'.join(node.name().split('_')[2:])
        translateSwitch = pm.createNode('choice', name=self.getName('%s_translateSpaceSwitch' % name))
        rotateSwitch = pm.createNode('choice', name=self.getName('%s_rotateSpaceSwitch' % name))
        translatePick = pm.createNode('pickMatrix', name=self.getName('%s_translate_mtx' % name))
        rotatePick = pm.createNode('pickMatrix', name=self.getName('%s_rotate_mtx' % name))
        blend = pm.createNode('blendMatrix', name=self.getName('%s_result_mtx' % name))

        translateSwitch.output.connect(translatePick.inputMatrix)
        translatePick.useRotate.set(0)
        translatePick.useShear.set(0)

        rotateSwitch.output.connect(rotatePick.inputMatrix)
        rotatePick.useTranslate.set(0)
        rotatePick.useScale.set(0)

        translatePick.outputMatrix.connect(blend.inputMatrix)
        rotatePick.outputMatrix.connect(blend.target[0].targetMatrix)
        blend.target[0].useTranslate.set(0)
        blend.target[0].useScale.set(0)

        for i, input in enumerate(inputs):
            suffix = input.name().split('.')[1].replace('_mtx', '')
            targMtx = node.worldMatrix[0].get()
            offsetMtx = targMtx * input.get().inverse()
            offset = transform.pmMtx2fourFourMtx(offsetMtx,
                                                 name=self.getName('%s_%s_offsetMtx' % (suffix, name)))
            multMtx = transform.multiplyMatrices([offset.output, input],
                                                 name=self.getName('%s_%s_mtx' % (suffix, name)))
            multMtx.matrixSum.connect(translateSwitch.input[i])
            multMtx.matrixSum.connect(rotateSwitch.input[i])
        switchAttr = attribute.addEnumAttr(self.params, 'translate_space', [enum for enum in enumNames])
        switchAttr.connect(translateSwitch.selector)
        switchAttr = attribute.addEnumAttr(self.params, 'rotate_space', [enum for enum in enumNames])
        switchAttr.connect(rotateSwitch.selector)
        blend.outputMatrix.connect(node.offsetParentMatrix)
        node.t.set((0, 0, 0))
        node.r.set((0, 0, 0))
        node.inheritsTransform.set(0)

    def mapToGuideLocs(self, rigNode, guideNode):
        '''
        Creates a binding between a rig node and a guide node. This is used when other components want to connect to
        nodes within the current one via guide parenting
        Args:
            rigNode: (pyNode) the rig node
            guideNode: (pyNode) the guide node

        Returns: None

        '''
        if type(rigNode) == 'str' or type(rigNode) == 'unicode':
            rigNode, guideNode = pm.PyNode(rigNode), pm.PyNode(guideNode)
        self.guideToRigMapping[guideNode] = rigNode

    def mapJointToGuideLocs(self, jointNode, guideNode):
        '''
        Creates a binding between a rig node and a guide node. This is used when other components want to connect to
        nodes within the current one via guide parenting
        Args:
            jointNode: (pyNode) the joint node
            guideNode: (pyNode) the guide node

        Returns: None

        '''
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
        '''
        Creates the connections and non DAG nodes in order to drive the component's behaviour.
        Overload this function in derived component classes to add any solvers / rigging / connections required
        by the component.
        Args:

        Returns: None

        '''
        print 'Added Systems: %s' % self.comp_name

    def addDeformers(self, rig, rObj):
        '''
        Creates the attributes and I/O necessary to drive its joints.
        Creates an 'input' group inside rig's 'joints' group. This is where the connection from the component to drive
        each joint comes in.
        Args:
            rig: (dict) dictionary object containing information about the rig as a whole
            rObj: (TRig) Instance of the base rig class

        Returns: None

        '''
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
            joints = [self.joints_list[i]['joint'] for i in range(len(self.joints_list))]

            # Create input node inside rig's joints group. This is where the connection from the component to drive
            # each joint comes in.
            input = dag.addChild(rObj.joints, 'group', self.getName('joints_input'))
            for i, j in enumerate(self.joints_list):
                if not joints[i].getParent() in joints:
                    j['joint'].setParent(parentJoint)
                name = j['joint'].name()
                outAttr = attribute.addMatrixAttr(self.deform, '%s_out_mtx' % name)
                j['driver'].worldMatrix[0].connect(outAttr)
                inAttr = attribute.addMatrixAttr(input, '%s_in_mtx' % name)
                outAttr.connect(inAttr)
                mul = transform.multiplyMatrices([inAttr, j['joint'].getParent().worldInverseMatrix[0]],
                                                 name=self.getName('%s_mtx' % name))
                mul.matrixSum.connect(j['joint'].offsetParentMatrix)
                j['joint'].jo.set(0,0,0)
                j['joint'].t.set((0,0,0))
                j['joint'].r.set((0,0,0))

                try:
                    j['joint'].getParent().s.disconnect(j['joint'].inverseScale)
                except:
                    pass
                j['joint'].segmentScaleCompensate.set(0)

        print 'Added Deformers: %s' % self.comp_name

    def addConnections(self, rig):
        '''
        Create I/O connections for the component both static and dynamic (spaceSwitching)
        Args:
            rig: (dict) dictionary object containing information about the rig as a whole

        Returns: None

        '''
        # First add simple connections based on guide hierarchy
        parent = self.guide.getGuideParent()
        if parent:
            compObj = pm.PyNode('_'.join(parent.name().split('_')[:2]) + '_comp')
            parentObj = rig[compObj.name()].guideToRigMapping[parent]
            output = rig[compObj.name()].addOutput(parentObj)
            input = attribute.addMatrixAttr(self.input, 'parent_in_mtx')
            output.connect(input)
            self.connectToInput(input, self.base_srt)
            print 'Parent Object: %s' % parentObj.name()
        else:
            print 'No parent found: %s' % self.guide.root.name()
        print 'Added Connections: %s' % self.comp_name

        # Next check for nodes that have multiple spaces
        switchNodes = [node for node in self.guide.locs if node.hasAttr('spaces')]
        for node in switchNodes:
            spaces = [space for space in node.spaces.get().replace(' ', '').split(',')]
            enumNames = []
            inputs = []
            if len(spaces) != 0:
                if spaces[0]:
                    for space in spaces:
                        spaceName, spaceTarg = space.split(':')
                        enumNames.append(spaceName)
                        inputName = '%s_in_mtx' % spaceName
                        try:
                            input = pm.Attribute('%s.%s' % (self.input.name(), inputName))
                        except:
                            input = attribute.addMatrixAttr(self.input, inputName)
                        inputs.append(input)

                        compObj = pm.PyNode('_'.join(spaceTarg.split('_')[:2]) + '_comp')
                        parentObj = rig[compObj.name()].guideToRigMapping[pm.PyNode(spaceTarg)]
                        output = rig[compObj.name()].addOutput(parentObj)
                        output.connect(input)
                    drivenNode = self.guideToRigMapping[node]
                    if node.hasAttr('is_tGuide_root'):
                        drivenNode = self.controls_list[0]
                    print 'controls: %s' % self.controls_list
                    if node.splitTranslateAndRotate.get():
                        self.connectToMultiInputsSplit(inputs, enumNames, drivenNode)
                    else:
                        self.connectToMultiInputs(inputs, enumNames, drivenNode)



    def finish(self):
        # Overload this function in derived component classes to
        # complete the component setup. Lock attrs, hide unnecessary nodes etc.
        print 'Finished: %s' % self.comp_name

