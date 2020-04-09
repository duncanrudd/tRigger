#tRigger
from tRigger.core import attribute, dag, icon, transform, metadata
reload(attribute)
reload(dag)
reload(metadata)
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
        attribute.addBoolAttr(self.root, 'hide_controls_on_playback', value=1)
        attribute.addBoolAttr(self.root, 'show_joints', value=1)
        attribute.addBoolAttr(self.root, 'show_controls', value=1)
        attribute.addBoolAttr(self.root, 'lock_geo', value=1)
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
        self.spaces = {} # Internal space switching options
        self.createGroups()
        attribute.addStringAttr(self.root, 'comp_type', compType)
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

        attribute.addMessageAttr(self.root, 'meta_rig_root')

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

    def addCtrl(self, shape, size, name, xform, parent=None, buffer=0, mirrorType=0, metaParent=None):
        '''
        Creates an icon for use as a control object
        Args:
            shape: (string) What kind of icon to use for the control's nurbs curve
            size: (float) Size of the newly created icon
            name: (string) Name of the new control
            xform: (pymel.Datatypes.matrix) 4x4 Matrix describing the placement of the control
            parent: (PyNode) DAG transform under which to parent the control
            buffer: (bool) if true, creates an extra transform above the control and below parent
            mirrorType: (int) 0 = copy all values from keyable attributes (i.e. for inverse handed controls)
                              1 = copy non srt values and rotate the transform over (i.e. ik controls)
            metaParent: (pm.PyNode) the notional parent control of the new control (for pickwalking and rig traversal)
                                    as opposed to its DAG parent

        Returns: (PyNode) The new control object

        '''
        stored = None
        try:
            stored = pm.PyNode('%s_stored' % name)
        except:
            pass
        if stored:
            ctrl = pm.createNode('transform', name='%s_ctrl' % name)
            temp = pm.duplicate(stored)
            for index, shape in enumerate(icon.getShapes(temp)):
                pm.parent(shape, ctrl, s=1, r=1)
                shape.rename('%sShape%s' % (ctrl.name(), str(index+1)))
            pm.delete(temp)
        else:
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

        attribute.addEnumAttr(ctrl, 'mirror_type', ['copy', 'flip'], k=0)
        attribute.addBoolAttr(ctrl, 'is_tControl', value=1)
        attribute.addStringAttr(ctrl, 'defaults')
        attribute.addMessageAttr(ctrl, 'meta_parent')
        attribute.addMessageAttr(ctrl, 'meta_component_root')

        self.root.message.connect(ctrl.meta_component_root)
        if metaParent:
            metadata.parentConnect(metaParent, ctrl)

        return ctrl

    def addOutput(self, node):
        '''
        Exposes the worldMatrix of a node within the component for use by another component. A matrix attr is added
        to the component's 'output' group
        Args:
            node: (PyNode) the node whose matrix needs to be exposed

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
            node: (PyNode) Node that should be driven by the input matrix

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

        mtxAttr.connect(node.offsetParentMatrix, f=1)
        node.t.set((0, 0, 0))
        node.r.set((0, 0, 0))

    def connectToMultiInputs(self, inputs, enumNames, node, add=0):
        '''
        Implements space switching on a node. Creates the space switch if add=0. Adds targets if add=1
        Args:
            inputs: ([pymel.Datatypes.matrix]) matrix attributes to pick from
            enumNames: ([string]) Names to expose to the UI to pick from
            node: node whose space will be switched.
            add: (bool) whether or not to add to an existing switch or create a new one

        Returns: None

        '''
        def _getOffset(input):
            if input.name().endswith('_mtx'):
                suffix = input.name().split('.')[1].replace('_mtx', '')
            else:
                suffix = '_'.join(input.name().split('_')[2:]).split('.')[0]
            offsetMtx = targMtx * input.get().inverse()
            offset = transform.pmMtx2fourFourMtx(offsetMtx,
                                                 name=self.getName('%s_%s_offsetMtx' % (suffix, name)))
            multMtx = transform.multiplyMatrices([offset.output, input],
                                                 name=self.getName('%s_%s_mtx' % (suffix, name)))
            return multMtx

        targMtx = node.worldMatrix[0].get()
        name = '_'.join(node.name().split('_')[2:])
        print name
        if name[0].lower() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            name = '_%s' % name
            print 'name changed: %s' % name
        indexOffset = 0
        if add:
            print 'ADDING CONNECTION'
            switch = pm.PyNode('%s_spaceSwitch' % node.name())
            attr = pm.Attribute('%s.%s_parent_space' % (self.params.name(), name))
            existingNames = pm.attributeQuery('%s_parent_space' % name, node=self.params, listEnum=1)[0]
            newEnumNames = '%s:%s' % (existingNames, ':'.join(enumNames))
            pm.addAttr(attr, enumName=newEnumNames, e=1)
            indexOffset = len(existingNames.split(':'))
        else:
            print 'CREATING CONNECTION'
            switch = pm.createNode('choice', name=self.getName('%s_spaceSwitch' % name))
            switchAttr = attribute.addEnumAttr(self.params, '%s_parent_space' % name, [enum for enum in enumNames])

            switchAttr.connect(switch.selector)
            switch.output.connect(node.offsetParentMatrix, f=1)
            node.t.set((0, 0, 0))
            node.r.set((0, 0, 0))
            node.inheritsTransform.set(0)
        for i, input in enumerate(inputs):
            multMtx = _getOffset(input)
            multMtx.matrixSum.connect(switch.input[i + indexOffset])

    def connectToMultiInputsSplit(self, inputs, enumNames, node, add=0):
        '''
        Implements space switching on a node with separate choices for translate and rotate.
        Scale is coupled with translate.
        Args:
            inputs: ([pymel.Datatypes.matrix]) matrix attributes to pick from
            enumNames: ([string]) Names to expose to the UI to pick from
            node: node whose space will be switched.
            add: (bool) whether or not to add to an existing switch or create a new one

        Returns: None

        '''
        name = '_'.join(node.name().split('_')[2:])
        if name[0].lower() in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            name = '_%s' % name
            print 'name changed: %s' % name
        targMtx = node.worldMatrix[0].get()
        indexOffset=0

        def _getOffset(input):
            if input.name().endswith('_mtx'):
                suffix = input.name().split('.')[1].replace('_mtx', '')
            else:
                suffix = '_'.join(input.name().split('_')[2:]).split('.')[0]
            offsetMtx = targMtx * input.get().inverse()
            offset = transform.pmMtx2fourFourMtx(offsetMtx,
                                                 name=self.getName('%s_%s_offsetMtx' % (suffix, name)))
            multMtx = transform.multiplyMatrices([offset.output, input],
                                                 name=self.getName('%s_%s_mtx' % (suffix, name)))
            return multMtx

        if add:
            translateSwitch = pm.PyNode('%s_translateSpaceSwitch' % node.name())
            rotateSwitch = pm.PyNode('%s_rotateSpaceSwitch' % node.name())
            tAttr = pm.Attribute('%s.%s_translate_space' % (self.params.name(), name))
            rAttr = pm.Attribute('%s.%s_rotate_space' % (self.params.name(), name))
            existingNames = pm.attributeQuery('%s_translate_space' % name, node=self.params, listEnum=1)[0]
            newEnumNames = '%s:%s' % (existingNames, ':'.join(enumNames))
            pm.addAttr(tAttr, enumName=newEnumNames, e=1)
            pm.addAttr(rAttr, enumName=newEnumNames, e=1)
            indexOffset = len(existingNames.split(':'))
        else:
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

            switchAttr = attribute.addEnumAttr(self.params, '%s_translate_space' % name, [enum for enum in enumNames])
            switchAttr.connect(translateSwitch.selector)
            switchAttr = attribute.addEnumAttr(self.params, '%s_rotate_space' % name, [enum for enum in enumNames])
            switchAttr.connect(rotateSwitch.selector)
            blend.outputMatrix.connect(node.offsetParentMatrix, f=1)
            node.t.set((0, 0, 0))
            node.r.set((0, 0, 0))
            node.inheritsTransform.set(0)

        for i, input in enumerate(inputs):
            multMtx = _getOffset(input)
            multMtx.matrixSum.connect(translateSwitch.input[i + indexOffset])
            multMtx.matrixSum.connect(rotateSwitch.input[i + indexOffset])

    def mapToGuideLocs(self, rigNode, guideNode):
        '''
        Creates a binding between a rig node and a guide node. This is used when other components want to connect to
        nodes within the current one via guide parenting
        Args:
            rigNode: (PyNode) the rig node
            guideNode: (PyNode) the guide node

        Returns: None

        '''
        if type(rigNode) == 'str' or type(rigNode) == 'unicode':
            rigNode, guideNode = pm.PyNode(rigNode), pm.PyNode(guideNode)
        self.guideToRigMapping[guideNode] = rigNode

    def getGuideLocFromMappedNode(self, node):
        '''
        Checks self.guideToRigMapping dict for values matching node. Returns the guide loc node is mapped to if any.
        Args:
            node: (PyNode) the node to check for
        Returns:
            (PyNode) or (None) Depending on whether a match is found
        '''
        for key in self.guideToRigMapping.keys():
            if self.guideToRigMapping[key] == node:
                return pm.PyNode(key)
        return None

    def mapJointToGuideLocs(self, jointNode, guideNode):
        '''
        Creates a binding between a rig node and a guide node. This is used when other components want to connect to
        nodes within the current one via guide parenting
        Args:
            jointNode: (PyNode) the joint node
            guideNode: (PyNode) the guide node

        Returns: None

        '''
        if type(jointNode) == 'str' or type(jointNode) == 'unicode':
            jointNode, guideNode = pm.PyNode(jointNode), pm.PyNode(guideNode)
        self.guideToJointMapping[guideNode] = jointNode

    def setColours(self, guide):
        '''
        Sets the drawing override settings on controls and the outliner colour for the component root
        Returns:
            None
        '''
        colour = pm.Attribute('guide.centre_colour').get()
        if self.comp_side == 'R':
            colour = pm.Attribute('guide.right_colour').get()
        elif self.comp_side == 'L':
            colour = pm.Attribute('guide.left_colour').get()

        for node in self.controls_list:
            icon.setColourRGB(node, colour)

        dag.setOutlinerColour(self.root, colour)

    def addCtrlTags(self):
        '''
        Adds controller tags to all controls in the component. Parenting is done in a separate function to allow all
        rig tags to be created before attempting to make connections
        Returns:
            None
        '''
        for control in self.controls_list:
            parent = metadata.getMetaParent(control)
            tag = pm.controller(control)

    def connectCtrlTags(self):
        '''
        If the control's meta_parent attr is connected, the parent control is mapped to the tag
        Returns:
            None
        '''
        for control in self.controls_list:
            parent = metadata.getMetaParent(control)
            if parent:
                tag, parentTag = pm.controller(control, q=1), pm.controller(parent, q=1)
                if tag and parentTag:
                    tag, parentTag = pm.PyNode(tag[0]), pm.PyNode(parentTag[0])
                    index = attribute.getNextAvailableIndex(parentTag.children)
                    tag.parent.connect(pm.Attribute('%s.children[%s]' % (parentTag.name(), str(index))))

    def copyGuideMapping(self, source, dest):
        '''
        Creates a message connection between two nodes so dest can access the guideNode mapped to source. Used when
        discovering internal space switching connections on right sided components when the guide node is mapped to a
        flipped srt instead of the control itself. e.g. shoulders
        Args:
            source: (pm.PyNode) the node whose mapping we want to copy
            dest: (pm.PyNode) the node to copy the mapping to

        Returns:
            None
        '''
        attribute.addMessageAttr(dest, 'mapping_node')
        source.message.connect(dest.mapping_node)



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
        else:
            print 'No parent found: %s' % self.guide.root.name()
        print 'Added Connections: %s' % self.comp_name

        # Next check for nodes that have multiple spaces
        processed = []
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

                            compObj = pm.PyNode('_'.join(spaceTarg.split('_')[:2]) + '_comp')
                            parentObj = rig[compObj.name()].guideToRigMapping[pm.PyNode(spaceTarg)]
                            output = rig[compObj.name()].addOutput(parentObj)
                            output.connect(input)
                        inputs.append(input)
                    drivenNode = self.guideToRigMapping[node]
                    if node.hasAttr('is_tGuide_root'):
                        drivenNode = self.controls_list[0]
                    conns = [conn for conn in pm.listConnections(drivenNode.message, d=1, p=1)]
                    if conns:
                        if 'mapping_node' in conns[0].name():
                            drivenNode = pm.PyNode(conns[0].name().split('.')[0])

                    if not drivenNode in processed:
                        # If only one space is specified, do a simple connection
                        if len(inputs) == 1 and drivenNode.name() not in self.spaces.keys():
                            self.connectToInput(inputs[0], drivenNode)
                        elif node.splitTranslateAndRotate.get():
                            self.connectToMultiInputsSplit(inputs, enumNames, drivenNode)
                        else:
                            self.connectToMultiInputs(inputs, enumNames, drivenNode)
                        processed.append(drivenNode)

        # Now check for nodes with internal space switching
        switchNodes = [pm.PyNode(node) for node in self.spaces.keys()]
        for node in switchNodes:
            spaces = [space for space in self.spaces[node.name()].replace(' ', '').split(',')]
            enumNames = []
            inputs = []
            if len(spaces) != 0:
                if spaces[0]:
                    for space in spaces:
                        spaceName, spaceTarg = space.split(':')
                        enumNames.append(spaceName)
                        if '.' in spaceTarg:
                            inputs.append(pm.Attribute(spaceTarg))
                        else:
                            inputs.append(pm.PyNode(spaceTarg))

                    add = 0
                    mappedNode = node
                    if pm.hasAttr(node, 'mapping_node'):
                        conns = pm.listConnections(node.mapping_node)
                        if conns:
                            mappedNode = conns[0]
                    guideNode = self.getGuideLocFromMappedNode(mappedNode)
                    if guideNode:
                        if guideNode.splitTranslateAndRotate.get():
                            if pm.hasAttr(self.params, '%s_translate_space' % '_'.join(node.name().split('_')[2:])):
                                add = 1
                            self.connectToMultiInputsSplit(inputs, enumNames, node, add=add)
                        else:
                            if pm.hasAttr(self.params, '%s_parent_space' % '_'.join(node.name().split('_')[2:])):
                                add = 1
                            self.connectToMultiInputs(inputs, enumNames, node, add=add)

        # Add cross component meta connections. Will try to connect any control whose meta_parent is self.base_srt
        # to the node that is mapped to the guide_root's parent.
        if parent:
            controls = metadata.getComponentControls(self.root)
            if controls:
                eldestControls = [control for control in controls if metadata.getMetaParent(control) == self.base_srt]
                compObj = pm.PyNode('_'.join(parent.name().split('_')[:2]) + '_comp')
                parentObj = rig[compObj.name()].guideToRigMapping[parent]
                for control in eldestControls:
                    if pm.hasAttr(parentObj, 'is_tControl'):
                        if parentObj.is_tControl.get():
                            print 'Eldest control: %s\tParent control: %s' % (control.name(), parentObj.name())
                            metadata.parentConnect(parentObj, control)

    def finish(self):
        # Overload this function in derived component classes to
        # complete the component setup. Lock attrs, hide unnecessary nodes etc.
        print 'Finished: %s' % self.comp_name

