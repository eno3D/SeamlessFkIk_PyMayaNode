"""
Setup callbacks to match ik and fk controls for simultaneous use.
"""

# Python imports
import math
import sys

# Maya imports
import maya.api.OpenMaya as oMa


def maya_useNewAPI():
    """
    Needed for maya to know that API 2.0 should be used.
    """
    pass


NAME = "SeamlessFkIk"
ID = oMa.MTypeId(0x90006)

class SeamlessFkIk(oMa.MPxNode):
    """
    Node to setup callbacks to match ik and fk controls a three joint chain.
    """
    # Guide attributes
    thigh_len_mobj = oMa.MObject()
    shin_len_mobj = oMa.MObject()

    # Control attributes
    # INFO: Every control needs a parent transform to position the control
    fk_01_mobj = oMa.MObject()
    fk_02_mobj = oMa.MObject()
    fk_03_mobj = oMa.MObject()
    ik_main_mobj = oMa.MObject()
    ik_pole_mobj = oMa.MObject()

    # Save references to the controls that are connected to the node
    fk_01_conn = None
    fk_02_conn = None
    fk_03_conn = None
    ik_main_conn = None
    ik_pole_conn = None

    def __init__(self):
        oMa.MPxNode.__init__(self)

    @staticmethod
    def callback(mobj, plug, user_data):
        """
        Callback.

        Args:
            mobj
            plug
            user_data
        """
        # Only calculate when all controls are connected
        if not (user_data.fk_01_conn and user_data.fk_02_conn and user_data.fk_03_conn and
                user_data.ik_main_conn and user_data.ik_pole_conn):
            return

        # Get currently selected object
        if oMa.MGlobal.getActiveSelectionList().isEmpty():
            return

        sel = oMa.MGlobal.getActiveSelectionList().getDependNode(0)
        m_obj = plug.node()

        # Cycle break -> only calculate for the currently selected object.
        if m_obj == user_data.fk_01_conn == sel:
            SeamlessFkIk.fk_to_ik(user_data)
            return

        if m_obj == user_data.fk_02_conn == sel:
            SeamlessFkIk.fk_to_ik(user_data)
            return

        if m_obj == user_data.fk_03_conn == sel:
            SeamlessFkIk.fk_to_ik(user_data)
            return

        if m_obj == user_data.ik_main_conn == sel:
            SeamlessFkIk.ik_to_fk(user_data)
            return

        if m_obj == user_data.ik_pole_conn == sel:
            SeamlessFkIk.ik_to_fk(user_data)
            return

    @staticmethod
    def fk_to_ik(node):
        """
        Calculates ik values corresponding to the current fk values and applies them.

        Args:
            node
        """
        # Get relevant data
        ik_pole_off = get_parent(node.ik_pole_conn)

        world_trans_ik_pole_off = get_world_trans(ik_pole_off)
        world_trans_fk_01 = get_world_trans(node.fk_01_conn)
        world_trans_fk_02 = get_world_trans(node.fk_02_conn)
        world_trans_fk_03 = get_world_trans(node.fk_03_conn)
        world_trans_ik_pole = get_world_trans(node.ik_pole_conn)

        world_rot_fk_03 = get_world_rot(node.fk_03_conn)

        # calculate ik pole position
        ik_pole_mid_point = (world_trans_fk_01 + world_trans_fk_03) / 2
        ik_pole_base = world_trans_fk_02 - ik_pole_mid_point

        # Handle the case when the leg is fully stretched
        if ik_pole_base.length() <= 0.0001:
            rot_fk_01 = get_rot_as_quat(node.fk_01_conn)
            rot_fk_02 = get_rot_as_quat(node.fk_02_conn)

            rot = rot_fk_01 * rot_fk_02

            ik_pole_base = oMa.MVector(2 * (rot.x * rot.z + rot.w * rot.y),
                                           2 * (rot.y * rot.z - rot.w * rot.x),
                                           1 - 2 * (rot.x * rot.x + rot.y * rot.y))

        ik_pole_len = (world_trans_ik_pole - world_trans_fk_02).length()

        pos_ik_pole = world_trans_fk_02 + ik_pole_base.normalize() * ik_pole_len - world_trans_ik_pole_off

        # Get the destination MPlugs
        ik_main_trans_plugs = get_trans_plugs(node.ik_main_conn)
        ik_main_rot_plugs = get_rot_plugs(node.ik_main_conn)
        ik_pole_trans_plugs = get_trans_plugs(node.ik_pole_conn)

        # Set the new values
        for i, plug in enumerate(ik_main_trans_plugs):
            plug.setFloat(world_trans_fk_03[i])

        for i, plug in enumerate(ik_main_rot_plugs):
            plug.setMAngle(oMa.MAngle(world_rot_fk_03[i], oMa.MAngle.kRadians))

        for i, plug in enumerate(ik_pole_trans_plugs):
            plug.setFloat(pos_ik_pole[i])

    @staticmethod
    def ik_to_fk(node):
        """
        Calculates fk values corresponding to the current ik values and applies them.

        Args:
            node
        """
        ik_main_off = get_parent(node.ik_main_conn)
        fk_01_off = get_parent(node.fk_01_conn)
        fk_02_off = get_parent(node.fk_02_conn)
        fk_03_off = get_parent(node.fk_03_conn)

        ik_main_world_trans = get_world_trans(node.ik_main_conn)
        fk_01_world_trans = get_world_trans(node.fk_01_conn)
        ik_main_off_world_trans = get_world_trans(ik_main_off)
        fk_01_off_world_trans = get_world_trans(fk_01_off)
        fk_02_off_world_trans = get_world_trans(fk_02_off)
        fk_03_off_world_trans = get_world_trans(fk_03_off)

        # calculate base information
        def_len = (ik_main_off_world_trans - fk_01_off_world_trans).length()

        # Calculate ik direction
        ik_dir_01 = ik_main_off_world_trans - fk_01_off_world_trans
        ik_dir_02 = ik_main_world_trans - fk_01_world_trans

        ik_dir_rot = ik_dir_01.rotateTo(ik_dir_02).asEulerRotation()

        # Apply ik direction -> important to calculate correct pole rotations
        fk_01_rot_plugs = get_rot_plugs(node.fk_01_conn)
        for i, plug in enumerate(fk_01_rot_plugs):
            plug.setMAngle(oMa.MAngle(ik_dir_rot[i], oMa.MAngle.kRadians))

        # Calculate ik pole rotations
        ik_pole_world_mat = get_world_matrix(node.ik_pole_conn, 0)
        fk_03_world_inv_mat = get_world_inv_matrix(node.fk_01_conn, 0)

        ik_pole_rot_mat = ik_pole_world_mat * fk_03_world_inv_mat

        ik_pole_vec = oMa.MTransformationMatrix(ik_pole_rot_mat).translation(oMa.MSpace.kWorld)
        ik_pole_vec.y = 0

        ik_pole_rot = oMa.MVector.kZaxisVector.rotateTo(ik_pole_vec).asEulerRotation()

        # Calculate ik rotations
        tri_a_len = (fk_02_off_world_trans - fk_01_off_world_trans).length()
        tri_b_len = (fk_03_off_world_trans - fk_02_off_world_trans).length()
        tri_c_len = (ik_main_world_trans - fk_01_world_trans).length()

        if tri_c_len >= def_len:
            fk_02_angle = 0
            fk_01_angle = 0
        else:
            fk_02_angle =  math.pi - solve_triangle(tri_a_len, tri_b_len, tri_c_len, "C")
            fk_01_angle = -solve_triangle(tri_a_len, tri_b_len, tri_c_len, "B")

        # Add rotations together
        fk_01_temp = oMa.MEulerRotation(fk_01_angle, ik_pole_rot.y, 0)

        ik_dir_mat = compose_mat(ik_dir_rot)
        fk_01_mat = compose_mat(fk_01_temp)
        rot_mat = fk_01_mat * ik_dir_mat

        # Apply everything
        fk_01_rot = get_rot_from_mat(rot_mat)
        fk_02_rot = (fk_02_angle, 0, 0)

        fk_01_rot_plugs = get_rot_plugs(node.fk_01_conn)
        for i, plug in enumerate(fk_01_rot_plugs):
            plug.setMAngle(oMa.MAngle(fk_01_rot[i], oMa.MAngle.kRadians))

        fk_02_rot_plugs = get_rot_plugs(node.fk_02_conn)
        for i, plug in enumerate(fk_02_rot_plugs):
            if not plug.isLocked:
                plug.setMAngle(oMa.MAngle(fk_02_rot[i], oMa.MAngle.kRadians))

        # Calculate ankle rotation
        fk_03_rot = rot_world_space_to_local_space(node.ik_main_conn, get_parent(node.fk_03_conn))

        fk_03_rot_plugs = get_rot_plugs(node.fk_03_conn)
        for i, plug in enumerate(fk_03_rot_plugs):
            plug.setMAngle(oMa.MAngle(fk_03_rot[i], oMa.MAngle.kRadians))

    # noinspection PyMethodOverriding
    def connectionMade(self, plug, other_plug, as_src):
        """
        Saves the object connected and applies the callback to it.

        Args:
            plug
            other_plug
            as_src
        """
        if plug.attribute() == self.fk_01_mobj:
            self.fk_01_conn = other_plug.node()
        elif plug.attribute() == self.fk_02_mobj:
            self.fk_02_conn = other_plug.node()
        elif plug.attribute() == self.fk_03_mobj:
            self.fk_03_conn = other_plug.node()
        elif plug.attribute() == self.ik_main_mobj:
            self.ik_main_conn = other_plug.node()
        elif plug.attribute() == self.ik_pole_mobj:
            self.ik_pole_conn = other_plug.node()
        else:
            return

        # Add the callback to the connected control
        oMa.MNodeMessage.addNodeDirtyPlugCallback(other_plug.node(), SeamlessFkIk.callback, self)

    # noinspection PyMethodOverriding
    def connectionBroken(self, plug, other_plug, as_src):
        """
        Removes the disconnected object from the saved ones and removes the callback.

        Args:
            plug
            other_plug
            as_src
        """
        if plug.attribute() == self.fk_01_mobj:
            self.fk_01_conn = None
        elif plug.attribute() == self.fk_02_mobj:
            self.fk_02_conn = None
        elif plug.attribute() == self.fk_03_mobj:
            self.fk_03_conn = None
        elif plug.attribute() == self.ik_main_mobj:
            self.ik_main_conn = None
        elif plug.attribute() == self.ik_pole_mobj:
            self.ik_pole_conn = None
        else:
            return

        # Remove the callback from the disconnected control
        remove_callbacks(other_plug.node())

    # noinspection PyMethodOverriding
    def compute(self, plug, data_block):
        pass


def creator():
    """
    Creates a new node.

    Return:
        node
    """
    return SeamlessFkIk()


def initialize():
    """
    Initializes the node.
    """
    # guide attributes
    fn_num = oMa.MFnNumericAttribute()
    SeamlessFkIk.thigh_len_mobj = fn_num.create("thighLen", "tl", oMa.MFnNumericData.kFloat)
    SeamlessFkIk.shin_len_mobj = fn_num.create("shinLen", "sl", oMa.MFnNumericData.kFloat)

    # message attributes
    fn_message = oMa.MFnMessageAttribute()
    SeamlessFkIk.fk_01_mobj = fn_message.create("fk01", "fk01")
    SeamlessFkIk.fk_02_mobj = fn_message.create("fk02", "fk02")
    SeamlessFkIk.fk_03_mobj = fn_message.create("fk03", "fk03")
    SeamlessFkIk.ik_main_mobj = fn_message.create("ikMain", "ikm")
    SeamlessFkIk.ik_pole_mobj = fn_message.create("ikPole", "ikp")

    # add attributes
    SeamlessFkIk.addAttribute(SeamlessFkIk.thigh_len_mobj)
    SeamlessFkIk.addAttribute(SeamlessFkIk.shin_len_mobj)

    SeamlessFkIk.addAttribute(SeamlessFkIk.fk_01_mobj)
    SeamlessFkIk.addAttribute(SeamlessFkIk.fk_02_mobj)
    SeamlessFkIk.addAttribute(SeamlessFkIk.fk_03_mobj)
    SeamlessFkIk.addAttribute(SeamlessFkIk.ik_main_mobj)
    SeamlessFkIk.addAttribute(SeamlessFkIk.ik_pole_mobj)


# noinspection PyPep8Naming
def initializePlugin(m_obj):
    """
    Initialize the plugin.

    Args:
        m_obj
    """
    m_plug = oMa.MFnPlugin(m_obj)

    try:
        m_plug.registerNode(NAME, ID, creator, initialize)
    except:
        sys.stderr.write("Failed to register node: %s" % NAME)
        raise


# noinspection PyPep8Naming
def uninitializePlugin(m_obj):
    """
    Uninitialize the plugin.

    Args:
        m_obj
    """
    m_plug = oMa.MFnPlugin(m_obj)

    try:
        m_plug.deregisterNode(ID)
    except:
        sys.stderr.write("Failed to deregister node: %s" % NAME)
        raise


def viewport_selection():
    """
    Iterator over current viewport selection.

    Return:
        m_obj
    """
    sel = oMa.MGlobal.getActiveSelectionList()

    for i in range(sel.length()):
        m_obj = sel.getDependNode(i)
        yield m_obj


def get_object_as_mobject(obj_name):
    """
    Given an object name it returns the corresponding MObject.

    Args:
        obj_name
    Return:
        m_obj
    """
    sel = oMa.MSelectionList()
    sel.add(obj_name)

    m_obj = sel.getDependNode(0)
    return m_obj


def get_parent(m_obj):
    """
    Given a MObject returns the corresponding parent as a MObject.

    Args:
        m_obj
    Return:
        parent
    """
    fn_dag = oMa.MFnDagNode(m_obj)
    parent = fn_dag.parent(0)

    return parent


def get_list_as_mvector(input_list):
    """
    Given a python list it returns a MVector with the corresponding values.

    Args:
        input_list
    Return:
        out_vec
    """
    out_vec = oMa.MVector(input_list[0], input_list[1], input_list[2])

    return out_vec


def get_mvector_as_list(input_mvector):
    """
    Given a MVector it returns a python list with the corresponding values.

    Args:
        input_mvector
    Return:
        out_list
    """
    out_list = [input_mvector.x, input_mvector.y, input_mvector.z]

    return out_list


def rot_world_space_to_local_space(m_obj, parent_m_obj):
    """
    Converts the world_space of a given MObject to the local space according to the given parent MObject.

    Args:
        m_obj
        parent_m_obj
    Return:
        rot
    """
    obj_world_mat = get_world_matrix(m_obj, 0)
    parent_inv_mat = get_world_inv_matrix(parent_m_obj, 0)

    local_space_mat = obj_world_mat * parent_inv_mat
    trans_matrix = oMa.MTransformationMatrix(local_space_mat)
    rot = trans_matrix.rotation()

    return rot


def remove_callbacks(m_obj):
    """
    Removes all callbacks from the given node.

    Args:
        m_obj
    """
    cbs = oMa.MMessage.nodeCallbacks(m_obj)
    for cb in cbs:
        oMa.MMessage.removeCallback(cb)


def get_world_matrix(m_obj, i):
    """
    Returns the worldMatrix MMatrix of the given MObject.

    Args:
        m_obj
        i
    Return:
        matrix
    """
    if not m_obj.hasFn(oMa.MFn.kTransform):
        return

    fn_obj = oMa.MFnDependencyNode(m_obj)
    plug = fn_obj.findPlug('worldMatrix', False).elementByLogicalIndex(i)
    matrix_obj = plug.asMObject()
    matrix_data = oMa.MFnMatrixData(matrix_obj)
    matrix = matrix_data.matrix()

    return matrix


def get_world_inv_matrix(m_obj, i):
    """
    Returns the inverseWorldMatrix MMatrix of the given MObject.

    Args:
        m_obj
        i
    Return:
        matrix
    """
    if not m_obj.hasFn(oMa.MFn.kTransform):
        return

    fn_obj = oMa.MFnDependencyNode(m_obj)
    plug = fn_obj.findPlug('worldInverseMatrix', False).elementByLogicalIndex(i)
    matrix_obj = plug.asMObject()
    matrix_data = oMa.MFnMatrixData(matrix_obj)
    matrix = matrix_data.matrix()

    return matrix


def get_parent_inv_matrix(m_obj, i):
    """
    Returns the parentInverseMatrix MMatrix of the given MObject.

    Args:
        m_obj
        i
    Return:
        matrix
    """
    if not m_obj.hasFn(oMa.MFn.kTransform):
        return

    fn_obj = oMa.MFnDependencyNode(m_obj)
    plug = fn_obj.findPlug('parentInverseMatrix', False).elementByLogicalIndex(i)
    matrix_obj = plug.asMObject()
    matrix_data = oMa.MFnMatrixData(matrix_obj)
    matrix = matrix_data.matrix()

    return matrix


def get_world_matrix_plug(m_obj, i):
    """
    Returns the worldMatrix MPlug of the given MObject.

    Args:
        m_obj
        i
    Return:
        plug
    """
    if not m_obj.hasFn(oMa.MFn.kTransform):
        return

    fn_obj = oMa.MFnDependencyNode(m_obj)
    plug = fn_obj.findPlug('worldMatrix', False).elementByLogicalIndex(i)

    return plug


def get_world_inv_matrix_plug(m_obj, i):
    """
    Returns the inverseWorldMatrix MPlug of the given MObject.

    Args:
        m_obj
        i
    Return:
        plug
    """
    if not m_obj.hasFn(oMa.MFn.kTransform):
        return

    fn_obj = oMa.MFnDependencyNode(m_obj)
    plug = fn_obj.findPlug('worldInverseMatrix', False).elementByLogicalIndex(i)

    return plug


def get_trans_plugs(m_obj):
    """
    Returns the translation MPlugs of the given MObject.

    Args:
        m_obj
    Return:
        plug
    """
    if not m_obj.hasFn(oMa.MFn.kTransform):
        return

    fn_obj = oMa.MFnDependencyNode(m_obj)

    p_names = ('tx', 'ty', 'tz')
    plugs = [fn_obj.findPlug(each_name, False) for each_name in p_names]

    return plugs


def get_rot_plugs(m_obj):
    """
    Returns the rotation MPlugs of the given MObject.

    Args:
        m_obj
    Return:
        plug
    """
    if not m_obj.hasFn(oMa.MFn.kTransform):
        return

    fn_obj = oMa.MFnDependencyNode(m_obj)

    p_names = ('rx', 'ry', 'rz')
    plugs = [fn_obj.findPlug(each_name, False) for each_name in p_names]

    return plugs


def is_trans_plug(m_plug):
    """
    Checks if a given plug is a translation plug.

    Args:
        m_plug
    Return:
        output
    """
    valid_plugs = ('translate', 'translateX', 'translateY', 'translateZ')
    plug_name = m_plug.name().split(".")[-1]

    output = plug_name in valid_plugs

    return output


def is_rot_plug(m_plug):
    """
    Checks if a given plug is a rotation plug.

    Args:
        m_plug
    Return:
        output
    """
    valid_plugs = ('rotate', 'rotateX', 'rotateY', 'rotateZ')
    plug_name = m_plug.name().split(".")[-1]

    output = plug_name in valid_plugs

    return output


def get_world_trans(m_obj):
    """
    Extracts the translation from the worldMatrix of the MObject.

    Args:
        m_obj
    Return:
        trans
    """
    plug = get_world_matrix_plug(m_obj, 0)
    matrix_obj = plug.asMObject()
    matrix_data = oMa.MFnMatrixData(matrix_obj)
    matrix = matrix_data.matrix()

    trans_matrix = oMa.MTransformationMatrix(matrix)
    trans = trans_matrix.translation(oMa.MSpace.kWorld)

    return trans


def get_world_rot(m_obj):
    """
    Extracts the rotation from the worldMatrix of the MObject.

    Args:
        m_obj
    Return:
        rot
    """
    plug = get_world_matrix_plug(m_obj, 0)
    matrix_obj = plug.asMObject()
    matrix_data = oMa.MFnMatrixData(matrix_obj)
    matrix = matrix_data.matrix()

    trans_matrix = oMa.MTransformationMatrix(matrix)
    rot = trans_matrix.rotation()

    return rot


def get_world_rot_as_quat(m_obj):
    """
    Extracts the rotation from the worldMatrix of the MObject.

    Args:
        m_obj
    Return:
        rot
    """
    plug = get_world_matrix_plug(m_obj, 0)
    matrix_obj = plug.asMObject()
    matrix_data = oMa.MFnMatrixData(matrix_obj)
    matrix = matrix_data.matrix()

    trans_matrix = oMa.MTransformationMatrix(matrix)
    rot = trans_matrix.rotation(asQuaternion=True)

    return rot


def get_rot(m_obj):
    """
    Extracts the rotation from the worldMatrix of the MObject.

    Args:
        m_obj
    Return:
        rot
    """
    mfn_obj = oMa.MFnTransform(m_obj)

    rot = mfn_obj.rotation()

    return rot


def get_rot_as_quat(m_obj):
    """
    Extracts the rotation from the worldMatrix of the MObject.

    Args:
        m_obj
    Return:
        rot
    """
    mfn_obj = oMa.MFnTransform(m_obj)

    rot = mfn_obj.rotation(asQuaternion=True)

    return rot


def get_trans_from_mat(m_mat):
    """
        Extracts the translation from a given MMatrix.

        Args:
            m_mat
        Return:
            trans
        """
    trans_matrix = oMa.MTransformationMatrix(m_mat)
    trans = trans_matrix.translation(oMa.MSpace.kWorld)

    return trans


def get_rot_from_mat(m_mat):
    """
        Extracts the rotation from a given MMatrix.

        Args:
            m_mat
        Return:
            rot
        """
    trans_matrix = oMa.MTransformationMatrix(m_mat)
    rot = trans_matrix.rotation()

    return rot


def compose_mat(rot):
    """
    Composes a new matrix with the given values.

    Args:
        rot
    Return:
        mat
    """
    trans_mat = oMa.MTransformationMatrix()
    trans_mat.setRotation(rot)

    mat = trans_mat.asMatrix()

    return mat


def solve_triangle(len_a, len_b, len_c, angle_to_solve):
    """
    Solves an triangle when given all three sides.

    Args:
        len_a
        len_b
        len_c
        angle_to_solve

    Return:
        angle
    """
    if angle_to_solve == "A":
        angle = math.acos((len_b*len_b + len_c*len_c - len_a*len_a) / (2*len_b*len_c))
    elif angle_to_solve == "B":
        angle = math.acos((len_c * len_c + len_a * len_a - len_b * len_b) / (2 * len_c * len_a))
    elif angle_to_solve == "C":
        angle = math.acos((len_a * len_a + len_b * len_b - len_c * len_c) / (2 * len_a * len_b))
    else:
        angle = 0
        print("Unsupported angle!")

    return angle
