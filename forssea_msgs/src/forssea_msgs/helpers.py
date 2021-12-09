import numpy as np
import rospy as rp
from forssea_msgs.msg import Waypoint
from geometry_msgs.msg import Quaternion, Point, Pose, TransformStamped, Transform
from std_msgs.msg import Header
from transforms3d.euler import euler2mat, euler2quat, quat2euler, mat2euler

from dynamic_reconfigure.client import Client


def update_max_thrust(max_thrust):
    c = Client('/controllers/qp_thrust_allocation/dof',
                timeout=20, config_callback=lambda cfg: 0)
    if not 0 <= max_thrust <= 250:
        rp.logwarn("Cap max_thrust({}) between 0 and 250".format(max_thrust)) 
    c.update_configuration({'max_thrust': max_thrust})

def state2pos(state):
    """
    Gets the position out of a RobotState message and returns it as a vertical Numpy array.
    :type state: forssea_msgs.msg.RobotState
    :rtype numpy.ndarray
    """
    return np.array([state.p.x, state.p.y, state.p.z])


def state2vel(state):
    """
    Gets the velocity out of a RobotState message and returns it as a vertical Numpy array.
    :type state: forssea_msgs.msg.RobotState
    :rtype numpy.ndarray
    """
    return np.array([state.v.x, state.v.y, state.v.z])


def state2acc(state):
    """
    Gets the acceleration out of a RobotState message and returns it as a vertical Numpy array.
    :type state: forssea_msgs.msg.RobotState
    :rtype numpy.ndarray
    """
    return np.array([state.a.x, state.a.y, state.a.z])


def state2eul(state):
    """
    Gets the euler angles out of a RobotState message and returns them as a vertical Numpy array.
    :type state: forssea_msgs.msg.RobotState
    :rtype numpy.ndarray
    """
    return np.array([state.e.x, state.e.y, state.e.z])


def state2rot(state):
    """
    Gets the angular velocity out of a RobotState message and returns them as a vertical Numpy array.
    :type state: forssea_msgs.msg.RobotState
    :rtype numpy.ndarray
    """
    return np.array([state.w.x, state.w.y, state.w.z])


def state2ang(state):
    """
    Gets the angular acceleration out of a RobotState message and returns them as a vertical Numpy array.
    :type state: forssea_msgs.msg.RobotState
    :rtype numpy.ndarray
    """
    return np.array([state.d.x, state.d.y, state.d.z])


def state2mat(state):
    """
    Computes the rotation matrix of a RobotState message and returns it as a 3x3 Numpy array.
    :type state: forssea_msgs.msg.RobotState
    :rtype numpy.ndarray
    """
    return euler2mat(*state2eul(state))


def state2array(state):
    """
    Gets the whole state out of a RobotState message and returns it as a vertical Numpy array
    :param state: forssea_msgs.msg.RobotState
    :rtype: numpy.ndarray
    """
    return np.hstack(
        [state2pos(state), state2vel(state), state2acc(state), state2eul(state), state2rot(state), state2ang(state)])


def state2quat(state, conv="wxyz"):
    """
    Computes the quaternion of a RobotState message and returns it as a horizontal Numpy array in the specified
    convention (wxyz or xyzw).
    :type state: forssea_msgs.msg.RobotState
    :type conv: str
    :rtype numpy.ndarray
    """
    if conv == "wxyz":
        return euler2quat(*state2eul(state))
    elif conv == "xyzw":
        return np.roll(euler2quat(*state2eul(state)), -1)


def state2pose(state):
    """
    Transforms a RobotState into a Pose message
    :type state: forssea_msgs.msg.RobotState
    :rtype Pose
    """
    return Pose(Point(state.p.x, state.p.y, state.p.z), Quaternion(*state2quat(state, conv="xyzw")))


def state2tf(state):
    """
    Transforms a RobotState into a TransformStamped message
    :type state: forssea_msgs.msg.RobotState
    :rtype TransformStamped
    """
    return TransformStamped(state.header, state.local_frame_id,
                            Transform(state.p, Quaternion(*state2quat(state, conv="xyzw"))))


def tf2state(tf, state):
    """
    Transforms a TransformStamped into a RobotState
    :type tf: TransformStamped
    :type state: forssea_msgs.msg.RobotState
    """
    pos2state(np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]), state)
    quat2state(tf.transform.rotation, state)
    state.header = tf.header
    state.local_frame_id = tf.child_frame_id


def pos2state(vec, state):
    """
    Fills the RobotState message with the provided position.
    :type state: forssea_msgs.msg.RobotState
    :type vec: numpy.ndarray
    """
    state.p.deserialize(vec.astype('float'))


def vel2state(vec, state):
    """
    Fills the RobotState message with the provided velocity.
    :type state: forssea_msgs.msg.RobotState
    :type vec: numpy.ndarray
    """
    state.v.deserialize(vec.astype('float'))


def acc2state(vec, state):
    """
    Fills the RobotState message with the provided acceleration.
    :type state: forssea_msgs.msg.RobotState
    :type vec: numpy.ndarray
    """
    state.a.deserialize(vec.astype('float'))


def eul2state(vec, state):
    """
    Fills the RobotState message with the provided euler angles.
    :type state: forssea_msgs.msg.RobotState
    :type vec: numpy.ndarray
    """
    state.e.deserialize(vec.astype('float'))


def mat2state(mat, state):
    """
    Fills the RobotState message with the euler angles computed from the provided rotation matrix.
    :type state: forssea_msgs.msg.RobotState
    :type mat: numpy.ndarray
    """
    state.e.deserialize(np.array(mat2euler(mat)).astype('float'))


def rot2state(vec, state):
    """
    Fills the RobotState message with the provided angular velocity.
    :type state: forssea_msgs.msg.RobotState
    :type vec: numpy.ndarray
    """
    state.w.deserialize(vec.astype('float'))


def ang2state(vec, state):
    """
    Fills the RobotState message with the provided angular acceleration.
    :type state: forssea_msgs.msg.RobotState
    :type vec: numpy.ndarray
    """
    state.d.deserialize(vec.astype('float'))


def quat2state(quat, state):
    """
    Fills the RobotState message with the euler angles computed from the provided quaternion.
    :type quat: geometry_msgs.msg.Quaternion
    :type state: forssea_msgs.msg.RobotState
    """
    state.e.deserialize(np.array(quat2euler([quat.w, quat.x, quat.y, quat.z])).astype('float'))


def point2state(point, state):
    """
    Fills the RobotState message with the position contained in the Point message.
    :type point: geometry_msgs.msg.Point
    :type state: forssea_msgs.msg.RobotState
    """
    state.p.deserialize(np.array([point.x, point.y, point.z]).astype('float'))


def array2state(array, state):
    """
    Fills the RobotState message with the whole state contained in an numpy array
    :type array: numpy.ndarray
    :type state: forssea_msgs.msg.RobotState
    """
    pos, vel, acc, eul, rot, ang = np.split(array, 6)
    pos2state(pos, state)
    vel2state(vel, state)
    acc2state(acc, state)
    eul2state(eul, state)
    rot2state(rot, state)
    ang2state(ang, state)


def build_header(frame):
    """
    Builds a header message and returns it.
    :type frame: str
    :rtype std_msgs.msg.Header
    """
    h = Header()
    h.stamp = rp.Time.now()
    h.frame_id = frame
    return h


def eul2quat_msg(phi, theta, psi):
    """
    Convert euler angles to Quaternion message and returns it.
    :type phi: double
    :type theta: double
    :type psi: double
    :rtype geometry_msgs.msg.Quaternion
    """
    q = euler2quat(phi, theta, psi)
    return Quaternion(q[1], q[2], q[3], q[0])


def wpt2pose(wpt):
    """
    Transforms a Waypoint message into a Pose message
    :param Waypoint wpt: Waypoint to transform
    :return: Resulting Pose
    :rtype: Pose
    """
    spt = Pose()
    spt.position = Point(wpt.x, wpt.y, wpt.z)
    spt.orientation = Quaternion(0, 0, np.sin(wpt.t / 2), np.cos(wpt.t / 2))
    return spt
