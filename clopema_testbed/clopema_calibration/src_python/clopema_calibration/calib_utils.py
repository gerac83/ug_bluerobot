

PACKAGE_NAME = 'clopema_calibration'

import roslib
roslib.load_manifest(PACKAGE_NAME)

import rospy
import math
import tf

from moveit_msgs.msg import RobotState, PlanningSceneComponents, Constraints, JointConstraint, RobotTrajectory
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import smach
from smach import Sequence, State
from clopema_smach import TrajectoryVisualizeState, TrajectoryExecuteState, SetServoPowerOffState
from clopema_smach import GenericAskState
from clopema_smach import BufferState


def lookup_rotation(source_frame, target_frame, tf_listener = None):
    """
    Compute rotation angle for the base station towards given frame.

    @param target_frame: Target frame
    @param tf_listener: Transform Listener (optional)
    @returns: Angle in radians
    """

    # Check the tf_listener and create new one if None
    if tf_listener is None:
        tf_listener = tf.TransformListener()

    # Get the transforamtion from baselink to frame
    (trans,rot) = tf_listener.lookupTransform(source_frame, target_frame, rospy.Time(0))

    # Compute dot product
    d = sum([a * b for (a,b) in zip([0,-1],trans)])
    d = d / math.sqrt(sum([a ** 2 for a in trans[0:2]]))

    return math.acos(d)


def parse_list(string, delim=','):
    """
    Parse list of numbers delimited by given character
    """
    return [float(s) for s in string.split(delim)]


################################################################################
# Planning Tools                                                               #
################################################################################


def joint_state_to_dict(joint_state):
    return dict(zip(joint_state.name, joint_state.position))


group_by_link = dict()
group_by_link['r1_tip_link'] = 'r1_arm'
group_by_link['r2_tip_link'] = 'r2_arm'
group_by_link['xtion1_link'] = 'r1_xtion'
group_by_link['xtion2_link'] = 'r2_xtion'

robot_by_link = dict()
robot_by_link['r1_tip_link'] = 1
robot_by_link['r2_tip_link'] = 2
robot_by_link['xtion1_link'] = 1
robot_by_link['xtion2_link'] = 2


class IKError(Exception):
    pass


class PlanningError(Exception):
    pass


def get_robot_state():
    """
    Get current robot state. It calls get_planning_scene service that is
    capability of move_group that needs to be enabled.

    @returns: Full RobotState
    """
    req = GetPlanningSceneRequest()
    req.components.components = PlanningSceneComponents.ROBOT_STATE

    proxy = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)

    try:
        res = proxy(req)
    except rospy.ServiceException, e:
        rospy.logerr("Service did not process request: %s" % str(e))
        return None

    return res.scene.robot_state


def get_joint_state(joint_name, robot_state = None):

    # Use current robot state if None
    if robot_state is None:
        robot_state = get_robot_state()

    for name, pos in zip(robot_state.joint_state.name, robot_state.joint_state.position):
        if name == joint_name:
            return pos

    raise Exception('Unable to find joint %s', joint_name)


def get_position_ik(group_name, pose_stamped, ik_link_name = None, robot_state = None):
    """
    """

    # Use current robot state if None
    if robot_state is None:
        robot_state = get_robot_state()

    # Prepare request
    req = GetPositionIKRequest()
    req.ik_request.group_name = group_name

    if ik_link_name is not None:
        req.ik_request.ik_link_name = ik_link_name

    req.ik_request.pose_stamped = pose_stamped
    req.ik_request.robot_state = robot_state
    req.ik_request.avoid_collisions = True

    # Call IK service
    ik_srv = rospy.ServiceProxy('compute_ik', GetPositionIK)
    try:
        res = ik_srv(req)
    except rospy.service.ServiceException, e:
        rospy.logerr("Service 'compute_ik' call failed: %s", str(e))
        return None

    # Check for error
    if res.error_code.val == res.error_code.SUCCESS:
        solution = res.solution
    else:
        rospy.logwarn("Unable to find IK error code: %d", res.error_code.val)
        solution = None

    # Return solution if everithing goes well
    return solution


def joint_to_robot_state(joint_state, robot_state=None):
    """
    Converts joint state to robot state filling missing joints from the current
    or given robot state.

    @joint_state: JointState to convert
    @robot_state: Optional RobotState that is used to fill missing values, if
                  None current state is used.
    """
    # Get robot state if empty
    if robot_state is None:
        robot_state = get_robot_state()

    # Convert joint state to dictionary
    js_dict = dict((x,y) for x, y in zip(joint_state.name, joint_state.position))

    # Fill joint_state values into robot_state
    rs = RobotState()
    for i,k in enumerate(robot_state.joint_state.name):
        if k in js_dict:
            rs.joint_state.name.append(k)
            rs.joint_state.position.append(js_dict[k])
            rs.joint_state.velocity.append(0)
        else:
            rs.joint_state.name.append(k)
            rs.joint_state.position.append(robot_state.joint_state.position[i])
            rs.joint_state.velocity.append(robot_state.joint_state.velocity[i])
    rs.multi_dof_joint_state = robot_state.multi_dof_joint_state
    rs.attached_collision_objects = robot_state.attached_collision_objects

    return rs


def plan_between_states(group_name, joint_state_A, joint_state_B, robot_state=None):
    """
    Plan trajectory between two joint states.

    """

    # Create robot state
    robot_state = joint_to_robot_state(joint_state_A, robot_state)

    # Prepare motion plan request
    req = GetMotionPlanRequest()
    req.motion_plan_request.start_state = robot_state
    req.motion_plan_request.group_name = group_name
    req.motion_plan_request.planner_id = "RRTConnectkConfigDefault"

    constraints = Constraints()
    constraints.name = "goal"
    for name, pos in zip(joint_state_B.name, joint_state_B.position):
        joint_constraints = JointConstraint()
        joint_constraints.joint_name = name
        joint_constraints.position = pos
        joint_constraints.tolerance_above = 0.001
        joint_constraints.tolerance_below = 0.001
        joint_constraints.weight = 100
        constraints.joint_constraints.append(joint_constraints)

    req.motion_plan_request.goal_constraints.append(constraints)

    # Call planner
    planner_srv = rospy.ServiceProxy('/plan_kinematic_path', GetMotionPlan)
    try:
        res = planner_srv(req)
    except rospy.service.ServiceException, e:
        rospy.logerr("Service 'plan_kinematic_path' call failed: %s", str(e))
        return None

    # Prepare and return output
    if res.motion_plan_response.error_code.val != res.motion_plan_response.error_code.SUCCESS:
        rospy.logwarn("Unable to find motion plan, error code: %d ", res.motion_plan_response.error_code.val)
        traj = None
    else:
        traj = res.motion_plan_response.trajectory

    return traj.joint_trajectory


def plan_to_state(group_name, state):
    """Plan trajectory for single arm from current state to given state.

    :param pose: Target state
    :type  pose: RobotState
    :returns:    Trajectory
    """
    stateA = get_robot_state()
    traj = plan_between_states(group_name, stateA.joint_state, state)

    return traj


def plan_to_trajectory(group_name, traj):
    """Plan trajectory to the start of the input trajectory.

    """
    joint_state = JointState()
    joint_state.name = traj.joint_names
    joint_state.position = traj.points[0].positions

    traj2 = plan_to_state(group_name, joint_state)

    if traj2 is None:
        rospy.logerr("Unable to plan to trajectory")

    return traj2


def to_PoseStamped(pose, frame_id, stamp = rospy.Time(0), seq = 0):
    """PoseStamped from Pose"""
    ps = PoseStamped()
    ps.pose = pose
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp
    ps.header.seq = seq
    return ps


def plan_through_states(group_name, states):
    """Plan through states and return list of trajectories.

    @param group_name:  Name of the planning group
    @param states:      List of states
    @returns:           List of trajectories
    """
    assert len(states) >= 2, "Length of the state list must be at least two"

    trajs = []
    s1 = states[0]
    for i,s2 in enumerate(states[1:]):
        traj = plan_between_states(group_name, s1, s2)

        if traj is not None:
            trajs.append(traj)
            s1 = s2
        else:
            rospy.logwarn("Unable to plan trajectory %d", i)

    return trajs


def plan_through_posearray(group_name, pos, ik_link_name = None):
    """Plan through pose array and return list of trajectories.

    :param pos: Pose array
    :type pos:  PoseArray
    :returns:   Tuple of list of trajectories and list of poses that were not reached.
    """
    assert len(pos.poses) >= 2, "Length of the pose array must be at least two"

    # Compute IK for each point
    states = []
    for i,p in enumerate(pos.poses):
        ps = to_PoseStamped(p, pos.header.frame_id)
        state = get_position_ik(group_name, ps, ik_link_name)

        if state is not None:
            states.append(state.joint_state)
        else:
            rospy.logwarn("Unable to find IK for pose %d", i)

    # Plan trajectory through each state
    trajs = plan_through_states(group_name, states)

    return trajs


def publish_poses(pose_array, tfb):
    for i,p in enumerate(pose_array.poses):
        tfb.sendTransform((p.position.x, p.position.y, p.position.z),(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), rospy.Time.now(), "c%02d" % i, pose_array.header.frame_id)


def execute_trajectory(trajectory, wait_for_execution=True):

    req = ExecuteKnownTrajectoryRequest()
    req.trajectory = RobotTrajectory()
    req.trajectory.joint_trajectory = trajectory
    req.wait_for_execution = wait_for_execution

    srv = rospy.ServiceProxy('/execute_kinematic_path', ExecuteKnownTrajectory)
    res = srv(req)

    if wait_for_execution:
        rospy.sleep(1)

    return res.error_code

################################################################################
# SMACH Tools
################################################################################


def gen_sm_capture(capture_sm, confirm=True):
    sm = Sequence(outcomes=['succeeded', 'preempted', 'aborted'], connector_outcome='succeeded')
    sm.register_input_keys(['trajectories'])
    sm_move  = gen_sm_vis_exec(confirm=confirm)
    with sm:
        Sequence.add("TRAJECTORY_BUFFER", BufferState(), remapping={'list':'trajectories', 'item':'trajectory'}, transitions = {"finished":"SERVO_OFF"})
        Sequence.add("MOVE_TO_POSE", sm_move, transitions={"aborted":"TRAJECTORY_BUFFER"})
        Sequence.add("CAPTURE", capture_sm, transitions={"aborted":"FINISH","succeeded":"TRAJECTORY_BUFFER"})
        Sequence.add("SERVO_OFF", SetServoPowerOffState())
        Sequence.add("FINISH", SetServoPowerOffState())
    return sm


def gen_sm_vis_exec(visualize = True, confirm = True, execute = True):
    sm = smach.Sequence(outcomes=['succeeded', 'preempted', 'aborted'], connector_outcome='succeeded')
    sm.register_input_keys(['trajectory'])
    msg = """Press key: [y = continue, r = repeat, a = abort]: """
    qe = GenericAskState(msg, answers={"y":"succeeded", "a":"aborted", "r":"repeat"})
    with sm:
        if visualize:
            Sequence.add("VISUALIZE", TrajectoryVisualizeState())
        if confirm and visualize:
            Sequence.add("Q_EXECUTE", qe, transitions = {'repeat':'VISUALIZE'})
        if execute:
            Sequence.add("EXECUTE", TrajectoryExecuteState())

    return sm


class WriteJointState(State):

    def __init__(self, name_template):
        State.__init__(self, outcomes = ["succeeded", "aborted", "preempted"])
        self.name_template = name_template
        self.n = 1

    def execute(self, ud):
        robot_state = get_robot_state()
        joint_state = robot_state.joint_state
        path = self.name_template % self.n
        self.n += 1

        with open(path, 'w') as f:
            f.write("NAME,POSITION\n")
            for i, name in enumerate(joint_state.name):
                f.write("%s,%f\n" % (name, joint_state.position[i]))

        return "succeeded"
