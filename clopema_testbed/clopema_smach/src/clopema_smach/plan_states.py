"""
Basic states for planning - wrapper for move_group interface
"""
import roslib; roslib.load_manifest('clopema_smach')
import rospy, smach, PyKDL, tf_conversions, geometry_msgs, copy

from smach import State
from smach_ros      import ServiceState
from clopema_smach  import *
from moveit_commander import MoveGroupCommander
from clopema_smach.msg import MA1400JointState
from clopema_moveit.srv import ClopemaJointInterpolation, ClopemaLinearInterpolation, ClopemaLinearInterpolationDual
import sensor_msgs
from sensor_msgs.msg._JointState import JointState

def resolve_planning_group(ik_link):
    """
    Return planning group based on the link name
    """
    if ik_link.startswith("r1_"):
        return "r1_arm"
    elif ik_link.startswith("r2_"):
        return "r2_arm"
    elif ik_link.startswith("xtion1"):
        return "r1_xtion"
    elif ik_link.startswith("xtion2"):
        return "r2_xtion"
    return None

def transform_to_tip(group, ee_frame, goal):
    """
    Transform the goal from the ee_frame to the tip link which is 
    resolved from planning group
    return (new_goal, tip_frame) 
    """
    mc = MoveGroupCommander(group)
    tip_frame = mc.get_end_effector_link()
    
    ee_pose = mc.get_current_pose(ee_frame)
    tip_pose = mc.get_current_pose(tip_frame)
    kdl_ee = tf_conversions.fromMsg(ee_pose.pose)
    kdl_tip = tf_conversions.fromMsg(tip_pose.pose)
    if isinstance(goal, geometry_msgs.msg.Pose):
        kdl_goal = tf_conversions.fromMsg(goal)
    elif isinstance(goal, geometry_msgs.msg.PoseStamped):
        kdl_goal = tf_conversions.fromMsg(goal.pose)
    else:
        rospy.logerr("Unknown pose type, only Pose and PoseStamped is allowed")
        rospy.logerr(str(type(goal)))
        return (None, tip_frame)
    
    grip = kdl_tip.Inverse() * kdl_ee
    kdl_pose = kdl_goal * grip.Inverse()
    
    return (tf_conversions.toMsg(kdl_pose), tip_frame)


class Plan1ToJointsState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                input_keys=['goal', 'robot'],
                                output_keys=['trajectory'])

    def execute(self, userdata):
        goal = userdata.goal
        mc = MoveGroupCommander("r" + str(userdata.robot) + "_arm")
        for joint in goal.__slots__:
            mc.set_joint_value_target("r" + str(userdata.robot) + "_joint_" + str(joint), getattr(goal, joint))
        plan = mc.plan()
        userdata.trajectory = plan.joint_trajectory
        if not plan.joint_trajectory.points:
            return 'aborted'
        return 'succeeded'

class Plan2ToJointsState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                input_keys=['goal_r1', 'goal_r2'],
                                output_keys=['trajectory'])

    def execute(self, userdata):
        goal_r1 = userdata.goal_r1
        goal_r2 = userdata.goal_r2
        mc = MoveGroupCommander("arms")
        for joint in goal_r1.__slots__:
            mc.set_joint_value_target("r1_joint_" + str(joint), getattr(goal_r1, joint))
        for joint in goal_r2.__slots__:
            mc.set_joint_value_target("r2_joint_" + str(joint), getattr(goal_r2, joint))
        plan = mc.plan()
        userdata.trajectory = plan.joint_trajectory
        if not plan.joint_trajectory.points:
            return 'aborted'
        return 'succeeded'
        
class PlanToHomeState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                output_keys=['trajectory'])

    def execute(self, userdata):
        goal_r1 = MA1400JointState()
        goal_r2 = MA1400JointState()
        mc = MoveGroupCommander("arms")
        for joint in goal_r1.__slots__:
            mc.set_joint_value_target("r1_joint_" + str(joint), getattr(goal_r1, joint))
        for joint in goal_r2.__slots__:
            mc.set_joint_value_target("r2_joint_" + str(joint), getattr(goal_r2, joint))
        plan = mc.plan()
        userdata.trajectory = plan.joint_trajectory
        if not plan.joint_trajectory.points:
            return 'aborted'
        return 'succeeded'
        
class Plan1ToPoseState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                input_keys=['goal', 'ik_link'],
                                output_keys=['trajectory'])

    def execute(self, userdata):
        group = resolve_planning_group(userdata.ik_link)
        if not group:
            return 'aborted'
        
        (goal, tip_frame) = transform_to_tip(group, userdata.ik_link, userdata.goal)
        mc = MoveGroupCommander(group)
        mc.set_pose_target(goal, tip_frame)
        plan = mc.plan()
        userdata.trajectory = plan.joint_trajectory
        if not plan.joint_trajectory.points:
            return 'aborted'
        return 'succeeded'

"""      
class Plan1BetweenPosesState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/clopema_planning_actions/plan_between_poses',
                              PlanBetweenPoses, request_slots=['start_pose', 'goal', 'ik_link'],
                              response_cb=self._res_cb, response_slots=['trajectory', 'error'])

    def _res_cb(self, ud, res):
        if res.error.val == 1:
            return 'succeeded'
        else:
            return 'aborted'
"""

class Plan2ToPoseState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                input_keys=['goal_1', 'ik_link_1', 'goal_2', 'ik_link_2'],
                                output_keys=['trajectory'])

    def execute(self, userdata):
        group = 'arms'            
        mc = MoveGroupCommander(group)
        group_1 = resolve_planning_group(userdata.ik_link_1)
        (goal_1, tip_frame_1) = transform_to_tip(group_1, userdata.ik_link_1, userdata.goal_1)
        group_2 = resolve_planning_group(userdata.ik_link_2)
        (goal_2, tip_frame_2) = transform_to_tip(group_2, userdata.ik_link_2, userdata.goal_2)
        mc.set_pose_target(goal_1, tip_frame_1)
        mc.set_pose_target(goal_2, tip_frame_2)
        plan = mc.plan()
        userdata.trajectory = plan.joint_trajectory
        if not plan.joint_trajectory.points:
            return 'aborted'
        return 'succeeded'

class GetRobotStateState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                              output_keys=['robot_1_state', 'robot_2_state', 'complete_state', 'robot_1_pose', 'robot_2_pose', 'ik_link_1', 'ik_link_2'])

    def execute(self, userdata):
        mc = MoveGroupCommander("r1_arm")
        userdata.robot_1_state = mc.get_current_joint_values()
        userdata.ik_link_1 = mc.get_end_effector_link()
        userdata.robot_1_pose = mc.get_current_pose()
        
        mc = MoveGroupCommander("r2_arm")
        userdata.robot_2_state = mc.get_current_joint_values()
        userdata.ik_link_2 = mc.get_end_effector_link()
        userdata.robot_2_pose = mc.get_current_pose()
        
        mc = MoveGroupCommander("arms")
        userdata.complete_state = mc.get_current_joint_values()
        return 'succeeded'
             

class Interpolate1JointsState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/clopema_planner/joint_interpolation', ClopemaJointInterpolation(),
                              request_cb=self._req_cb, input_keys=['robot', 'goals'],
                              response_cb=self._res_cb, output_keys=['trajectory'])

    def _req_cb(self, ud, req):
        msg = JointState()
        msg.name.append('r' + str(ud.robot) + '_joint_s')
        msg.name.append('r' + str(ud.robot) + '_joint_l')
        msg.name.append('r' + str(ud.robot) + '_joint_u')
        msg.name.append('r' + str(ud.robot) + '_joint_r')
        msg.name.append('r' + str(ud.robot) + '_joint_b')
        msg.name.append('r' + str(ud.robot) + '_joint_t')
        for g in ud.goals:
            msg.position = []
            msg.position.append(g.s)
            msg.position.append(g.l)
            msg.position.append(g.u)
            msg.position.append(g.r)
            msg.position.append(g.b)
            msg.position.append(g.t)
            req.poses.append(copy.deepcopy(msg))
        
    def _res_cb(self, ud, res):
        ud.trajectory = res.joint_trajectory
        if res.error:
            rospy.logerr('Can not interpolate because: ' + res.error)
            return 'aborted'
        return 'succeeded'

class Interpolate1LinearState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/clopema_planner/linear_interpolation', ClopemaLinearInterpolation(),
                              request_cb=self._req_cb, input_keys=['ik_link', 'poses'],
                              response_cb=self._res_cb, output_keys=['trajectory'])

    def _req_cb(self, ud, req):
        req.ik_link_name = ud.ik_link
        for pose in ud.poses:
            req.header.frame_id = pose.header.frame_id
            break #set to the first pose if exist
        for pose in ud.poses:
            if not req.header.frame_id == pose.header.frame_id:
                rospy.logerr('Can not interpolate for poses in different frames')
                req.ik_link_name = ''
                req.poses = []
                return
            req.poses.append(pose.pose)
        
    def _res_cb(self, ud, res):
        ud.trajectory = res.joint_trajectory
        if res.error:
            rospy.logerr('Can not interpolate because: ' + res.error)
            return 'aborted'
        return 'succeeded'
        
class Interpolate2LinearState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/clopema_planner/linear_interpolation_dual', ClopemaLinearInterpolationDual(),
                              request_cb=self._req_cb, input_keys=['ik_link_1', 'poses_1', 'ik_link_2', 'poses_2'],
                              response_cb=self._res_cb, output_keys=['trajectory'])
        
    def _req_cb(self, ud, req):
        for pose in ud.poses_1:
            req.header.frame_id = pose.header.frame_id
            break #set to the first pose if exist
        if not (len(ud.poses_1) == len(ud.poses_2)):
            rospy.logerr('Number of poses must be equal for both groups')
            req.ik_link_name_1 = ''
            req.ik_link_name_2 = ''
            return
        
        req.ik_link_name_1 = ud.ik_link_1
        req.ik_link_name_2 = ud.ik_link_2
        
        for pose in ud.poses_1:
            if not req.header.frame_id == pose.header.frame_id:
                rospy.logerr('Can not interpolate for poses in different frames')
                req.ik_link_name_1 = ''
                req.ik_link_name_2 = ''
                req.poses_1 = []
                req.poses_2 = []
                return
            req.poses_1.append(pose.pose)
        for pose in ud.poses_2:
            if not req.header.frame_id == pose.header.frame_id:
                rospy.logerr('Can not interpolate for poses in different frames')
                req.ik_link_name_1 = ''
                req.ik_link_name_2 = ''
                req.poses_1 = []
                req.poses_2 = []
                return
            req.poses_2.append(pose.pose)

    def _res_cb(self, ud, res):
        ud.trajectory = res.joint_trajectory
        if res.error:
            rospy.logerr('Can not interpolate because: ' + res.error)
            return 'aborted'
        return 'succeeded'

"""
class Plan2ToPoseStateNotTearing(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/clopema_planning_actions/plan_2_to_pose_not_tearing',
                              Plan2ToPoseNotTearing, request_slots=['goal_1', 'ik_link_1', 'goal_2', 'ik_link_2', 'cloth_size'],
                              response_cb=self._res_cb, response_slots=['trajectory', 'error'])

    def _res_cb(self, ud, res):
        if res.error.val == 1:
            return 'succeeded'
        else:
            return 'aborted'
"""
class Plan2ToPoseStateNotTearing(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                input_keys=['goal_1', 'ik_link_1', 'goal_2', 'ik_link_2','cloth_size'],
                                output_keys=['trajectory'])

    def execute(self, userdata):
        group = 'arms'            
        mc = MoveGroupCommander(group)
        group_1 = resolve_planning_group(userdata.ik_link_1)
        (goal_1, tip_frame_1) = transform_to_tip(group_1, userdata.ik_link_1, userdata.goal_1)
        group_2 = resolve_planning_group(userdata.ik_link_2)
        (goal_2, tip_frame_2) = transform_to_tip(group_2, userdata.ik_link_2, userdata.goal_2)
        mc.set_pose_target(goal_1, tip_frame_1)
        mc.set_pose_target(goal_2, tip_frame_2)
        
        plan = mc.plan()
        userdata.trajectory = plan.joint_trajectory
        if not plan.joint_trajectory.points:
            return 'aborted'
        return 'succeeded'

class PlanExtAxisState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                input_keys=['position'],
                                output_keys=['trajectory'])

    def execute(self, userdata):
        mc = MoveGroupCommander("ext")
        mc.set_joint_value_target("ext_axis", userdata.position)
        plan = mc.plan()
        userdata.trajectory = plan.joint_trajectory
        if not plan.joint_trajectory.points:
            return 'aborted'
        return 'succeeded'
        
Plan1ToXtionPoseState = Plan1ToPoseState
