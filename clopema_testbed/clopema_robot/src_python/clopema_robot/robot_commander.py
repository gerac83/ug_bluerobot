# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Oct 17, 2013

import rospy
import actionlib
import numpy as np
import services
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

from moveit_commander import MoveGroupCommander, RobotCommander
from clopema_libs.pose import pose2affine, affine2pose
from clopema_robot import services

from clopema_moveit.srv import GetCartesianPathDual, GetCartesianPathDualRequest
from moveit_msgs.msg import DisplayTrajectory, JointConstraint
from robot_state import RobotState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander.exception import MoveItCommanderException
import copy, math
from clopema_moveit.srv import ClopemaGraspFromTable, ClopemaGraspFromTableRequest
from clopema_moveit.srv import ClopemaGraspFromTableDual, ClopemaGraspFromTableDualRequest
from services import add_time_parametrisation, set_drive_power, set_robot_speed
#from clopema_moveit.srv._ClopemaGraspFromTableDual import ClopemaGraspFromTableDual

SRV_SET_SERVO_POWER_OFF = '/joint_trajectory_action/set_power_off'
SRV_SET_ROBOT_IO        = "/write_io"
SRV_GET_ROBOT_IO        = "/read_io"
SRV_GET_IK              = "/compute_ik"
SRV_CARTESIAN_PATH_DUAL = "/compute_cartesian_path_dual"
SRV_CARTESIAN_PATH      = "/compute_cartesian_path"
SRV_GRASP_FROM_TABLE    = "/clopema_planner/grasp_from_table"
SRV_GRASP_FROM_TABLE_DUAL="/clopema_planner/grasp_from_table_dual"

TOPIC_DISPLAY_PLANNED_PATH = "/move_group/display_planned_path"

ACTION_R1_GRIPPER       = "/r1_gripper/command"
ACTION_R2_GRIPPER       = "/r2_gripper/command"

GRIPPER_OPEN = 1
GRIPPER_CLOSE = 0

DX100_JOINTS = ["r1_joint_s", "r1_joint_l", "r1_joint_u", "r1_joint_r", "r1_joint_b", "r1_joint_t", "r2_joint_s", "r2_joint_l", "r2_joint_u", "r2_joint_r", "r2_joint_b", "r2_joint_t", "ext_axis"]
GROUP_FOR_EE = {"r1_gripper":"r1_arm", "r2_gripper":"r2_arm", "r2_ps":"arms", "r1_ee":"r1_arm", "r2_ee":"r2_arm", "xtion1_link_ee":"r1_xtion", "xtion2_link_ee":"r2_xtion", "r1_tip_link":"r1_arm", "r2_tip_link":"r2_arm"}


def _make_tuple(a):
    if type(a) is tuple:
        return a
    else:
        return (a,)


class ClopemaRobotCommander(MoveGroupCommander):
    """ Common interface to CloPeMa robor, it extends the MoveGroupCommander """

    def __init__(self, group_name='arms'):
        """ Initialize the robot commander for particular group."""
        MoveGroupCommander.__init__(self, group_name)
        self.set_planner_id("RRTConnectkConfigDefault")
        # Sleep in order to load an ik module
        rospy.sleep(1)
        self.robot = RobotCommander()
        self._ik_use_tip_link = True
        self._start_state = None
        self.overwrite_time_parameterization = True

        # Decrease the joint tolerance 
        self.set_goal_joint_tolerance(0.00001)


    @staticmethod
    def set_servo_power_off(force = False):
        """Shut off the servo power."""
        set_drive_power(power=False)

    @staticmethod
    def set_robot_speed(speed):
        """
        Set robot speed, possible value can be from 0 to 1. This value is
        then converted to robot speed units (i.e. speed*10000).

        Note that there is a speed limit defined by the I001 variable on the
        teach pendant. This limit is in robot speed units and will overide the
        speed set by this function.
        """
        set_robot_speed(speed)

    @staticmethod
    def get_robot_speed():
        res = set_robot_speed(-1)
        return res.current_speed

    @staticmethod
    def set_robot_io(address, value):
        raise NotImplementedError()

    @staticmethod
    def get_robot_io(address):
        raise NotImplementedError()

    @staticmethod
    def get_dx100_joints():
        return DX100_JOINTS

    @staticmethod
    def set_gripper_state(gripper_action, state, wait = False):
        """
        Set gripper state i.e. open or close.

        :arg gripper:   Gripper id, use robot.R1_GRIPPER or robot.R2_GRIPPER
        :arg state:     Gripper state, use robot.GRIPPER_OPEN or robot.GRIPPER_CLOSE
        :arg wait:      Wait for finish, True or False, False is depfault.

        Note: This function requires the node to be initialize i.e. rospy.init_node()
        to ba called, otherwise it will result in error.
        """
        # Gripper action server client
        client = actionlib.SimpleActionClient(gripper_action, GripperCommandAction)
        client.wait_for_server()

        # Execute goal
        goal = GripperCommandGoal()
        goal.command.position = state
        client.send_goal(goal)
        if wait:
            client.wait_for_result(rospy.Duration.from_sec(5.0))

    def get_current_state(self):
        """ Get a RobotState message describing the current state of the robot"""
        return self.robot.get_current_state()

    def append_ext_axis(self, start_state, trajectory, position):
        """Append trajectory with movement of the external axis.

        INPUT
            start_state         [moveit_msgs/RobotState]
            trajectory          [moveit_msgs/RobotTrajectory]
            position            [Float]
        OUTPUT
            trajectory          [moveit_msgs/RobotTrajectory or None]
        """

        traj = copy.deepcopy(trajectory)
        N = len(traj.joint_trajectory.points)

        start = start_state.joint_state.position[start_state.joint_state.name.index("ext_axis")]
        step = (position - start) / (N - 1)

        if "ext_axis" in traj.joint_trajectory.joint_names:
            index = traj.joint_trajectory.joint_names.index("ext_axis")
            for i in range(0, N):
                traj.joint_trajectory.points[i].positions = list(traj.joint_trajectory.points[i].positions)[index] = start + step * i
                traj.joint_trajectory.points[i].velocities = list(traj.joint_trajectory.points[i].velocities)[index] = 0
                traj.joint_trajectory.points[i].accelerations = list(traj.joint_trajectory.points[i].accelerations)[index] = 0
        else:
            traj.joint_trajectory.joint_names = list(traj.joint_trajectory.joint_names) + ["ext_axis"]
            for i in range(0, N):
                traj.joint_trajectory.points[i].positions = list(traj.joint_trajectory.points[i].positions) + [start + step * i]
                traj.joint_trajectory.points[i].velocities = list(traj.joint_trajectory.points[i].velocities) + [0]
                traj.joint_trajectory.points[i].accelerations = list(traj.joint_trajectory.points[i].accelerations) + [0]

        if services.check_trajectory(start_state, traj, persistent=True):
            return traj
        else:
            rospy.logerr("The trajectory is not collision free after adding external axis movement.")
            return None

        return traj

    def get_ik(self, pose=None, link=None, poses=None, links=None, robot_state=None, constraints=None):
        """Get inverse kinematic for one or two arms of the active group.

        Arguments:
            pose        : {PoseStamped} Single target pose.
            poses       : {Iterable of PoseStaped}
                          Target poses for multiple end effectors.
            link        : {String} Single end effector link.
            links       : {Iterable of strings} Multiple end effector links.
            robot_state : {RobotState message}
                          Initial robot state, if ommited current robot state is used.
            constraints : {Constrains message}
                          Constrains for iverse kinematic solver.

            Note that either pose and link or poses and links must be defined.

        Returns:
            robot_state : {RobotState message}
                          Solution to the inverse kinematic problem.
        """
        from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
        from std_msgs.msg import Header

        # Check validity of input
        assert((pose is not None and link is not None) or (poses is not None and links is not None))

        if poses is None:
            poses = [pose]
            links = [link]

        # Get IK service proxy
        rospy.wait_for_service(SRV_GET_IK)
        srv = rospy.ServiceProxy(SRV_GET_IK, GetPositionIK)

        # Get robot state if neccesary
        if robot_state is None:
            robot_state = self.get_current_state()

        for p, l in zip(poses, links):
            if type(p) is Pose:
                p = PoseStamped(Header(0, rospy.Time.now(), "base_link"), p)

            # Convert to tip link if required
            if self._ik_use_tip_link:
                p_,l = self._fix_link(p.pose, l)
                p.pose = p_

            # Pepare request
            req = GetPositionIKRequest()
            req.ik_request.group_name = GROUP_FOR_EE[l]

            # Allways avoid collisions
            req.ik_request.avoid_collisions = True
            req.ik_request.robot_state = robot_state

            if constraints is not None:
                req.ik_request.constraints = constraints

            req.ik_request.ik_link_name = l
            req.ik_request.pose_stamped = p

            res = srv(req)
            if res.error_code.val is res.error_code.SUCCESS:
                robot_state = res.solution
            else:
                rospy.logwarn("Unable to compute IK error code: %d", res.error_code.val)
                return None

        return robot_state

    @staticmethod
    def get_group_for_ee(ee_name):
        """Ger name of the group for end effector name."""
        return GROUP_FOR_EE[ee_name]

    def _convert_link(self, pose, source_link, target_link):
        """Convert link to another link.

        Note: The transformation between source and target link should be
              fixed, but it is not checked.

        Arguments:
            pose         -- Pose to be transformed.
            source_link  -- Link of the pose.
            target_link  -- Link where the pose should be tranformed.

        Returns:
            Pose for the target_link so that the source_link is in the orignal
            pose.
        """
        Ts = pose2affine(self.robot.get_link(source_link).pose().pose)
        Tt = pose2affine(self.robot.get_link(target_link).pose().pose)
        Tp = pose2affine(pose)

        T  = np.dot(np.linalg.inv(Tt), Ts)
        p  = affine2pose(np.dot(Tp, np.linalg.inv(T)))

        return p

    def _fix_link(self, pose, link):
        """Helper to fix link if other than tip link.

        Arguments:
            pose    -- Pose for given link
            link    -- Link

        Returns:
            pose, link -- Fixed pose and link
        """
        target_link = link
        if link.startswith("r1"):
            target_link = "r1_tip_link"
        elif link.startswith("r2"):
            target_link = "r2_tip_link"
        elif link.startswith("xtion1"):
            target_link = "xtion1_link_ee"
        elif link.startswith("xtion2"):
            target_link = "xtion2_link_ee"

        if link is target_link:
            return pose, link
        else:
            return self._convert_link(pose, link, target_link), target_link

    def set_start_state(self, msg):
        self._start_state = msg
        MoveGroupCommander.set_start_state(self, msg)

    def set_start_state_to_current_state(self):
        self._start_state = None
        MoveGroupCommander.set_start_state_to_current_state(self)

    def execute(self, msg):
        if self.overwrite_time_parameterization:
            start_state = self._start_state if self._start_state else self.get_current_state()
            group = self.get_name()
            msg = add_time_parametrisation(msg, start_state, group)
            if not msg.success:
                msg = None
                raise Exception("Unsucesfull time parametrisation.")
            else:
                msg = msg.parametrized_trajectory
        elif msg.joint_trajectory.points[-1].time_from_start == 0:
            raise Exception("The time from start is not set!")

        self._start_state = None
        MoveGroupCommander.execute(self, msg)

    def async_execute(self, msg):
        if self.overwrite_time_parameterization:
            start_state = self._start_state if self._start_state else self.get_current_state()
            group = self.get_name()
            msg = add_time_parametrisation(msg, start_state, group)
            if not msg.success:
                msg = None
            else:
                msg = msg.parametrized_trajectory
        elif msg.joint_trajectory.points[-1].time_from_start == 0:
            raise Exception("The time from start is not set!")

        self._start_state = None
        services.execute(msg, wait=False, persistent=True)

    def go(self, joints = None, wait = True):
        self._start_state = None
        MoveGroupCommander.go(self,joints, wait)

    def _get_start_state(self):
        if self._start_state is None:
            return self.get_current_state()
        else:
            return self._start_state

    def set_joint_value_target(self, arg1, arg2 = None, arg3 = None):
        """HotFix: See MoveGroupCommander for info."""

        if (type(arg1) is PoseStamped) or (type(arg1) is Pose):
            approx = False
            eef = ""
            if arg2 is not None:
                if type(arg2) is str:
                    eef = arg2
                else:
                    if type(arg2) is bool:
                        approx = arg2
                    else:
                        raise MoveItCommanderException("Unexpected type")
            if arg3 is not None:
                if type(arg3) is str:
                    eef = arg3
                else:
                    if type(arg3) is bool:
                        approx = arg3
                    else:
                        raise MoveItCommanderException("Unexpected type")

            # There is a problem that when the r1_ee or r2_ee are specified as
            # eef the end effector ends up 0.1 m below the target position to
            # fix this we first transform the pose to the tip_link and then call
            # MoveGroupCommander
            if self._ik_use_tip_link:
                if type(arg1) is PoseStamped:
                    p, eef = self._fix_link(arg1.pose, eef)
                    arg1.pose = p
                else:
                    arg1, eef = self._fix_link(arg1, eef)
            MoveGroupCommander.set_joint_value_target(self, arg1, eef, approx)
        else:
            MoveGroupCommander.set_joint_value_target(self, arg1, arg2, arg3)

    def compute_cartesian_path(self, waypoints, eefs, eef_step=0.01, jump_threshold=1.2, avoid_collisions=True):
        """Compute cartesian path for one or two arms.

        Arguments:
            waypoints       -- List of poses or list tuples
            eefs            -- End effector link or tuple of links
            eef_step        --
            jump_threshold  --
        """
        # Wait for service an make proxi
        if type(eefs) is tuple and len(eefs) > 1:
            rospy.wait_for_service(SRV_CARTESIAN_PATH_DUAL)
            srv = rospy.ServiceProxy(SRV_CARTESIAN_PATH_DUAL, GetCartesianPathDual)

            req = GetCartesianPathDualRequest()
            req.start_state = self._get_start_state()
            # req.robot_state = self._get_start_state()
            req.group_name = self.get_name()
            req.header.frame_id = self.get_pose_reference_frame()
            req.header.stamp = rospy.Time.now()

            waypoints1, waypoints2 = zip(*waypoints)

            req.waypoints_1 = waypoints1
            req.link_name_1 = eefs[0]
            req.link_name_2 = eefs[1]
            req.waypoints_2 = waypoints2

            req.max_step = eef_step
            req.jump_threshold = jump_threshold
            req.avoid_collisions = avoid_collisions
            if not avoid_collisions:
                rospy.logwarn("!!!!Collisions are disabled!!!!")

            res = srv.call(req)
            if res.error_code.val is res.error_code.SUCCESS:
                return (res.solution, res.fraction)
            else:
                rospy.logwarn("Unable to compute collsion free cartesian path! error code: %d", res.error_code.val)
                return None
        else:
            waypoints2, links = zip(*[self._fix_link(wp, eefs) for wp in waypoints])
            rospy.wait_for_service(SRV_CARTESIAN_PATH_DUAL)
            srv = rospy.ServiceProxy(SRV_CARTESIAN_PATH_DUAL, GetCartesianPathDual)

            req = GetCartesianPathDualRequest()
            req.start_state = self._get_start_state()
            # req.robot_state = self._get_start_state()
            req.group_name = self.get_name()
            req.header.frame_id = self.get_pose_reference_frame()
            req.header.stamp = rospy.Time.now()

            req.waypoints_1 = waypoints2
            req.link_name_1 = links[0]
            req.link_name_2 = links[0]
            req.waypoints_2 = waypoints2

            req.max_step = eef_step
            req.jump_threshold = jump_threshold
            req.avoid_collisions = avoid_collisions
            if not avoid_collisions:
                rospy.logwarn("!!!!Collisions are disabled!!!!")

            res = srv.call(req)
            if res.error_code.val is res.error_code.SUCCESS:
                print res.error_code.val, req.group_name, req.header.frame_id
                return (res.solution, res.fraction)
            else:
                rospy.logwarn("Unable to compute collsion free cartesian path! error code: %d", res.error_code.val)
                return None
            # return MoveGroupCommander.compute_cartesian_path(self, waypoints2, eef_step, jump_threshold, True)

    def plan_between_joint_states(self, js1, js2):
        """Plan path between two joint states.

        Arguments:
            js1,js2 : {JointState message}

        Returns:
            trajectory : {RobotTrajectory message}
        """
        tmp_robot_state = self._get_start_state()
        rs = RobotState(copy.deepcopy(tmp_robot_state))
        rs.update_from_joint_state(js1)
        self.set_start_state(rs.msg)
        self.set_joint_value_target(js2)
        traj = self.plan()
        self.set_start_state(tmp_robot_state)
        if len(traj.joint_trajectory.joint_names) <= 0:
            return None
        else:
            return traj

    def display_planned_path(self, path):
        """Sends the trajectory to apropriate topic to be displayed.

        Arguments:
            path    : {RobotTrajectory message} Trajectory to be displayed

        """
        self.pub = rospy.Publisher(TOPIC_DISPLAY_PLANNED_PATH, DisplayTrajectory)
        dsp = DisplayTrajectory()
        dsp.trajectory = [path]
        self.pub.publish(dsp)

    def set_path_constraints(self, value):
        # In order to force planning in joint coordinates we add dummy joint
        # constrains.
        empty_joint_constraints = JointConstraint()
        empty_joint_constraints.joint_name = "r1_joint_grip"
        empty_joint_constraints.position = 0
        empty_joint_constraints.tolerance_above = 1.5
        empty_joint_constraints.tolerance_below = 1.5
        empty_joint_constraints.weight = 1
        value.joint_constraints.append(empty_joint_constraints)
        MoveGroupCommander.set_path_constraints(self, value)
        
    def grasp_from_table_plan(self, poses, tip="r1_ee", table_desk="t3_desk", final_poses=[], base_frame="base_link",
                               offset_minus=0.02, offset_plus=0.02, table_minus=0, table_plus=0.1, grasping_angle=math.pi/6):
        
        """ 
        Grasp from table. 
        Arguments:              
            poses= [pos x, pos y, pos z, orien x, orien y orien z orien w]
            tip =  "r1_ee"
            table_desk="t3_desk"
            final_poses= optinoal
            base_frame="base_link"
            offset_minus = 0.02
            offset_plus = 0.02
            table_minus = 0
            table_plus = 0.1
            grasping_angle = math.pi/ 6
        
        Returns:
            trajectory
        """
        
        rospy.wait_for_service(SRV_GRASP_FROM_TABLE)
        srv = rospy.ServiceProxy(SRV_GRASP_FROM_TABLE, ClopemaGraspFromTable)

        req = ClopemaGraspFromTableRequest()     
        req.frame_id = base_frame 
        req.ik_link = tip
        req.table_desk = table_desk
        req.poses = poses
        req.offset_minus = offset_minus
        req.offset_plus = offset_plus
        req.offset_table_minus = table_minus
        req.offset_table_plus = table_plus
        req.grasping_angle = grasping_angle
        req.final_poses = final_poses
        
        tmp_robot_state = self._get_start_state()
        self.set_start_state(tmp_robot_state)
        
        res = srv.call(req)
        
        if res.error is "":
            return (res.joint_trajectories)
        else:
            rospy.logwarn("Unable to compute path! error code: %d", res.error)
            return None

    def grasp_from_table_dual_plan(self, poses_1, poses_2, tip1="r1_ee", tip2="r2_ee", table_desk="t3_desk", final_poses_1=[], 
                                   final_poses_2=[], base_frame="base_link",offset_minus=0.02, offset_plus=0.02, offset_table_minus=0,
                                   offset_table_plus=0.1, grasping_angle=math.pi/6):
        """ 
        Grasp from table dual. 
        Arguments:              
            poses_1, poses_2= [pos x, pos y, pos z, orien x, orien y orien z orien w]
            tip1 =  "r1_ee"
            tip2=    "r2_ee"
            table_desk="t3_desk"
            final_poses_1, final_poses_2= optinoal
            base_frame="base_link"
            offset_minus = 0.02
            offset_plus = 0.02
            table_minus = 0
            table_plus = 0.1
            grasping_angle = math.pi/ 6
        
        Returns:
            trajectory
        """
        
        rospy.wait_for_service(SRV_GRASP_FROM_TABLE_DUAL)
        srv = rospy.ServiceProxy(SRV_GRASP_FROM_TABLE_DUAL, ClopemaGraspFromTableDual)
        
        req = ClopemaGraspFromTableDualRequest()
        req.frame_id    = base_frame
        req.ik_link_1   = tip1
        req.ik_link_2   = tip2
        req.table_desk  = table_desk
        req.poses_1     = poses_1
        req.poses_2     = poses_2
        req.offset_minus = offset_minus
        req.offset_plus = offset_plus
        req.offset_table_minus = offset_table_minus
        req.offset_table_plus = offset_table_plus
        req.grasping_angle = grasping_angle
        req.final_poses_1 = final_poses_1
        req.final_poses_2 = final_poses_2

        tmp_robot_state = self._get_start_state()
        self.set_start_state(tmp_robot_state)
        res = srv.call(req)
        
        if res.error is "":
            return (res.joint_trajectories)
        else:
            rospy.logwarn("Unable to compute path! error code: %s", res.error)
            return None
