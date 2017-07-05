import roslib; roslib.load_manifest('clopema_smach')
import rospy
import smach, trajectory_msgs.msg, copy
from smach_ros           import ServiceState
from smach import State
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory, RobotState
from moveit_commander import MoveGroupCommander
from trajectory_msgs.msg import JointTrajectory
from clopema_robot.services import set_drive_power

from moveit_msgs.srv import ExecuteKnownTrajectory
from moveit_msgs.srv._ExecuteKnownTrajectory import ExecuteKnownTrajectoryRequest

__all__ = ['TrajectoryVisualizeState', 'TrajectoryExecuteState',
           'TrajectoryReverseState', 'TrajectoryMergeState',
           'SetServoPowerOffState']

class TrajectoryVisualizeState(State):
    """
    Visualize trajectory in the RVIZ
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                                input_keys=['trajectory'])
        self.trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory)
        
    def execute(self, userdata):
        if (self.trajectory_publisher.get_num_connections() < 1):
            rospy.sleep(0.5)
        msg = DisplayTrajectory()
        msg.model_id = 'clopema'
        if isinstance(userdata.trajectory, (list, tuple)):
            for traj in userdata.trajectory:
                rtraj = RobotTrajectory()
                rtraj.joint_trajectory = traj;
                msg.trajectory.append(rtraj)            
        else:
            msg.trajectory.append(RobotTrajectory())
            msg.trajectory[0].joint_trajectory = userdata.trajectory
        self.trajectory_publisher.publish(msg)
        return 'succeeded'

class TrajectoryExecuteState(State): 
    """
    Execute trajectory, ie. move robot along defined trajectory
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['trajectory'])
        rospy.wait_for_service('/execute_kinematic_path')
        self.execute_proxy = rospy.ServiceProxy('/execute_kinematic_path', ExecuteKnownTrajectory)
        
    def execute(self, userdata):
        try:
            if isinstance(userdata.trajectory, (list, tuple)):
                for traj in userdata.trajectory:
                    req = ExecuteKnownTrajectoryRequest()
                    req.trajectory = RobotTrajectory()
                    req.trajectory.joint_trajectory = traj
                    req.wait_for_execution = True
                    res = self.execute_proxy(req)
                    if res.error_code.val == res.error_code.SUCCESS:
                        rospy.loginfo('Part of the trajectory was executed successfully')
                    else:
                        print 'Error code value: ', res.error_code.val
                        return 'aborted'
            else:   
                req = ExecuteKnownTrajectoryRequest()
                req.trajectory = RobotTrajectory()
                req.trajectory.joint_trajectory = userdata.trajectory
                req.wait_for_execution = True
                res = self.execute_proxy(req)
                if res.error_code.val == res.error_code.SUCCESS:
                    return 'succeeded'
                else:
                    print 'Error code value: ', res.error_code.val
                    return 'aborted'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return 'succeeded'
    

class TrajectoryExecuteSingleState(ServiceState):
    """
    Execute trajectory, ie. move robot along defined trajectory
    """
    def __init__(self):
        ServiceState.__init__(self, '/execute_kinematic_path', ExecuteKnownTrajectory,
                                    request_cb=self._req_cb,
                                    input_keys=['trajectory'],
                                    response_cb=self._res_cb, response_slots=['error_code'])
    def _req_cb(self, userdata, req):
        req.trajectory = RobotTrajectory()
        req.trajectory.joint_trajectory = userdata.trajectory
        req.wait_for_execution = True
    
    def _res_cb(self, userdata, res):
        if res.error_code.val == res.error_code.SUCCESS:
            return 'succeeded'
        else:
            print 'Error code value: ', res.error_code.val
            return 'aborted'
    
        
class SetServoPowerOffState(smach.State):
    """
    Set servo power off, force is set to true
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        set_drive_power(power=False)
        return 'succeeded'
        
class TrajectoryMergeState(smach.State):
    """
    Merge two trajectory together
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['trajectory_1', 'trajectory_2'],
                             output_keys=['trajectory'])
        
    def execute(self, userdata):
        merged = trajectory_msgs.msg.JointTrajectory()
        ta = userdata.trajectory_1
        tb = userdata.trajectory_2

        if not tb.points:
            userdata.trajectory = userdata.trajectory_1
            return 'succeeded'
        
        if not ta.points:
            userdata.trajectory = userdata.trajectory_2
            return 'succeeded'

        merged.header = ta.header
        
        # create joint_names as an 'unique' copy
        merged.joint_names = copy.deepcopy(ta.joint_names)
        for joint_name in tb.joint_names:
            exist = False
            for name in merged.joint_names:
                if name == joint_name :
                    exist = True
                    break
            if not exist:
                merged.joint_names.append(joint_name)
        
        # first set p to contain initial values from second trajectory
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0] * len(merged.joint_names)
        p.velocities = [0.0] * len(merged.joint_names)
        p.accelerations = [0.0] * len(merged.joint_names)
        
        for name in tb.joint_names:
            p.positions[merged.joint_names.index(name)] = tb.points[0].positions[tb.joint_names.index(name)]
        
        # append ta to the merged trajectory
        for point in ta.points:
            for name in ta.joint_names:
                p.positions[merged.joint_names.index(name)] = point.positions[ta.joint_names.index(name)]
            merged.points.append(copy.deepcopy(p))
        
        # set p to containt last value from first trajectory
        for name in ta.joint_names:
            p.positions[merged.joint_names.index(name)] = ta.points[len(ta.points) - 1].positions[ta.joint_names.index(name)]
        
        # append ta to the merged trajectory
        for point in tb.points:
            for name in tb.joint_names:
                p.positions[merged.joint_names.index(name)] = point.positions[tb.joint_names.index(name)]
            merged.points.append(copy.deepcopy(p))
        
        userdata.trajectory = merged
        return 'succeeded'


class TrajectoryReverseState(smach.State):
    """
    Reverse trajectory
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['trajectory_in'],
                             output_keys=['trajectory'])
        
    def execute(self, userdata):
        traj = copy.deepcopy(userdata.trajectory_in)
        traj.points.reverse()
        userdata.trajectory = traj
        return 'succeeded'

