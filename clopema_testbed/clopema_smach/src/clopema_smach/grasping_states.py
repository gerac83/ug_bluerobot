"""
Grasping states, grasp, grasp_from_table, etc..
"""
import roslib; roslib.load_manifest('clopema_smach')
import rospy, smach, PyKDL, tf_conversions, geometry_msgs, copy

from smach import State
from smach_ros      import ServiceState
from clopema_smach  import *
from clopema_moveit.srv import ClopemaGraspIt, ClopemaGraspFromTable

class PlanGraspItState(ServiceState):
    """
    Plan trajectories for grasping object from the specified pose (goals).
    Goals are ordered geometry_msgs/Pose and first valid is chosen.
    \param poses - array of geometry_msgs/Pose, z axis pointing to the grasping direction
    \param frame_id - base frame of the poses
    \param ik_link - link for which poses are specified
    \param offset_minus, offset_plus - offsets in z-axis
    \parma trajectories - output trajectories including grippers command
    """
    def __init__(self):
        service = '/clopema_planner/grasp_it'
        ServiceState.__init__(self, service, ClopemaGraspIt,
                              request_slots=['ik_link', 'poses', 'frame_id', 'offset_minus', 'offset_plus'],
                              response_cb=self.result_cb,
                              output_keys=['trajectory'])
        
    def result_cb(self, userdata, response):
        if response.error == "":
            userdata.trajectory = response.joint_trajectories
            return 'succeeded'
        else:
            return 'aborted'
        
class PlanGraspFromTable(ServiceState):
    def __init__(self):
        service = '/clopema_planner/grasp_from_table'
        ServiceState.__init__(self, service, ClopemaGraspFromTable,
                              request_slots=['ik_link', 'poses', 'frame_id', 'table_desk', 'offset_minus',
                                              'offset_plus', 'offset_table_plus', 'offset_table_minus', 
                                              'grasping_angle'],
                              response_cb=self.result_cb,
                              output_keys=['trajectory'])
        
    def result_cb(self, userdata, response):
        if response.error == "":
            userdata.trajectory = response.joint_trajectories
            return 'succeeded'
        else:
            return 'aborted'
