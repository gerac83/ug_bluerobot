#include <virtual_robot/VirtualGripper.h>
#include <sensor_msgs/JointState.h>

VirtualGripper::VirtualGripper():
    node_("~"),
    as_(node_, "command", boost::bind(&VirtualGripper::cb_execute_goal, this, _1), false) {

    if(!node_.getParam("joint_name", joint_name)) {
        ROS_ERROR_STREAM("Joint Name is not specified, stoping");
        return;
    }
    node_.param<double>("default_position", joint_position, 0.0);

    pub_joint_states = node_.advertise<sensor_msgs::JointState>("joint_states", 1);
    timer_joint_states = node_.createTimer(ros::Duration(0.05), &VirtualGripper::cb_timer_joint_states, this);
    as_.start();
}


void VirtualGripper::cb_execute_goal(const control_msgs::GripperCommandGoalConstPtr& goal) {
    joint_position = goal->command.position;
    as_.setSucceeded();
}

void VirtualGripper::cb_timer_joint_states(const ros::TimerEvent& te) {
    sensor_msgs::JointState jmsg;
    jmsg.header.frame_id = joint_name;
    jmsg.header.stamp = ros::Time::now();
    jmsg.name.push_back(joint_name);
    jmsg.position.push_back(joint_position);
    jmsg.velocity.resize(1, 0.0);
    pub_joint_states.publish(jmsg);
}
