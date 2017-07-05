/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 10/20/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Time parametrization service.;
 */

#include <ros/ros.h>
#include <clopema_robot/TimeParametrization.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <clopema_robot/SetSpeed.h>

robot_model::RobotModelPtr kinematic_model;
float speed = 0.2;

/** \brief Return true if trajectory is supposed to controll grippers only */
bool is_gripper_trajectory(const std::vector<std::string>& jnames) {
    bool only_grip = true;//IF contains only gripper do nothing
    for(unsigned int i = 0; i < jnames.size(); ++i) {
        bool is_grip = jnames[i] == "r1_joint_grip";
        is_grip |= jnames[i] == "r2_joint_grip";
        if(!is_grip) {
            only_grip = false;
            break;
        }
    }
    return only_grip;
}

/** \brief Check whether all names \e inames are presented in the vector \e names */
bool are_all_inputs_presented(const std::vector<std::string>& inames, const std::vector<std::string>& names) {
    for(unsigned int i = 0; i < inames.size(); ++i) {
        std::string jn = inames[i];
        bool found = std::find(names.begin(), names.end(), jn) != names.end();
        if(!found)
            return false;
    }
    return true;
}

/** \brief Set speed callback method */
bool cb_speed(clopema_robot::SetSpeedRequest& req, clopema_robot::SetSpeedResponse& res) {
    if(req.speed > 0.21) {
        ROS_WARN_STREAM("Speed should not be larger then 0.2, bounding");
        speed = 0.2;
    } else if(req.speed > 0.2) {
        speed = 0.2;
    } else if(req.speed > 0.00001 && req.speed < 0.21) {
        speed = req.speed;
    } else if(fabs(req.speed + 1.0) < 0.001) { //equal to -1
    } else {
        ROS_WARN_STREAM("Invalid speed setting requested, not changing.");
    }
    res.current_speed = speed;
    return true;
}

void retimeTrajectory(trajectory_msgs::JointTrajectory& t, double factor) {
    double inv_factor = (1.0 / factor);    
    for(unsigned int i  = 0; i < t.points.size(); ++i) {
        t.points[i].time_from_start *= inv_factor;
        
        for(unsigned int j = 0; j < t.joint_names.size(); ++j) {
            t.points[i].velocities[j] *= factor;
        }
    }
}

/** \brief In case trajectory points have the same timestamp offset rest of trajectory */
void offset_timestamps(trajectory_msgs::JointTrajectory& t) {
    for(unsigned int i  = 1; i < t.points.size(); ++i) {
        double t_start = t.points[i-1].time_from_start.toSec();
        double t_end = t.points[i].time_from_start.toSec();
        if (fabs(t_end - t_start) > 0.001) {
            continue;
        }
        
        ROS_WARN_STREAM("Offseting the trajectory from position: " << i);
        //have the same time, offset rest of trajectory
        for(unsigned int j = i; j < t.points.size(); ++j) {
            t.points[j].time_from_start += ros::Duration(0.01);
        }
        i--; //check that point again it should be corrected
    }
}

/** \brief Add time parametrization to trajectory service callback method */
bool cb_parametrise(clopema_robot::TimeParametrizationRequest& req, clopema_robot::TimeParametrizationResponse& res) {

    if(is_gripper_trajectory(req.trajectory.joint_trajectory.joint_names)) {
        res.parametrized_trajectory = req.trajectory;
        res.success = true;
        return true;
    }

    robot_state::RobotState rs(kinematic_model);
    robot_state::robotStateMsgToRobotState(req.start_state, rs);

    robot_trajectory::RobotTrajectory rt(kinematic_model, "all");
    rt.setRobotTrajectoryMsg(rs, req.trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    if(!success) {
        res.success = false;
        return true;
    }

    moveit_msgs::RobotTrajectory tmsg;
    rt.getRobotTrajectoryMsg(tmsg);
    if(!are_all_inputs_presented(req.trajectory.joint_trajectory.joint_names, tmsg.joint_trajectory.joint_names)) {
        ROS_WARN_STREAM("Some joints are missing after parametrization, are you executing trajectory on the correct groups? (more info in debug stream)");
        ROS_DEBUG_STREAM(req.trajectory.joint_trajectory);
        ROS_DEBUG_STREAM(tmsg.joint_trajectory);
        res.success = false;
        return true;
    }
    offset_timestamps(tmsg.joint_trajectory);
    if(speed > 0 && speed < 0.21) {
        retimeTrajectory(tmsg.joint_trajectory, speed / 0.2);
    }
    offset_timestamps(tmsg.joint_trajectory);
    res.parametrized_trajectory = tmsg;
    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "time_parametrization_service");
    ros::NodeHandle node("~");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();

    ros::ServiceServer srv_parametrise = node.advertiseService("/move_group/add_time_parametrization", &cb_parametrise);
    ros::ServiceServer srv_speed = node.advertiseService("/move_group/set_robot_speed", &cb_speed);

    ros::spin();
    return 0;
}
