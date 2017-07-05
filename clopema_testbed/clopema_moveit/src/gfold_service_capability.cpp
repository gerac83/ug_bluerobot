/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: Nov 1, 2013
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 */

#include "gfold_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include "cartesian_path_service_capability.h"

move_group::GFoldService::GFoldService() :
    MoveGroupCapability("GFoldService"), display_computed_paths_(true) {
}

void move_group::GFoldService::initialize() {
    display_path_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
                        planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10, true);
    gfold_service_ = root_node_handle_.advertiseService(GFOLD_SERVICE_NAME,
                     &GFoldService::computeService, this);
}

bool move_group::GFoldService::computeService(
    clopema_moveit::GFold::Request& req,
    clopema_moveit::GFold::Response& res) {
    ROS_INFO("Received request to compute gfold");
    context_->planning_scene_monitor_->updateFrameTransforms();

    robot_state::RobotState start_state =
        planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);

    bool inverse = false;
    if(boost::starts_with(req.ik_link_1, "r2_") || boost::starts_with(req.ik_link_2, "r1_")) {
        inverse = true;
    }
    // Define problem from the request
    Problem problem;
    problem.header.frame_id = req.frame_id;
    int arm = -1;
    if(!req.ik_link_1.empty() && !req.ik_link_2.empty()) {
        arm = 3;
        problem.final_pose_1_link = req.ik_link_1;
        problem.final_pose_2_link = req.ik_link_2;
    } else if(!req.ik_link_1.empty() && req.ik_link_2.empty()) {
        arm = 1;
        problem.final_pose_1_link = req.ik_link_1;
        problem.final_pose_2_link = req.ik_link_1;
    } else if(req.ik_link_1.empty() && !req.ik_link_2.empty()) {
        arm = 2;
        problem.final_pose_1_link = req.ik_link_2;
        problem.final_pose_2_link = req.ik_link_2;
    } else {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
        return true;
    }

    Eigen::Affine3d to_base = start_state.getGlobalLinkTransform(
                                  problem.header.frame_id).inverse();
    Eigen::Affine3d e = to_base
                        * start_state.getGlobalLinkTransform(problem.final_pose_1_link);
    tf::poseEigenToMsg(e, problem.start_pose_1);
    tf::poseEigenToMsg(e, problem.final_pose_1);
    e = to_base * start_state.getGlobalLinkTransform(problem.final_pose_2_link);
    tf::poseEigenToMsg(e, problem.start_pose_2);
    tf::poseEigenToMsg(e, problem.final_pose_2);

    problem.final_pose_1.position = req.final_point_1;
    problem.final_pose_2.position = req.final_point_2;
    if(arm == 1)
        problem.final_pose_2.position = req.final_point_1;
    else if(arm == 2)
        problem.final_pose_1.position = req.final_point_2;

    std::string r1_tip = "r1_tip_link";
    std::string r2_tip = "r2_tip_link";
    if(inverse) {
        r1_tip = "r2_tip_link";
        r2_tip = "r1_tip_link";
    }
    if(arm == 1) {
        r2_tip = r1_tip;
    } else if(arm == 2) {
        r1_tip = r2_tip;
    }

    transform_to_tip(start_state, problem.final_pose_1,
                     problem.final_pose_1_link, r1_tip);
    transform_to_tip(start_state, problem.start_pose_1,
                     problem.final_pose_1_link, r1_tip);
    transform_to_tip(start_state, problem.final_pose_2,
                     problem.final_pose_2_link, r2_tip);
    transform_to_tip(start_state, problem.start_pose_2,
                     problem.final_pose_2_link, r2_tip);
    problem.final_pose_1_link = r1_tip;
    problem.final_pose_2_link = r2_tip;
    if(!compute_gfold_poses(problem)) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return true;
    }

    if(!interpolate_gfold(problem, start_state, res.joint_trajectory, arm, req.allow_collision_1, req.allow_collision_2)) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return true;
    }

    robot_state::robotStateToRobotStateMsg(start_state, res.start_state);
    if(display_computed_paths_) {
        moveit_msgs::DisplayTrajectory disp;
        disp.model_id =
            context_->planning_scene_monitor_->getRobotModel()->getName();
        disp.trajectory.resize(1);
        disp.trajectory[0].joint_trajectory = res.joint_trajectory;
        disp.trajectory_start = res.start_state;
        display_path_.publish(disp);
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

void move_group::GFoldService::transform_to_tip(
    const robot_state::RobotState& rs, geometry_msgs::Pose& pose,
    const std::string& frame, const std::string& tip) {
    Eigen::Affine3d ee;
    tf::poseMsgToEigen(pose, ee);
    Eigen::Affine3d e = ee
                        * (rs.getGlobalLinkTransform(tip).inverse()
                           * rs.getGlobalLinkTransform(frame)).inverse();
    tf::poseEigenToMsg(e, pose);
}

void move_group::GFoldService::transform_to_tip(
    const robot_state::RobotState& rs, geometry_msgs::Point& point,
    const std::string& frame, const std::string& tip) {
    geometry_msgs::Pose pose;
    pose.position = point;
    pose.orientation.w = 1;
    transform_to_tip(rs, pose, frame, tip);
    point = pose.position;
}

bool move_group::GFoldService::compute_gfold_poses(Problem& prob) {
    tf::pointMsgToKDL(prob.start_pose_1.position, prob.g11);
    tf::pointMsgToKDL(prob.start_pose_2.position, prob.g21);
    tf::pointMsgToKDL(prob.final_pose_1.position, prob.g15);
    tf::pointMsgToKDL(prob.final_pose_2.position, prob.g25);

    Vector vg1 = prob.g15 - prob.g11;
    Vector vg2 = prob.g25 - prob.g21;
    Vector n(0, 0, 1); //assumption that gravity is pointing in -this direction
    double d1 = vg1.Normalize() / 2.0;
    double d2 = vg2.Normalize() / 2.0;

    double eps = 0.001;
    if(fabs(d1 - d2) < eps) {
        prob.g12 = prob.g11;
        prob.g22 = prob.g21;
    } else if(d1 > d2) {
        prob.g12 = prob.g11 + (d1 - d2) * vg1 + n * (d1 - d2);
        prob.g22 = prob.g21;
    } else if(d2 > d1) {
        prob.g12 = prob.g11;
        prob.g22 = prob.g21 + (d2 - d1) * vg2 + n * (d2 - d1);
    }

    prob.g13 = prob.g11 + d1 * vg1 + n * d1;
    prob.g23 = prob.g21 + d2 * vg2 + n * d2;

    if(fabs(d1 - d2) < eps) {
        prob.g14 = prob.g13;
        prob.g24 = prob.g23;
    } else if(d1 > d2) {
        prob.g14 = prob.g11 + (d1 + d2) * vg1 + n * (d1 - d2);
        prob.g24 = prob.g25;
    } else if(d2 > d1) {
        prob.g14 = prob.g15;
        prob.g24 = prob.g21 + (d2 + d1) * vg2 + n * (d2 - d1);
    }

    return true;
}

bool move_group::GFoldService::interpolate_gfold(const Problem& problem,
        const robot_state::RobotState& state,
        trajectory_msgs::JointTrajectory& traj, int arm,
        const std::vector<std::string>& allow_collision_1, const std::vector<std::string>& allow_collision_2) {

    std::vector<Pose> waypoints1, waypoints2;
    Pose pose1, pose2;
    pose1.orientation = problem.start_pose_1.orientation;
    pose2.orientation = problem.start_pose_2.orientation;

    if(!KDL::Equal(problem.g11, problem.g12, 0.001)
            || !KDL::Equal(problem.g21, problem.g22, 0.001)) {
        tf::pointKDLToMsg(problem.g12, pose1.position);
        waypoints1.push_back(pose1);
        tf::pointKDLToMsg(problem.g22, pose2.position);
        waypoints2.push_back(pose2);
    }

    tf::pointKDLToMsg(problem.g13, pose1.position);
    waypoints1.push_back(pose1);
    tf::pointKDLToMsg(problem.g23, pose2.position);
    waypoints2.push_back(pose2);

    if(!KDL::Equal(problem.g13, problem.g14, 0.001)
            || !KDL::Equal(problem.g23, problem.g24, 0.001)) {
        tf::pointKDLToMsg(problem.g14, pose1.position);
        waypoints1.push_back(pose1);
        tf::pointKDLToMsg(problem.g24, pose2.position);
        waypoints2.push_back(pose2);
    }

    tf::pointKDLToMsg(problem.g15, pose1.position);
    waypoints1.push_back(pose1);
    tf::pointKDLToMsg(problem.g25, pose2.position);
    waypoints2.push_back(pose2);

    clopema_moveit::GetCartesianPathDual::Request req;
    clopema_moveit::GetCartesianPathDual::Response res;

    robot_state::robotStateToRobotStateMsg(state, req.start_state);
    req.group_name = "arms";
    req.header.frame_id = problem.header.frame_id;
    req.header.stamp = ros::Time::now();
    req.waypoints_1 = waypoints1;
    req.link_name_1 = problem.final_pose_1_link;
    req.link_name_2 = problem.final_pose_2_link;
    req.waypoints_2 = waypoints2;
    req.max_step = 0.01;
    req.jump_threshold = 1.3;
    req.avoid_collisions = false;

    MoveGroupCartesianPathServiceDual dualsrv;
    dualsrv.setContext(context_);
    dualsrv.computeService(req, res);

    double d = 0;
    if(res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        if(allow_collision_1.size() != allow_collision_2.size()) {
            d = -1.0;
            ROS_ERROR_STREAM("Allow collision pairs have not equal sizes");
        } else {
            planning_scene::PlanningScenePtr ps;
            ps = planning_scene::PlanningScene::clone(planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_));
    
            //check for collision
            bool valid = false;
            for(unsigned int i = 0; i < allow_collision_1.size(); ++i) {
                ps->getAllowedCollisionMatrixNonConst().setEntry(allow_collision_1[i], allow_collision_2[i], true);
            }
            valid = ps->isPathValid(res.start_state, res.solution);
            for(unsigned int i = 0; i < allow_collision_1.size(); ++i) {
                ps->getAllowedCollisionMatrixNonConst().setEntry(allow_collision_1[i], allow_collision_2[i], false);
            }
            
            if(!valid) {
                d = -1.0;
            } else {
                d = res.fraction;
                traj = res.solution.joint_trajectory;
            }
        }
    } else
        d = -1.0;

    return (fabs(d - 1.0) < 0.0001);
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::GFoldService, move_group::MoveGroupCapability)


