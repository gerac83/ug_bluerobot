/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: Nov 1, 2013
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 */

#include "cartesian_path_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>

move_group::MoveGroupCartesianPathServiceDual::MoveGroupCartesianPathServiceDual() :
        MoveGroupCapability("CartesianPathServiceDual"), display_computed_paths_(
                true) {
}

void move_group::MoveGroupCartesianPathServiceDual::initialize() {
    display_path_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
            planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10, true);
    cartesian_path_service_ = root_node_handle_.advertiseService(
            CARTESIAN_PATH_SERVICE_NAME,
            &MoveGroupCartesianPathServiceDual::computeService, this);
}

namespace {
bool isStateValid(const planning_scene::PlanningScene *planning_scene,
        const kinematic_constraints::KinematicConstraintSet *constraint_set,
        robot_state::RobotState *state,
        const robot_state::JointModelGroup *group, const double *ik_solution,
        const std::string& group_name) {
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    return (!planning_scene
            || !planning_scene->isStateColliding(*state, group_name, true))
            && (!constraint_set || constraint_set->decide(*state).satisfied);
}
}

bool move_group::MoveGroupCartesianPathServiceDual::transform_poses(
    EigenSTL::vector_Affine3d& waypoints,
    std::vector<geometry_msgs::Pose> waypoints_in,
    clopema_moveit::GetCartesianPathDual::Request& req,
    const std::string& link_name) {
    
    const std::string &default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();    
    bool no_transform = req.header.frame_id.empty()
        || robot_state::Transforms::sameFrame(req.header.frame_id, default_frame)
        || robot_state::Transforms::sameFrame(req.header.frame_id, link_name);
        
        
    //Get start state
    Eigen::Affine3d f = Eigen::Affine3d::Identity();
    if (!no_transform) {
        robot_state::RobotState start_state =
        planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
        robot_state::robotStateMsgToRobotState(req.start_state, start_state);
        f = start_state.getGlobalLinkTransform(req.header.frame_id);
    }
    
    for (std::size_t i = 0; i < waypoints_in.size(); ++i) {
        if (no_transform) {
            tf::poseMsgToEigen(waypoints_in[i], waypoints[i]);
        } else {
            //Transform to the "default frame" from "frame_id"
            Eigen::Affine3d ep;
            tf::poseMsgToEigen(waypoints_in[i], ep);
            waypoints[i] = f * ep;
        }
    }
    return true;
}

bool move_group::MoveGroupCartesianPathServiceDual::computeService(
        clopema_moveit::GetCartesianPathDual::Request &req,
        clopema_moveit::GetCartesianPathDual::Response &res) {
    ROS_INFO("Received request to compute Cartesian path");
    context_->planning_scene_monitor_->updateFrameTransforms();

    robot_state::RobotState start_state =
            planning_scene_monitor::LockedPlanningSceneRO(
                    context_->planning_scene_monitor_)->getCurrentState();
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);

    std::string link_name_1 = req.link_name_1;
    std::string link_name_2 = req.link_name_2;
    if (link_name_1.empty() || link_name_2.empty()) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
        return true;
    }

    if (req.waypoints_1.size() != req.waypoints_2.size()) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
        ROS_ERROR_STREAM("Number of poses not equal");
        return true;
    }

    const robot_model::JointModelGroup *jmg1, *jmg2, *jmg;
    { //Get kinematics-allowed subgroups, if there are no subgroup return with error
        if(req.group_name != "arms" && req.group_name != "xtions") {
            ROS_ERROR_STREAM("Group have to be set to arms or xtions not to: " << req.group_name);
            res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            return true;
        }
        jmg = start_state.getJointModelGroup(req.group_name);
        std::vector<std::string> subgroups = jmg->getSubgroupNames();
        for (unsigned int i = 0; i < subgroups.size(); ++i) {
            const robot_model::JointModelGroup *jg =
                    start_state.getJointModelGroup(subgroups[i]);
            if (jg->canSetStateFromIK(link_name_1)) {
                jmg1 = jg;
            }
            if (jg->canSetStateFromIK(link_name_2)) {
                jmg2 = jg;
            }
        }
        if (!jmg1 || !jmg2) {
            res.error_code.val =
                    moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            return true;
        }
    }

    EigenSTL::vector_Affine3d waypoints_1(req.waypoints_1.size());
    EigenSTL::vector_Affine3d waypoints_2(req.waypoints_2.size());
    if (!transform_poses(waypoints_1, req.waypoints_1, req, link_name_1)) {
        res.error_code.val =
                moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
        return true;
    }
    if (!transform_poses(waypoints_2, req.waypoints_2, req, link_name_2)) {
        res.error_code.val =
                moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
        return true;
    }

    if (req.max_step < std::numeric_limits<double>::epsilon()) {
        ROS_ERROR(
                "Maximum step to take between consecutive configrations along Cartesian path was not specified (this value needs to be > 0)");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return true;
    }

    if (waypoints_1.empty() || waypoints_2.empty()) {
        return true;
    }

    /**Collision and constraints checking function. */
    robot_state::GroupStateValidityCallbackFn constraint_fn;
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    boost::scoped_ptr<kinematic_constraints::KinematicConstraintSet> kset;
    if (req.avoid_collisions
            || !kinematic_constraints::isEmpty(req.path_constraints)) {
        ls.reset(
                new planning_scene_monitor::LockedPlanningSceneRO(
                        context_->planning_scene_monitor_));
        kset.reset(
                new kinematic_constraints::KinematicConstraintSet(
                        (*ls)->getRobotModel()));
        kset->add(req.path_constraints, (*ls)->getTransforms());
        constraint_fn =
                boost::bind(&isStateValid,
                        req.avoid_collisions ?
                                static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get() :
                                NULL, kset->empty() ? NULL : kset.get(), _1, _2,
                        _3, req.group_name);
    }

    bool global_frame = !robot_state::Transforms::sameFrame(link_name_1,
            req.header.frame_id);
    ROS_INFO(
            "Attempting to follow %u waypoints for link '%s' using a step of %lf m and jump threshold %lf (in %s reference frame)", (unsigned int)waypoints_1.size(), link_name_1.c_str(), req.max_step, req.jump_threshold, global_frame ? "global" : "link");
    ROS_INFO(
            "Attempting to follow %u waypoints for link '%s' using a step of %lf m and jump threshold %lf (in %s reference frame)", (unsigned int)waypoints_2.size(), link_name_2.c_str(), req.max_step, req.jump_threshold, global_frame ? "global" : "link");
    std::vector<robot_state::RobotStatePtr> traj;

    robot_state::RobotStatePtr rs(new RobotState(start_state));
    res.fraction = computeCartesianPath(rs, jmg, traj,
            start_state.getLinkModel(link_name_1), jmg1,
            start_state.getLinkModel(link_name_2), jmg2, waypoints_1,
            waypoints_2, global_frame, req.max_step, req.jump_threshold,
            constraint_fn);

    /*
     res.fraction = start_state.computeCartesianPath(jmg1, traj,
     start_state.getLinkModel(link_name_1), waypoints_1, global_frame,
     req.max_step, req.jump_threshold, constraint_fn);
     */
    robot_state::robotStateToRobotStateMsg(start_state, res.start_state);

    robot_trajectory::RobotTrajectory rt(
            context_->planning_scene_monitor_->getRobotModel(), req.group_name);
    for (std::size_t i = 0; i < traj.size(); ++i)
        rt.addSuffixWayPoint(traj[i], 0.2);
    rt.getRobotTrajectoryMsg(res.solution);
    ROS_INFO(
            "Computed Cartesian path with %u points (followed %lf%% of requested trajectory)", (unsigned int)traj.size(), res.fraction * 100.0);
    if (display_computed_paths_ && rt.getWayPointCount() > 0) {
        moveit_msgs::DisplayTrajectory disp;
        disp.model_id =
                context_->planning_scene_monitor_->getRobotModel()->getName();
        disp.trajectory.resize(1, res.solution);
        robot_state::robotStateToRobotStateMsg(rt.getFirstWayPoint(),
                disp.trajectory_start);
        display_path_.publish(disp);
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

double move_group::MoveGroupCartesianPathServiceDual::computeCartesianPath(
        const RobotStatePtr& rs, const JointModelGroup *group,
        std::vector<RobotStatePtr> &traj, const LinkModel *link1,
        const JointModelGroup *jmg1, const LinkModel *link2,
        const JointModelGroup *jmg2, const Eigen::Affine3d &target1,
        const Eigen::Affine3d &target2, bool global_reference_frame,
        double max_step, double jump_threshold,
        const GroupStateValidityCallbackFn &validCallback,
        const kinematics::KinematicsQueryOptions &options) {

    const std::vector<const JointModel*> &cjnt =
            group->getContinuousJointModels();
    // make sure that continuous joints wrap
    for (std::size_t i = 0; i < cjnt.size(); ++i)
        rs->enforceBounds(cjnt[i]);

    // this is the Cartesian pose we start from, and we move in the direction indicated
    Eigen::Affine3d start_pose1 = rs->getGlobalLinkTransform(link1);
    Eigen::Affine3d start_pose2 = rs->getGlobalLinkTransform(link2);

    // the target can be in the local reference frame (in which case we rotate it)
    Eigen::Affine3d rotated_target1 =
            global_reference_frame ? target1 : start_pose1 * target1;
    Eigen::Affine3d rotated_target2 =
            global_reference_frame ? target2 : start_pose2 * target2;

    bool test_joint_space_jump = jump_threshold > 0.0;

    // decide how many steps we will need for this trajectory
    double distance; //pick larger distance
    {
        double distance1 = (rotated_target1.translation()
                - start_pose1.translation()).norm();
        double distance2 = (rotated_target2.translation()
                - start_pose2.translation()).norm();
        if (distance1 > distance2)
            distance = distance1;
        else
            distance = distance2;
    }
    unsigned int steps = (test_joint_space_jump ? 5 : 1)
            + (unsigned int) floor(distance / max_step);

    traj.clear();
    traj.push_back(RobotStatePtr(new RobotState(*rs)));

    std::vector<double> dist_vector;
    double total_dist = 0.0;

    double last_valid_percentage = 0.0;
    Eigen::Quaterniond start_quaternion1(start_pose1.rotation());
    Eigen::Quaterniond target_quaternion1(rotated_target1.rotation());
    Eigen::Quaterniond start_quaternion2(start_pose2.rotation());
    Eigen::Quaterniond target_quaternion2(rotated_target2.rotation());

    bool dist_not_valid = false;
    for (unsigned int i = 0; i <= steps; ++i) {
        double percentage = (double) i / (double) steps;

        Eigen::Isometry3d pose1(
                start_quaternion1.slerp(percentage, target_quaternion1));
        pose1.translation() = percentage * rotated_target1.translation()
                + (1 - percentage) * start_pose1.translation();

        Eigen::Isometry3d pose2(
                start_quaternion2.slerp(percentage, target_quaternion2));
        pose2.translation() = percentage * rotated_target2.translation()
                + (1 - percentage) * start_pose2.translation();

        if (!rs->setFromIK(jmg1, pose1, link1->getName(), 1, 0.0, validCallback,
                options)) {
            ROS_ERROR_STREAM(link1->getName() << ": Unable to set from IK");
            break;
        }
        if (!rs->setFromIK(jmg2, pose2, link2->getName(), 1, 0.0, validCallback,
                options)) {
            ROS_ERROR_STREAM(link2->getName() << ": Unable to set from IK");
            break;
        }

        traj.push_back(RobotStatePtr(new RobotState(*rs)));
        // compute the distance to the previous point (infinity norm)
        if (test_joint_space_jump) {
            double dist_prev_point = traj.back()->distance(
                    *traj[traj.size() - 2], group);
            dist_vector.push_back(dist_prev_point);
            total_dist += dist_prev_point;
            
            //check each joint jump is not larger than 1.0rad
            for(std::size_t i = 0; i < traj.back()->getVariableCount(); ++i) {
                double d = fabs(traj.back()->getVariablePosition(i) - traj[traj.size()-2]->getVariablePosition(i));
                if(d > 1.0) {
                    traj.pop_back();
                    ROS_WARN_STREAM("IKT jump higher than 1.0 RAD");
                    dist_not_valid = true;
                    break;
                }
            }
            if(dist_not_valid)
                break;
        }
        last_valid_percentage = percentage;
    }

    if(dist_not_valid) { //if IKT Jump detected do not recompute same things using MoveIt...
        return last_valid_percentage;
    }

    if (test_joint_space_jump) {
        // compute the average distance between the states we looked at
        double thres = jump_threshold * (total_dist / (double) dist_vector.size());
        
        for (std::size_t i = 0; i < dist_vector.size(); ++i) {
            if (dist_vector[i] > thres) {
                CONSOLE_BRIDGE_logDebug("Truncating Cartesian path due to detected jump in joint-space distance");
                last_valid_percentage = (double) i / (double) steps;
                traj.resize(i);
                ROS_INFO_STREAM("Threshold: " << thres);
                for (std::size_t iiii = 0; iiii < dist_vector.size(); ++iiii) {
                    ROS_INFO_STREAM(dist_vector[iiii]);
                }
                break;
            }
        }
    }

    return last_valid_percentage;
}

double move_group::MoveGroupCartesianPathServiceDual::computeCartesianPath(
        const RobotStatePtr& rs, const JointModelGroup *group,
        std::vector<RobotStatePtr> &traj, const LinkModel *link1,
        const JointModelGroup *jmg1, const LinkModel *link2,
        const JointModelGroup *jmg2,
        const EigenSTL::vector_Affine3d &waypoints1,
        const EigenSTL::vector_Affine3d &waypoints2,
        bool global_reference_frame, double max_step, double jump_threshold,
        const GroupStateValidityCallbackFn &validCallback,
        const kinematics::KinematicsQueryOptions &options) {
    double percentage_solved = 0.0;
    for (std::size_t i = 0; i < waypoints1.size(); ++i) {
        std::vector<RobotStatePtr> waypoint_traj;
        double wp_percentage_solved = computeCartesianPath(rs, group,
                waypoint_traj, link1, jmg1, link2, jmg2, waypoints1[i],
                waypoints2[i], global_reference_frame, max_step, jump_threshold,
                validCallback);
        if (fabs(wp_percentage_solved - 1.0)
                < std::numeric_limits<double>::epsilon()) {
            percentage_solved = (double) (i + 1) / (double) waypoints1.size();
            traj.insert(traj.end(), waypoint_traj.begin(), waypoint_traj.end());
        } else {
            percentage_solved += wp_percentage_solved
                    / (double) waypoints1.size();
            traj.insert(traj.end(), waypoint_traj.begin(), waypoint_traj.end());
            break;
        }
    }
    return percentage_solved;
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupCartesianPathServiceDual,
        move_group::MoveGroupCapability)


