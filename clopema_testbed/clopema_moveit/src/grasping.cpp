/***
 * Grasping services.
 */

#include <ros/ros.h>
#include <clopema_moveit/ClopemaGraspIt.h>
#include <clopema_moveit/ClopemaGraspFromTable.h>
#include <clopema_moveit/ClopemaGraspFromTableDual.h>
#include <clopema_moveit/GetCartesianPathDual.h>
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/foreach.hpp>

boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> r1_arm_group;
boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> r2_arm_group;
boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> arms_group;
boost::shared_ptr<ros::Publisher> pub_traj;
planning_scene_monitor::PlanningSceneMonitorPtr ps;

void change_collision_with_table(bool enable_collision, const std::string& table_desk) {
    //allow or disable collision between grippers and the table desks
    planning_scene_monitor::LockedPlanningSceneRW(ps)->getAllowedCollisionMatrixNonConst().setEntry("r1_gripper", table_desk, enable_collision);
    planning_scene_monitor::LockedPlanningSceneRW(ps)->getAllowedCollisionMatrixNonConst().setEntry("r2_gripper", table_desk, enable_collision);
}

trajectory_msgs::JointTrajectory grip_traj(const std::string& group, bool open) {
    trajectory_msgs::JointTrajectory traj;
    int num_joints = 1;
    if(group == "r1_arm") {
        traj.joint_names.push_back("r1_joint_grip");
    } else if(group == "r2_arm") {
        traj.joint_names.push_back("r2_joint_grip");
    } else if(group == "arms") {
        traj.joint_names.push_back("r1_joint_grip");
        traj.joint_names.push_back("r2_joint_grip");
        num_joints = 2;
    }
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(0);
    traj.points[0].accelerations.resize(num_joints);
    traj.points[0].velocities.resize(num_joints);

    if(open)
        traj.points[0].positions.resize(num_joints, 1.0);
    else
        traj.points[0].positions.resize(num_joints, 0.0);

    return traj;
}

trajectory_msgs::JointTrajectory open_grip_traj(const std::string& group) {
    return grip_traj(group, true);
}

trajectory_msgs::JointTrajectory close_grip_traj(const std::string& group) {
    return grip_traj(group, false);
}

/** \brief Resolve planning group based on ik link. */
std::string resolve_planning_group(const std::string& ik_link_name) {
    std::string group = "";
    if(strncmp(ik_link_name.c_str(), "r2", strlen("r2")) == 0) {
        group = "r2_arm";
    } else if(strncmp(ik_link_name.c_str(), "r1", strlen("r1")) == 0) {
        group = "r1_arm";
    } else if(strncmp(ik_link_name.c_str(), "xtion1", strlen("xtion1")) == 0) {
        group = "r1_xtion";
    } else if(strncmp(ik_link_name.c_str(), "xtion2", strlen("xtion2")) == 0) {
        group = "r2_xtion";
    }
    return group;
}

geometry_msgs::Pose pre_grasp(const geometry_msgs::Pose& pose, double distance) {
    geometry_msgs::Pose out;
    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose, e);
    e.translate(Eigen::Vector3d(0, 0, -distance));
    tf::poseEigenToMsg(e, out);
    return out;
}

geometry_msgs::Pose post_grasp(const geometry_msgs::Pose& pose, double distance) {
    geometry_msgs::Pose out;
    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose, e);
    e.translate(Eigen::Vector3d(0, 0, distance));
    tf::poseEigenToMsg(e, out);
    return out;
}

void transform_to_tip_link(geometry_msgs::Pose& pose,
                           boost::shared_ptr<moveit::planning_interface::MoveGroupInterface>& g,
                           const std::string& ik_link,
                           const robot_state::RobotState& rs) {

    if(g->getEndEffectorLink() == ik_link)
        return;

    Eigen::Affine3d gripper = rs.getFrameTransform(g->getEndEffectorLink()).inverse() * rs.getFrameTransform(ik_link);
    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose, e);
    e = e * gripper.inverse();
    tf::poseEigenToMsg(e, pose);
}

/** @brief Project pose to the table - result pose will be in table frame. */
bool project_to_the_table(const geometry_msgs::Pose& pose,
                          const robot_state::RobotState& rs,
                          const std::string& ik_link, const std::string& table_id,
                          const std::string& frame_id, geometry_msgs::Pose& out_pose) {

    if(!rs.knowsFrameTransform(table_id)) {
        ROS_ERROR_STREAM("Do not known table id: " << table_id);
        return false;
    }
    Eigen::Affine3d e = Eigen::Affine3d::Identity();
    tf::poseMsgToEigen(pose, e);
    Eigen::Affine3d t_table = rs.getFrameTransform(table_id);
    Eigen::Affine3d t_base = rs.getFrameTransform(frame_id);
    e = t_table.inverse() * t_base * e;
    e.translation()[2] = 0.0;

    Eigen::Affine3d e_moved = e;
    e_moved.translate(Eigen::Affine3d::VectorType(0.0, 0.0, 1.0));
    Eigen::Vector4d vx, vy, vz;
    for(int i = 0; i < 3; ++i) {
        vx(i) = 0.0;
        vy(i) = e(i, 1);
        vz(i) = 0.0;
    }
    vx(2) = -1.0; //x-pointing down
    vz(0) = (e_moved.translation() - e.translation())[0];
    vz(1) = (e_moved.translation() - e.translation())[1];
    vz.normalize();
    vy = vz.cross3(vx);
    vx.normalize();
    vy.normalize();
    vz.normalize();
    for(int i = 0; i < 3; ++i) {
        e(i, 0) = vx(i);
        e(i, 1) = vy(i);
        e(i, 2) = vz(i);
    }
    for (int  i = 0 ; i < 3; ++i) { //TODO: MoveIt modify memory (probably), fix it!!
        Eigen::Affine3d ee = e;
        tf::poseEigenToMsg(ee, out_pose);
        if(fabs(sqrt(pow(out_pose.orientation.x, 2) + pow(out_pose.orientation.y, 2) + pow(out_pose.orientation.z, 2) + pow(out_pose.orientation.w, 2)) - 1.0) > 0.0005) {
            ROS_ERROR_STREAM("Result quaternion is not Identity!!!" << i);
            continue;
        } else {
            return true;
        }
    }
    return false;
}

bool compute_poses_table(
    const clopema_moveit::ClopemaGraspFromTableRequest& req,
    const robot_state::RobotState& rs,
    unsigned int pose_i, geometry_msgs::Pose& plan_pose,
    geometry_msgs::Pose& touch_table_pose,
    geometry_msgs::Pose& grasping_pose, geometry_msgs::Pose& final_pose) {

    geometry_msgs::Pose pose_in = req.poses[pose_i];
    geometry_msgs::Pose pose;
    if(!project_to_the_table(pose_in, rs, req.ik_link, req.table_desk, req.frame_id, pose))
        return false;
    Eigen::Affine3d e_pose, e, rotate_grasp, t_offset_minus, t_offset_plus, t_offset_table_plus, t_offset_table_minus;
    Eigen::Affine3d to_base = rs.getFrameTransform(req.table_desk);
    t_offset_minus = Eigen::Affine3d::Identity();
    t_offset_plus = Eigen::Affine3d::Identity();
    t_offset_table_plus = Eigen::Affine3d::Identity();
    t_offset_table_minus = Eigen::Affine3d::Identity();
    t_offset_minus.translation()[2] = -req.offset_minus;
    t_offset_plus.translation()[2] = req.offset_plus;
    t_offset_table_minus.translation()[2] = -req.offset_table_minus;
    t_offset_table_plus.translation()[2] = req.offset_table_plus;
    {
        //rotation to the grasping angle
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, req.grasping_angle, 0);
        rotate_grasp = Eigen::Affine3d::Identity();
        tf::poseMsgToEigen(p, rotate_grasp);
    }

    tf::poseMsgToEigen(pose, e_pose);
    {
        e = e_pose;
        e = to_base * t_offset_table_plus * e * t_offset_minus * rotate_grasp;
        tf::poseEigenToMsg(e, plan_pose);
    }
    {
        e = e_pose;
        e = to_base * t_offset_table_minus * e * t_offset_minus * rotate_grasp;
        tf::poseEigenToMsg(e, touch_table_pose);
    }
    {
        e = e_pose;
        e = to_base * t_offset_table_minus * e * t_offset_plus * rotate_grasp;
        tf::poseEigenToMsg(e, grasping_pose);
    }
    {
        if(req.final_poses.size() > pose_i) {
            tf::poseMsgToEigen(req.final_poses[pose_i], e);
            Eigen::Affine3d t_table = rs.getFrameTransform(req.table_desk);
            Eigen::Affine3d t_base = rs.getFrameTransform(req.frame_id);
            e = to_base * t_table.inverse() * t_base * e;
            tf::poseEigenToMsg(e, final_pose);
        } else {
            e = e_pose;
            e = to_base * t_offset_table_plus * e * t_offset_plus * rotate_grasp;
            tf::poseEigenToMsg(e, final_pose);
        }
    }
    return true;
}

bool grasp_it_cb(clopema_moveit::ClopemaGraspItRequest& req,
                 clopema_moveit::ClopemaGraspItResponse& res) {
    std::string group = resolve_planning_group(req.ik_link);
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> g;
    if(group == "r1_arm")
        g = r1_arm_group;
    else if(group == "r2_arm")
        g = r2_arm_group;


    g->setStartStateToCurrentState();
    g->setPoseReferenceFrame(req.frame_id);
    robot_state::RobotState start_state(*g->getCurrentState());
    start_state.update();
    for(unsigned int i = 0; i < req.poses.size(); ++i) {
        
        g->setStartStateToCurrentState();
        geometry_msgs::Pose pose = req.poses[i];
        transform_to_tip_link(pose, g, req.ik_link, start_state);
        res.joint_trajectories.clear();
        res.error.clear();
        ROS_WARN_STREAM(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        {
            //Plan to pre grasping pose
            g->setStartStateToCurrentState();
            g->setJointValueTarget(pre_grasp(pose, req.offset_minus));
            bool succeed = static_cast<bool>(g->plan(plan));
            if(!succeed) {
                res.error = "Can not plan.";
                continue;
            }
            res.joint_trajectories.push_back(plan.trajectory_.joint_trajectory);
        }
        {
            //Open gripper
            res.joint_trajectories.push_back(open_grip_traj(group));
        }

        {
            //Interpolate to grasping pose
            int k = res.joint_trajectories.size() - 2;
            start_state.setVariablePositions(
                res.joint_trajectories[k].joint_names,
                res.joint_trajectories[k].points.back().positions);
            g->setStartState(start_state);

            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(post_grasp(pose, req.offset_plus));
            moveit_msgs::RobotTrajectory trajectory;
            double f = g->computeCartesianPath(waypoints, 0.01, 1.2,
                                               trajectory);
            if(fabs(1.0 - f) > 0.005) {
                res.error = "Can not interpolate whole trajectory down.";
                ROS_WARN_STREAM(f);
                continue;
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }

        {
            //Close gripper
            res.joint_trajectories.push_back(close_grip_traj(group));
        }

        {
            //Interpolate to pre-grasping pose
            int k = res.joint_trajectories.size() - 2;
            start_state.setVariablePositions(
                res.joint_trajectories[k].joint_names,
                res.joint_trajectories[k].points.back().positions);
            g->setStartState(start_state);

            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(pre_grasp(pose, req.offset_minus));
            moveit_msgs::RobotTrajectory trajectory;
            double f = g->computeCartesianPath(waypoints, 0.01, 1.2,
                                               trajectory);
            if(fabs(1.0 - f) > 0.005) {
                res.error = "Can not interpolate whole trajectory up.";
                continue;
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);

        }
        break;
    }

    if(!res.error.empty()) {
        ROS_INFO_STREAM(res.error);
        res.joint_trajectories.clear();
    } else { //publish trajectory
        moveit_msgs::DisplayTrajectory disp;
        disp.model_id = "clopema";
        for(unsigned int i = 0; i < res.joint_trajectories.size(); ++i) {
            moveit_msgs::RobotTrajectory rtraj;
            rtraj.joint_trajectory = res.joint_trajectories[i];
            disp.trajectory.push_back(rtraj);
        }
        robot_state::robotStateToRobotStateMsg(*g->getCurrentState(),
                                               disp.trajectory_start);
        pub_traj->publish(disp);
    }

    g->clearPoseTargets();
    g->clearPathConstraints();
    g.reset();

    return true;
}


/** \brief Check joint trajectory for the collision in the global planning scene.
 *  \details i.e. dissablign the collision in 'ps' affect behaviour of this method. */
bool check_trajectory_for_collisions(const robot_state::RobotState& plan_state, const trajectory_msgs::JointTrajectory& trajectory) {
    bool valid = true;
    robot_state::RobotState rs_tmp(plan_state);
    BOOST_FOREACH(const trajectory_msgs::JointTrajectoryPoint & p, trajectory.points) {
        rs_tmp.setVariablePositions(trajectory.joint_names, p.positions);
        rs_tmp.update();
        if(!planning_scene_monitor::LockedPlanningSceneRO(ps)->isStateValid(rs_tmp)) {
            valid = false;
            break;
        }
    }
    return valid;
}

bool grasp_from_table_cb2(clopema_moveit::ClopemaGraspFromTableRequest& req, clopema_moveit::ClopemaGraspFromTableResponse& res) {
    std::string group = resolve_planning_group(req.ik_link);
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> g = (group == "r1_arm") ? r1_arm_group : r2_arm_group;
    robot_state::RobotState start_state(planning_scene_monitor::LockedPlanningSceneRO(ps)->getCurrentState());
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);
    start_state.update();
    for(unsigned int i = 0; i < req.poses.size(); ++i) {
        ROS_INFO_STREAM("Pose: " << i);
        start_state.update();
        g->setStartState(start_state);
        res.joint_trajectories.clear();
        res.error.clear();

        geometry_msgs::Pose plan_pose, touch_table_pose, grasping_pose, final_pose;
        if(!compute_poses_table(req, start_state, i, plan_pose, touch_table_pose, grasping_pose, final_pose)) {
            res.error = "Cannot compute grasping poses.";
            continue;
        }
        transform_to_tip_link(plan_pose, g, req.ik_link, start_state);
        transform_to_tip_link(touch_table_pose, g, req.ik_link, start_state);
        transform_to_tip_link(grasping_pose, g, req.ik_link, start_state);
        transform_to_tip_link(final_pose, g, req.ik_link, start_state);

        robot_state::RobotState plan_state(start_state);
        if(!plan_state.setFromIK(plan_state.getJointModelGroup(group), plan_pose, g->getEndEffectorLink())) {
            res.error = "Cannot set plan_state from IKT";
            continue;
        }
        plan_state.update();
        if(!planning_scene_monitor::LockedPlanningSceneRO(ps)->isStateValid(plan_state)) {
            res.error = "Cannot set plan_state from IKT because of collision";
            continue;
        }

        res.joint_trajectories.push_back(open_grip_traj(group));
        moveit_msgs::RobotTrajectory trajectory;
        {
            //Do the interpolation to the grasping pose
            g->setStartState(plan_state);
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(touch_table_pose);
            waypoints.push_back(grasping_pose);
            if(g->computeCartesianPath(waypoints, 0.01, 1.2, trajectory, false) < 0.999) {
                res.error = "Cannot interpolate whole trajectory - down.";
                continue;
            }
            {
                //check collision after the interpolation
                change_collision_with_table(true, req.table_desk); //enable collision
                bool valid = check_trajectory_for_collisions(plan_state, trajectory.joint_trajectory);
                change_collision_with_table(false, req.table_desk); //disable collision
                if(!valid) {
                    res.error = "Cannot interpolate, because of collision - down";
                    continue;
                }
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        res.joint_trajectories.push_back(close_grip_traj(group));

        robot_state::RobotState after_grasp_state(plan_state);
        after_grasp_state.setVariablePositions(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points.back().positions);
        g->setStartState(after_grasp_state);
        {
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(final_pose);
            if(g->computeCartesianPath(waypoints, 0.01, 1.2, trajectory, false) < 0.999) {
                res.error = "Cannot interpolate whole trajectory - up.";
                continue;
            }
            {
                //check collision after the interpolation
                change_collision_with_table(true, req.table_desk); //enable collision
                bool valid = check_trajectory_for_collisions(after_grasp_state, trajectory.joint_trajectory);
                change_collision_with_table(false, req.table_desk); //disable collision
                if(!valid) {
                    res.error = "Cannot interpolate, because of collision - up";
                    continue;
                }
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        {
            //Plan trajectory
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            g->setStartState(start_state);
            g->setJointValueTarget(plan_state);
            if(!g->plan(plan)) {
                res.error = "Can not plan.";
                continue;
            }
            res.joint_trajectories.insert(res.joint_trajectories.begin(), plan.trajectory_.joint_trajectory);
        }
        break;
    }
    if(!res.error.empty()) {
        ROS_INFO_STREAM(res.error);
        res.joint_trajectories.clear();
    } else { //publish trajectory
        moveit_msgs::DisplayTrajectory disp;
        disp.model_id = "clopema";
        for(unsigned int i = 0; i < res.joint_trajectories.size(); ++i) {
            moveit_msgs::RobotTrajectory rtraj;
            rtraj.joint_trajectory = res.joint_trajectories[i];
            disp.trajectory.push_back(rtraj);
        }
        robot_state::robotStateToRobotStateMsg(start_state, disp.trajectory_start);
        pub_traj->publish(disp);
    }
    return true;

}

double computeCartesianPathDual(
    const std::vector<geometry_msgs::Pose>& waypoints1,
    const std::string& link1,
    const std::vector<geometry_msgs::Pose>& waypoints2,
    const std::string& link2, double step, double jump_threshold,
    const robot_state::RobotState& start_state, const std::string& base_link,
    moveit_msgs::RobotTrajectory& msg, bool avoid_collisions) {
    clopema_moveit::GetCartesianPathDual::Request req;
    clopema_moveit::GetCartesianPathDual::Response res;


    robot_state::robotStateToRobotStateMsg(start_state, req.start_state);
    req.group_name = "arms";
    req.header.frame_id = base_link;
    req.header.stamp = ros::Time::now();
    req.waypoints_1 = waypoints1;
    req.link_name_1 = link1;
    req.link_name_2 = link2;
    req.waypoints_2 = waypoints2;
    req.max_step = step;
    req.jump_threshold = jump_threshold;
    req.avoid_collisions = avoid_collisions;

    if(ros::service::call("/compute_cartesian_path_dual", req, res)) {
        if(res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            msg = res.solution;
            return res.fraction;
        } else
            return -1.0;
    }

    return -1.0;
}

bool grasp_from_table_dual_cb(clopema_moveit::ClopemaGraspFromTableDualRequest& req, clopema_moveit::ClopemaGraspFromTableDualResponse& res) {

    if(req.poses_1.size() != req.poses_2.size()) {
        res.error = "Poses size must be equal for both hand";
        return true;
    }

    robot_state::RobotState start_state(planning_scene_monitor::LockedPlanningSceneRO(ps)->getCurrentState());
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);
    start_state.update();

    for(unsigned int i = 0; i < req.poses_1.size(); ++i) {
        res.joint_trajectories.clear();
        res.error.clear();
        geometry_msgs::Pose plan_pose_1, touch_table_pose_1, grasping_pose_1, final_pose_1;
        geometry_msgs::Pose plan_pose_2, touch_table_pose_2, grasping_pose_2, final_pose_2;
        {
            //compute poses
            boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> g;
            clopema_moveit::ClopemaGraspFromTableRequest req1;
            req1.frame_id = req.frame_id;
            req1.grasping_angle = req.grasping_angle;
            req1.offset_minus = req.offset_minus;
            req1.offset_plus = req.offset_plus;
            req1.offset_table_minus = req.offset_table_minus;
            req1.offset_table_plus = req.offset_table_plus;
            req1.start_state = req.start_state;
            req1.table_desk = req.table_desk;
            {
                //first poses transformation
                req1.poses = req.poses_1;
                req1.final_poses = req.final_poses_1;
                req1.ik_link = req.ik_link_1;
                if(!compute_poses_table(req1, start_state, i, plan_pose_1, touch_table_pose_1, grasping_pose_1, final_pose_1)) {
                    res.error = "Cannot compute grasping poses for first arm.";
                    continue;
                }
                g = (resolve_planning_group(req1.ik_link) == "r1_arm") ? r1_arm_group : r2_arm_group;
                transform_to_tip_link(plan_pose_1, g, req1.ik_link, start_state);
                transform_to_tip_link(touch_table_pose_1, g, req1.ik_link, start_state);
                transform_to_tip_link(grasping_pose_1, g, req1.ik_link, start_state);
                transform_to_tip_link(final_pose_1, g, req1.ik_link, start_state);
            }
            {
                //second poses transformation
                req1.poses = req.poses_2;
                req1.final_poses = req.final_poses_2;
                req1.ik_link = req.ik_link_2;
                if(!compute_poses_table(req1, start_state, i, plan_pose_2, touch_table_pose_2, grasping_pose_2, final_pose_2)) {
                    res.error = "Cannot compute grasping poses for second arm.";
                    continue;
                }
                g = (resolve_planning_group(req1.ik_link) == "r1_arm") ? r1_arm_group : r2_arm_group;
                transform_to_tip_link(plan_pose_2, g, req1.ik_link, start_state);
                transform_to_tip_link(touch_table_pose_2, g, req1.ik_link, start_state);
                transform_to_tip_link(grasping_pose_2, g, req1.ik_link, start_state);
                transform_to_tip_link(final_pose_2, g, req1.ik_link, start_state);
            }
        }
        std::string tip_1 = resolve_planning_group(req.ik_link_1) == "r1_arm" ? "r1_tip_link" : "r2_tip_link";
        std::string tip_2 = resolve_planning_group(req.ik_link_2) == "r1_arm" ? "r1_tip_link" : "r2_tip_link";

        robot_state::RobotState plan_state(start_state);
        const robot_model::JointModelGroup* jmg1 = plan_state.getJointModelGroup(resolve_planning_group(req.ik_link_1));
        const robot_model::JointModelGroup* jmg2 = plan_state.getJointModelGroup(resolve_planning_group(req.ik_link_2));
        if(!plan_state.setFromIK(jmg1, plan_pose_1, tip_1)) {
            res.error = "Cannot set plan_state from IKT for " + req.ik_link_1;
            continue;
        }
        if(!plan_state.setFromIK(jmg2, plan_pose_2, tip_2)) {
            res.error = "Cannot set plan_state from IKT for " + req.ik_link_2;
            continue;
        }
        plan_state.update();

        if(!planning_scene_monitor::LockedPlanningSceneRO(ps)->isStateValid(plan_state)) {
            res.error = "Cannot set plan_state from IKT because of collision";
            continue;
        }

        res.joint_trajectories.push_back(open_grip_traj("arms"));
        moveit_msgs::RobotTrajectory trajectory;

        {
            //Do the interpolation to the grasping pose
            std::vector<geometry_msgs::Pose> waypoints_1, waypoints_2;
            waypoints_1.push_back(touch_table_pose_1);
            waypoints_1.push_back(grasping_pose_1);
            waypoints_2.push_back(touch_table_pose_2);
            waypoints_2.push_back(grasping_pose_2);
            double d = computeCartesianPathDual(waypoints_1, tip_1, waypoints_2, tip_2, 0.01, 1.2, plan_state, "base_link" , trajectory, false);
            if(d < 0.999) {
                res.error = "Can not interpolate whole trajectory to grasp.";
                continue;
            }
            {
                //check collision after the interpolation
                change_collision_with_table(true, req.table_desk); //enable collision
                bool valid = check_trajectory_for_collisions(plan_state, trajectory.joint_trajectory);
                change_collision_with_table(false, req.table_desk); //disable collision
                if(!valid) {
                    res.error = "Can not interpolate, because of collision, to grasp";
                    continue;
                }
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        res.joint_trajectories.push_back(close_grip_traj("arms"));

        robot_state::RobotState after_grasp_state(plan_state);
        after_grasp_state.setVariablePositions(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points.back().positions);
        {
            std::vector<geometry_msgs::Pose> waypoints_1, waypoints_2;
            waypoints_1.push_back(final_pose_1);
            waypoints_2.push_back(final_pose_2);
            double d = computeCartesianPathDual(waypoints_1, tip_1, waypoints_2, tip_2, 0.01, 1.2, after_grasp_state, "base_link" , trajectory, false);
            if(d < 0.999) {
                res.error = "Can not interpolate whole trajectory to final pose.";
                continue;
            }
            {
                //check collision after the interpolation
                change_collision_with_table(true, req.table_desk); //enable collision
                bool valid = check_trajectory_for_collisions(after_grasp_state, trajectory.joint_trajectory);
                change_collision_with_table(false, req.table_desk); //disable collision
                if(!valid) {
                    res.error = "Can not interpolate, because of collision, to final pose";
                    continue;
                }
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        {
            //Plan trajectory
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            arms_group->setStartState(start_state);
            arms_group->setJointValueTarget(plan_state);
            if(!arms_group->plan(plan)) {
                res.error = "Can not plan.";
                continue;
            }
            res.joint_trajectories.insert(res.joint_trajectories.begin(), plan.trajectory_.joint_trajectory);
        }
        break;
    }
    if(!res.error.empty()) {
        ROS_INFO_STREAM(res.error);
        res.joint_trajectories.clear();
    } else { //publish trajectory
        moveit_msgs::DisplayTrajectory disp;
        disp.model_id = "clopema";
        for(unsigned int i = 0; i < res.joint_trajectories.size(); ++i) {
            moveit_msgs::RobotTrajectory rtraj;
            rtraj.joint_trajectory = res.joint_trajectories[i];
            disp.trajectory.push_back(rtraj);
        }
        robot_state::robotStateToRobotStateMsg(start_state, disp.trajectory_start);
        pub_traj->publish(disp);
    }
    return true;

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "graspping_services",
              ros::init_options::AnonymousName);
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    r1_arm_group.reset(new moveit::planning_interface::MoveGroupInterface("r1_arm"));
    r2_arm_group.reset(new moveit::planning_interface::MoveGroupInterface("r2_arm"));
    arms_group.reset(new moveit::planning_interface::MoveGroupInterface("arms"));
    r1_arm_group->setPlannerId("RRTConnectkConfigDefault");
    r2_arm_group->setPlannerId("RRTConnectkConfigDefault");
    arms_group->setPlannerId("RRTConnectkConfigDefault");

    ros::ServiceServer ser_graspit = node.advertiseService("/clopema_planner/grasp_it", grasp_it_cb);
    //ros::ServiceServer ser_grasptable = node.advertiseService("/clopema_planner/grasp_from_table", grasp_from_table_cb2);
    //ros::ServiceServer ser_grasptable_dual = node.advertiseService("/clopema_planner/grasp_from_table_dual", grasp_from_table_dual_cb);

    pub_traj.reset(new ros::Publisher);
    *pub_traj = node.advertise<moveit_msgs::DisplayTrajectory>(
                    "/move_group/display_planned_path", 1);
    ps.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    ps->startStateMonitor();

    ros::waitForShutdown();
    spinner.stop();
    r1_arm_group.reset();
    r2_arm_group.reset();
    arms_group.reset();
    pub_traj.reset();
    ps.reset();
    return 0;
}


