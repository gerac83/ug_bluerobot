/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2014-02-28
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Grasping services - grasp_from_table, grasp_it
 *
 */

#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <boost/foreach.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <clopema_moveit/ClopemaGraspFromTable.h>
#include <clopema_moveit/ClopemaGraspFromTableDual.h>
#include <clopema_moveit/ClopemaGraspIt.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;
using namespace moveit::planning_interface;

class GraspingServices {
    public:
        GraspingServices() : node() {
            crc.reset(new clopema_robot::ClopemaRobotCommander("arms"));
            //ser_graspit = node.advertiseService("/clopema_planner/grasp_it", &GraspingServices::grasp_it_cb, this);
            ser_grasptable = node.advertiseService("/clopema_planner/grasp_from_table", &GraspingServices::grasp_from_table_cb, this);
            ser_grasptable_dual = node.advertiseService("/clopema_planner/grasp_from_table_dual", &GraspingServices::grasp_from_table_dual_cb, this);
        }

        ~GraspingServices() {}

        /** @brief Project pose to the table - result pose will be in table frame. */
        bool project_to_the_table(const geometry_msgs::Pose& pose, const robot_state::RobotState& rs,
                                  const std::string& ik_link, const std::string& table_id, const std::string& frame_id, geometry_msgs::Pose& out_pose);

        /** \brief Compute poses for grasping from the table */
        bool compute_poses_table(const geometry_msgs::Pose& pose_in, const geometry_msgs::Pose& final_pose_in , std::string table_desk, std::string frame_id, std::string ik_link,
                                 const robot_state::RobotState& rs, double offset_minus, double offset_plus, double offset_table_minus, double offset_table_plus, double grasping_angle,
                                 geometry_msgs::Pose& plan_pose, geometry_msgs::Pose& touch_table_pose, geometry_msgs::Pose& grasping_pose, geometry_msgs::Pose& final_pose);

        /** \brief Return false if quaternion is not valid -- length is not equal to one. */
        bool isValidQuaternion(const geometry_msgs::Quaternion& q);


        /** \brief Callback function to grasp the garment from the table */
        bool grasp_from_table_cb(clopema_moveit::ClopemaGraspFromTableRequest& req, clopema_moveit::ClopemaGraspFromTableResponse& res);

        /** \brief Get all table links based on the table desk frame.
          * \details based on the number 'num' it return the first or second column of the collision allowed matrix - for the crc.checkTraj function */
        std::vector<std::string> getEnabledCollisionLinks(const std::string& table_desk, const std::string& group, int num) {
            std::vector<std::string> ret1, ret2;
            if(group == "r1_arm") {
                ret2.push_back("r1_gripper");
                ret1.push_back(table_desk);
            } else if(group == "r2_arm") {
                ret2.push_back("r2_gripper");
                ret1.push_back(table_desk);
            } else {
                ret2.push_back("r1_gripper");
                ret2.push_back("r2_gripper");
                ret1.push_back(table_desk);
                ret1.push_back(table_desk);
            }

            if(num == 1)
                return ret1;
            else
                return ret2;
        }

        /** \brief Callback function for dual arm grasping */
        bool grasp_from_table_dual_cb(clopema_moveit::ClopemaGraspFromTableDualRequest& req, clopema_moveit::ClopemaGraspFromTableDualResponse& res);

    private:
        ros::NodeHandle node;
        boost::shared_ptr<clopema_robot::ClopemaRobotCommander> crc;
        boost::mutex mutex_robot;
        ros::ServiceServer ser_graspit;
        ros::ServiceServer ser_grasptable;
        ros::ServiceServer ser_grasptable_dual;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasping_services");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(3);
    spinner.start();
    GraspingServices gs;

    ros::waitForShutdown();
    spinner.stop();
    return 0;
}

bool GraspingServices::grasp_from_table_dual_cb(clopema_moveit::ClopemaGraspFromTableDualRequest& req, clopema_moveit::ClopemaGraspFromTableDualResponse& res) {
    boost::unique_lock<boost::mutex> l(mutex_robot);
    if(req.poses_1.size() != req.poses_2.size()) {
        res.error = "Poses size must be equal for both hand";
        return true;
    }
    robot_state::RobotState start_state(*crc->getCurrentState());
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);
    start_state.update();

    for(unsigned int i = 0; i < req.poses_1.size(); ++i) {
        ROS_INFO_STREAM("Last errors: " << res.error);
        ROS_INFO_STREAM("Pose: " << i);
        res.joint_trajectories.clear();
        res.error.clear();
        geometry_msgs::Pose plan_pose_1, touch_table_pose_1, grasping_pose_1, final_pose_1;
        geometry_msgs::Pose plan_pose_2, touch_table_pose_2, grasping_pose_2, final_pose_2;
        std::string tip_1, tip_2, group_1, group_2;
        {
            {
                geometry_msgs::Pose final_pose_in, pose_in = req.poses_1[i];
                if(req.final_poses_1.size() > i) {
                    final_pose_in = req.final_poses_1[i];
                }
                geometry_msgs::Pose plan_pose, touch_table_pose, grasping_pose, final_pose;
                if(!compute_poses_table(pose_in, final_pose_in, req.table_desk, req.frame_id, req.ik_link_1,
                                        start_state, req.offset_minus, req.offset_plus, req.offset_table_minus, req.offset_table_plus, req.grasping_angle,
                                        plan_pose, touch_table_pose, grasping_pose, final_pose)) {
                    res.error = "Cannot compute grasping poses.";
                    continue;
                }
                std::string tip_link;
                if(!crc->transform_to_tip(plan_pose, req.ik_link_1, plan_pose_1, tip_link)) {
                    ROS_ERROR_STREAM("Cannot transform to the tip link");
                    return false;
                }
                crc->transform_to_tip(touch_table_pose, req.ik_link_1, touch_table_pose_1, tip_link);
                crc->transform_to_tip(grasping_pose, req.ik_link_1, grasping_pose_1, tip_link);
                crc->transform_to_tip(final_pose, req.ik_link_1, final_pose_1, tip_link);
                tip_1 = tip_link;
                std::string group;
                if(tip_link == "r1_tip_link")
                    group = "r1_arm";
                else if(tip_link == "r2_tip_link")
                    group = "r2_arm";
                else {
                    res.error = "Not known tip link, cannot resplve planning group.";
                    continue;
                }
                group_1 = group;
            }
            {
                geometry_msgs::Pose final_pose_in, pose_in = req.poses_2[i];
                if(req.final_poses_2.size() > i) {
                    final_pose_in = req.final_poses_2[i];
                }
                geometry_msgs::Pose plan_pose, touch_table_pose, grasping_pose, final_pose;
                if(!compute_poses_table(pose_in, final_pose_in, req.table_desk, req.frame_id, req.ik_link_2,
                                        start_state, req.offset_minus, req.offset_plus, req.offset_table_minus, req.offset_table_plus, req.grasping_angle,
                                        plan_pose, touch_table_pose, grasping_pose, final_pose)) {
                    res.error = "Cannot compute grasping poses.";
                    continue;
                }
                std::string tip_link;
                if(!crc->transform_to_tip(plan_pose, req.ik_link_2, plan_pose_2, tip_link)) {
                    ROS_ERROR_STREAM("Cannot transform to the tip link");
                    return false;
                }
                crc->transform_to_tip(touch_table_pose, req.ik_link_2, touch_table_pose_2, tip_link);
                crc->transform_to_tip(grasping_pose, req.ik_link_2, grasping_pose_2, tip_link);
                crc->transform_to_tip(final_pose, req.ik_link_2, final_pose_2, tip_link);
                tip_2 = tip_link;
                std::string group;
                if(tip_link == "r1_tip_link")
                    group = "r1_arm";
                else if(tip_link == "r2_tip_link")
                    group = "r2_arm";
                else {
                    res.error = "Not known tip link, cannot resplve planning group.";
                    continue;
                }
                group_2 = group;
            }
        }

        robot_state::RobotState plan_state(start_state);
        if(!plan_state.setFromIK(plan_state.getJointModelGroup(group_1), plan_pose_1, tip_1)) {
            res.error = "Cannot set plan_state from IKT for " + req.ik_link_1;
            continue;
        }
        if(!plan_state.setFromIK(plan_state.getJointModelGroup(group_2), plan_pose_2, tip_2)) {
            res.error = "Cannot set plan_state from IKT for " + req.ik_link_2;
            continue;
        }
        plan_state.update();
        res.joint_trajectories.push_back(crc->gripper_trajectory(clopema_robot::GRIPPER_OPEN, "arms"));
        moveit_msgs::RobotTrajectory trajectory;
        {
            //Do the interpolation to the grasping pose
            std::vector<geometry_msgs::Pose> waypoints_1, waypoints_2;
            waypoints_1.push_back(touch_table_pose_1);
            waypoints_1.push_back(grasping_pose_1);
            waypoints_2.push_back(touch_table_pose_2);
            waypoints_2.push_back(grasping_pose_2);
            crc->setStartState(plan_state);
            crc->setPoseReferenceFrame("base_link");
            double d = crc->computeCartesianPathDual(waypoints_1, tip_1, waypoints_2, tip_2, 0.01, 1.2, trajectory, false);
            if(d < 0.999) {
                res.error = "Cannot interpolate whole trajectory to grasp.";
                continue;
            }
            bool valid = crc->check_trajectory(trajectory, getEnabledCollisionLinks(req.table_desk, "arms", 1), getEnabledCollisionLinks(req.table_desk, "arms", 2));
            if(!valid) {
                res.error = "Cannot interpolate, because of collision - down";
                continue;
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        res.joint_trajectories.push_back(crc->gripper_trajectory(clopema_robot::GRIPPER_CLOSE, "arms"));

        robot_state::RobotState after_grasp_state(plan_state);
        after_grasp_state.setVariablePositions(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points.back().positions);
        {
            std::vector<geometry_msgs::Pose> waypoints_1, waypoints_2;
            waypoints_1.push_back(final_pose_1);
            waypoints_2.push_back(final_pose_2);
            crc->setStartState(after_grasp_state);
            double d = crc->computeCartesianPathDual(waypoints_1, tip_1, waypoints_2, tip_2, 0.01, 1.2, trajectory, false);
            if(d < 0.999) {
                res.error = "Cannot interpolate whole trajectory to grasp.";
                continue;
            }
            bool valid = crc->check_trajectory(trajectory, getEnabledCollisionLinks(req.table_desk, "arms", 1), getEnabledCollisionLinks(req.table_desk, "arms", 2));
            if(!valid) {
                res.error = "Cannot interpolate, because of collision - down";
                continue;
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        {
            //Plan trajectory
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            crc->setStartState(start_state);
            crc->setJointValueTarget(plan_state);
            if(!crc->plan(plan)) {
                res.error = "Cannot plan.";
                continue;
            }
            res.joint_trajectories.insert(res.joint_trajectories.begin(), plan.trajectory_.joint_trajectory);
        }
        break;
    }
    if(!res.error.empty()) {
        ROS_INFO_STREAM(res.error);
        res.joint_trajectories.clear();
    }
    return true;

}


bool GraspingServices::grasp_from_table_cb(clopema_moveit::ClopemaGraspFromTableRequest& req, clopema_moveit::ClopemaGraspFromTableResponse& res) {
    boost::unique_lock<boost::mutex> l(mutex_robot);
    robot_state::RobotState start_state(*crc->getCurrentState());
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);
    start_state.update();

    for(unsigned int i = 0; i < req.poses.size(); ++i) {
        ROS_WARN_STREAM("Last errors: " << res.error);
        ROS_INFO_STREAM("Pose: " << i);
        crc->setStartState(start_state);
        res.joint_trajectories.clear();
        res.error.clear();

        geometry_msgs::Pose final_pose_in, pose_in = req.poses[i];
        if(req.final_poses.size() > i) {
            final_pose_in = req.final_poses[i];
        }
        //req.offset_plus = 0.02;
        geometry_msgs::Pose plan_pose, touch_table_pose, grasping_pose, final_pose;
        if(!compute_poses_table(pose_in, final_pose_in, req.table_desk, req.frame_id, req.ik_link,
                                start_state, req.offset_minus, req.offset_plus, req.offset_table_minus, req.offset_table_plus, req.grasping_angle,
                                plan_pose, touch_table_pose, grasping_pose, final_pose)) {
            res.error = "Cannot compute grasping poses.";
            continue;
        }
        std::string tip_link, group;
        if(!crc->transform_to_tip(plan_pose, req.ik_link, plan_pose, tip_link)) {
            res.error = "Cannot transform to the tip link";
            continue;
        }
        crc->transform_to_tip(touch_table_pose, req.ik_link, touch_table_pose, tip_link);
        crc->transform_to_tip(grasping_pose, req.ik_link, grasping_pose, tip_link);
        crc->transform_to_tip(final_pose, req.ik_link, final_pose, tip_link);

        if(tip_link == "r1_tip_link")
            group = "r1_arm";
        else if(tip_link == "r2_tip_link")
            group = "r2_arm";
        else {
            res.error = "Not known tip link, cannot resplve planning group.";
            continue;
        }

        robot_state::RobotState plan_state(start_state);
        if(!plan_state.setFromIK(plan_state.getJointModelGroup(group), plan_pose, tip_link)) {
            res.error = "Cannot set plan_state from IKT";
            continue;
        }
        plan_state.update();
        res.joint_trajectories.push_back(crc->gripper_trajectory(clopema_robot::GRIPPER_OPEN, group));
        moveit_msgs::RobotTrajectory trajectory;
        {
            //Do the interpolation to the grasping pose
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(touch_table_pose);
            waypoints.push_back(grasping_pose);
            crc->setStartState(plan_state);
            if(crc->computeCartesianPathDual(waypoints, tip_link, waypoints, tip_link, 0.01, 1.2, trajectory, false) < 0.999) {
                res.error = "Cannot interpolate whole trajectory - down.";
                continue;
            }
            //check collision after the interpolation
            bool valid = crc->check_trajectory(trajectory, getEnabledCollisionLinks(req.table_desk, group, 1), getEnabledCollisionLinks(req.table_desk, group, 2));
            if(!valid) {
                res.error = "Cannot interpolate, because of collision - down";
                continue;
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        res.joint_trajectories.push_back(crc->gripper_trajectory(clopema_robot::GRIPPER_CLOSE, group));

        robot_state::RobotState after_grasp_state(plan_state);
        after_grasp_state.setVariablePositions(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points.back().positions);
        {
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(final_pose);
            crc->setStartState(after_grasp_state);
            if(crc->computeCartesianPathDual(waypoints, tip_link, waypoints, tip_link, 0.01, 1.2, trajectory, false) < 0.999) {
                res.error = "Cannot interpolate whole trajectory - up.";
                continue;
            }
            bool valid = crc->check_trajectory(trajectory, getEnabledCollisionLinks(req.table_desk, group, 1), getEnabledCollisionLinks(req.table_desk, group, 2));
            if(!valid) {
                res.error = "Cannot interpolate, because of collision - up";
                continue;
            }
            res.joint_trajectories.push_back(trajectory.joint_trajectory);
        }
        {
            //Plan trajectory
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            crc->setStartState(start_state);
            crc->setJointValueTarget(plan_state);
            if(!crc->plan(plan)) {
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
    }
    return true;
}


bool GraspingServices::project_to_the_table(const geometry_msgs::Pose& pose, const robot_state::RobotState& rs,
        const std::string& ik_link, const std::string& table_id, const std::string& frame_id, geometry_msgs::Pose& out_pose) {
    if(!rs.knowsFrameTransform(table_id)) {
        ROS_INFO_STREAM("Do not known table id: " << table_id);
        return false;
    }
    Affine3d e, e_tmp, e_moved;
    tf::poseMsgToEigen(pose, e);
    Affine3d t_table = rs.getFrameTransform(table_id);
    Affine3d t_base = rs.getFrameTransform(frame_id);
    e_tmp = t_table.inverse() * t_base * e;
    e_tmp.translation()[2] = 0.0;
    e = e_tmp;
    e_moved = e * Translation3d(0.0, 0.0, 1.0);
    Eigen::Vector3d vx, vy, vz;

    for(int i = 0; i < 3; ++i) {
        vx(i) = 0.0;
        vy(i) = e(i, 1);
        vz(i) = 0.0;
    }
    vx(2) = -1.0; //x-pointing down
    vz(0) = (e_moved.translation() - e.translation())[0];
    vz(1) = (e_moved.translation() - e.translation())[1];
    vz.normalize();
    vy = vz.cross(vx);
    vx.normalize();
    vy.normalize();
    vz.normalize();
    for(int i = 0; i < 3; ++i) {
        e(i, 0) = vx(i);
        e(i, 1) = vy(i);
        e(i, 2) = vz(i);
    }
    tf::poseEigenToMsg(e, out_pose);
    if(!isValidQuaternion(out_pose.orientation)) {
        ROS_ERROR_STREAM("Result quaternion is not Identity!!!");
        ROS_INFO_STREAM(e.matrix());
        ROS_INFO_STREAM(out_pose);
    } else {
        return true;
    }
    return false;
}

bool GraspingServices::compute_poses_table(const geometry_msgs::Pose& pose_in, const geometry_msgs::Pose& final_pose_in , std::string table_desk, std::string frame_id, std::string ik_link,
        const robot_state::RobotState& rs, double offset_minus, double offset_plus, double offset_table_minus, double offset_table_plus, double grasping_angle,
        geometry_msgs::Pose& plan_pose, geometry_msgs::Pose& touch_table_pose, geometry_msgs::Pose& grasping_pose, geometry_msgs::Pose& final_pose) {

    geometry_msgs::Pose pose;
    if(!project_to_the_table(pose_in, rs, ik_link, table_desk, frame_id, pose))
        return false;
    Affine3d e_pose, e, rotate_grasp, t_offset_minus, t_offset_plus, t_offset_table_plus, t_offset_table_minus;
    Affine3d to_base = rs.getFrameTransform(table_desk);
    t_offset_minus.setIdentity();
    t_offset_plus.setIdentity();
    t_offset_table_plus.setIdentity();
    t_offset_table_minus.setIdentity();
    rotate_grasp.setIdentity();
    t_offset_minus.translation()[2] = -offset_minus;
    t_offset_plus.translation()[2] = offset_plus;
    t_offset_table_minus.translation()[2] = -offset_table_minus;
    t_offset_table_plus.translation()[2] = offset_table_plus;
    {
        //rotation to the grasping angle
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, grasping_angle, 0);
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
        if(isValidQuaternion(final_pose_in.orientation)) {
            tf::poseMsgToEigen(final_pose_in, e);
            Eigen::Affine3d t_table = rs.getFrameTransform(table_desk);
            Eigen::Affine3d t_base = rs.getFrameTransform(frame_id);
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

bool GraspingServices::isValidQuaternion(const geometry_msgs::Quaternion& q) {
    if(fabs(sqrt(pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2) + pow(q.w, 2)) - 1.0) > 0.0005) {
        return false;
    } else {
        return true;
    }
}
