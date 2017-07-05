/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2014-01-03
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Grasp and fold service
 */

#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <clopema_robot/GraspAndFold.h>
#include <moveit/robot_state/conversions.h>
#include <boost/foreach.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <geometry_msgs/PoseArray.h>

static const bool PUBLISH_GRASPING_POINTS = false;

boost::shared_ptr<clopema_robot::ClopemaRobotCommander> crc;

/** @brief Project pose to the table without changing frame_id and with offsets. */
bool project_to_the_table(geometry_msgs::Pose& pose,
                          const robot_state::RobotState& rs,
                          const std::string& ik_link, const std::string& table_id,
                          const std::string& frame_id, double offset_above_table, double grasping_angle, double offset_plus) {
    using namespace Eigen;
    if(!rs.knowsFrameTransform(table_id)) {
        ROS_ERROR_STREAM("Do not known table id: " << table_id);
        return false;
    }
    Affine3d e, e_tmp, e_moved;
    tf::poseMsgToEigen(pose, e);
    Eigen::Affine3d t_table = rs.getFrameTransform(table_id);
    Eigen::Affine3d t_base = rs.getFrameTransform(frame_id);
    e_tmp = t_table.inverse() * t_base * e;
    e_tmp.translation()[2] = offset_above_table;
    e = e_tmp;
    e_moved = e * Translation3d(0.0, 0.0, 1.0);
    Eigen::Vector3d vx, vy, vz;

    for(int i = 0; i < 3; ++i) {
        vx(i) = 0;
        vy(i) = e(i, 1);
        vz(i) = 0;
    }
    vx(2) = -1; //x-pointing down
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

    Eigen::Affine3d rotate_grasp;
    {
        //rotation to the grasping angle
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, grasping_angle, 0);
        rotate_grasp.setIdentity();
        tf::poseMsgToEigen(p, rotate_grasp);
    }
    Eigen::Affine3d t_offset_plus = Eigen::Affine3d::Identity();
    t_offset_plus.translation()[2] = offset_plus;
    e = t_base.inverse() * t_table * e * t_offset_plus * rotate_grasp; //transform to original frame
    tf::poseEigenToMsg(e, pose);
    return true;
}

void publish_grasping_point(geometry_msgs::Pose p1, geometry_msgs::Pose p2, geometry_msgs::Pose p3, geometry_msgs::Pose p4, std::string table_desk) {
    if(PUBLISH_GRASPING_POINTS) {
        geometry_msgs::PoseArray pa;
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, 0);
        pa.poses.resize(4, p);
        pa.poses[0].position = p1.position;
        pa.poses[1].position = p2.position;
        pa.poses[2].position = p3.position;
        pa.poses[3].position = p4.position;
        pa.header.frame_id = table_desk;
        pa.header.stamp = ros::Time::now();

        static ros::NodeHandle node;
        static ros::Publisher pub = node.advertise<geometry_msgs::PoseArray>("/pose_array_shifted", 1);
        static bool first(true);
        if(first || (first = false))
            ros::Duration(2.0).sleep();
        pub.publish(pa);
    }
}

/** \brief Plan to the end pose after GFold was computed.
 *  \details The interpolation in direction of grasping is used to move as far as possible but no more then req.offset_table_plus.
 *            i.e. The angle is from range 45deg to 135deg w.r.t teble desk (45 is the best) [the angle of movement]
 * */
bool plan_to_end_pose(const clopema_robot::GraspAndFold::Request& req, const robot_state::RobotState& start_state, 
                      trajectory_msgs::JointTrajectory& traj,
                      const geometry_msgs::Pose final_point_1, const geometry_msgs::Pose final_point_2,
                      std::string tip_1, std::string tip_2,
                      const Eigen::Vector3d& v, const Eigen::Vector3d& C_up, const Eigen::Vector3d& D_up,
                      const std::vector<std::string>& elinks1, const std::vector<std::string>& elinks2 ) {
                        
        for(double dist = req.offset_table_plus; dist > -req.offset_table_plus; dist -= 0.01) {
            //Move in direction of folding.
            geometry_msgs::Pose end_pose_1, end_pose_2;
            tf::pointEigenToMsg(C_up + v * dist, end_pose_1.position);
            tf::pointEigenToMsg(D_up + v * dist, end_pose_2.position);
            end_pose_1.orientation = final_point_1.orientation;
            end_pose_2.orientation = final_point_2.orientation;
            project_to_the_table(end_pose_1, start_state, tip_1, req.table_desk, req.frame_id, req.offset_table_plus, req.grasping_angle, 0.0);
            project_to_the_table(end_pose_2, start_state, tip_2, req.table_desk, req.frame_id, req.offset_table_plus, req.grasping_angle, 0.0);
            
            std::vector<geometry_msgs::Pose> wp1, wp2;
            wp1.push_back(end_pose_1);
            wp2.push_back(end_pose_2);
            
            //go away in the reverse grasping direction by 5cm
            wp1.push_back(geometry_msgs::Pose());
            wp2.push_back(geometry_msgs::Pose());
            Eigen::Affine3d e1, e2, ep1a, ep2a;
            tf::poseMsgToEigen(end_pose_1, e1);
            tf::poseMsgToEigen(end_pose_2, e2);
            tf::poseEigenToMsg(e1 * Eigen::Translation3d(0.0,0.0,-0.1), wp1.back());
            tf::poseEigenToMsg(e2 * Eigen::Translation3d(0.0,0.0,-0.1), wp2.back());
            
            moveit_msgs::RobotTrajectory rtraj;
            crc->setPoseReferenceFrame(req.frame_id);
            double d;
            if(req.poses_2.empty())
                d = crc->computeCartesianPathDual(wp1, tip_1, wp1, tip_1, 0.01, 1.5, rtraj, false);
            else
                d = crc->computeCartesianPathDual(wp1, tip_1, wp2, tip_2, 0.01, 1.5, rtraj, false);
            if(!(fabs(d - 1.0) < 0.00001)) {
                ROS_WARN_STREAM("Cannot interpolate up");
                continue;
            }

            ROS_INFO_STREAM("checking trajectory");
            if(!crc->check_trajectory(rtraj, elinks1, elinks2)) {
                ROS_WARN_STREAM("Cannot interpolate up because of collision");
                continue;
            }
            traj = rtraj.joint_trajectory;
            ROS_INFO_STREAM("Selected distance in move to end_pose:" << dist);
            return true;
        }
        return false;
}

bool grasp_and_fold(const clopema_robot::GraspAndFold::Request& req, unsigned int i, const robot_state::RobotState& start_state,
                    std::vector<trajectory_msgs::JointTrajectory>& trajectories, trajectory_msgs::JointTrajectory& gtraj, bool first_combination) {

    std::string tip_1 = "r1_ee", tip_2 = "r2_ee";
    if(!first_combination) {
        tip_1 = "r2_ee", tip_2 = "r1_ee";
    }
    if(req.poses_2.empty())
        tip_2 = "";

    const double oh = 0.02, op = req.offset_plus, og = 0.00; //offset height, point, gripper border
    geometry_msgs::Pose final_point_1, final_point_2, grasp_point_1, grasp_point_2, end_pose_1, end_pose_2;
    //final_points are gfold final, end_poses are interpolated up after gfold
    Eigen::Vector3d n, v, A, A_, B, B_, C, C_, D, D_, Cup, Dup, C_up, D_up;
    {
        tf::pointMsgToEigen(req.poses_1[i].position, A);
        if(req.poses_2.empty())
            tf::pointMsgToEigen(req.poses_1[i].position, B);
        else
            tf::pointMsgToEigen(req.poses_2[i].position, B);
        tf::pointMsgToEigen(req.final_point_1, A_);
        if(req.poses_2.empty())
            tf::pointMsgToEigen(req.final_point_1, B_);
        else
            tf::pointMsgToEigen(req.final_point_2, B_);

        v = A_ - A;
        n = B - A;
        v.normalize();
        if(!n.isZero()) //graspin with one arm so not move the point B==A
            n.normalize();

        C = n * op + v * op + A;
        D = -n * op + v * op + B;
        C_ = n * op - v * op + A_;
        D_ = -n * op - v * op + B_;

        double l_c = (C_ - C).norm();
        double l_d = (D_ - D).norm();
        double x_c = l_c / 2.0 - sqrt(pow(l_c / 2.0 - og, 2.0) - pow(oh, 2.0));
        double x_d = l_d / 2.0 - sqrt(pow(l_d / 2.0 - og, 2.0) - pow(oh, 2.0));
        if(boost::math::isnan<double>(x_c) || boost::math::isnan<double>(x_d)) {
            ROS_ERROR_STREAM("Too small grasping size");
            return false;
        }
        C_up = C_ - x_c * v;
        D_up = D_ - x_d * v;

        tf::pointEigenToMsg(C, grasp_point_1.position);
        tf::pointEigenToMsg(D, grasp_point_2.position);
        tf::pointEigenToMsg(C_up, final_point_1.position);
        tf::pointEigenToMsg(D_up, final_point_2.position);
        grasp_point_1.orientation = final_point_1.orientation = req.poses_1[i].orientation;
        if(req.poses_2.empty())
            grasp_point_2.orientation = final_point_2.orientation = req.poses_1[i].orientation;
        else
            grasp_point_2.orientation = final_point_2.orientation = req.poses_2[i].orientation;

        if(!project_to_the_table(final_point_1, start_state, tip_1, req.table_desk, req.frame_id, oh, req.grasping_angle, 0.0)) {
            ROS_ERROR_STREAM(" [grasp and fold] Cannot project to the table");
            return false;
        }
        project_to_the_table(final_point_2, start_state, tip_2, req.table_desk, req.frame_id, oh, req.grasping_angle, 0.0);
    }

    bool okk;
    crc->setStartState(start_state);
    if(req.poses_2.empty()) {
        okk = crc->grasp_from_table_plan(grasp_point_1, tip_1, req.table_desk, trajectories, geometry_msgs::Pose(),
                                         req.frame_id, req.offset_minus + op, 0.0, req.offset_table_minus, req.offset_table_plus, req.grasping_angle);
    } else
        okk = crc->grasp_from_table_dual_plan(grasp_point_1, tip_1, grasp_point_2, tip_2, req.table_desk, trajectories, geometry_msgs::Pose(), geometry_msgs::Pose(),
                                              req.frame_id, req.offset_minus + op, 0.0, req.offset_table_minus, req.offset_table_plus, req.grasping_angle);
    if(!okk)
        return false;
    trajectories.pop_back(); //remove interpolation up

    robot_state::RobotState after_grasp_state(start_state);
    BOOST_FOREACH(const trajectory_msgs::JointTrajectory & t, trajectories) {
        after_grasp_state.setVariablePositions(t.joint_names, t.points.back().positions);
    }
    after_grasp_state.update();
    crc->setStartState(after_grasp_state);
    std::vector<std::string> elinks1, elinks2;
    elinks1.push_back("r1_gripper");
    elinks1.push_back("r2_gripper");
    elinks2.push_back(req.table_desk);
    elinks2.push_back(req.table_desk);
    if(!crc->computeGFold(final_point_1.position, tip_1, final_point_2.position, tip_2, gtraj, req.frame_id, elinks1, elinks2)) {
        return false;
    }

    trajectories.push_back(gtraj);
    after_grasp_state.setVariablePositions(gtraj.joint_names, gtraj.points.back().positions);
    after_grasp_state.update();

    trajectory_msgs::JointTrajectory traj_to_end_pose;
    crc->setStartState(after_grasp_state);
    if(!plan_to_end_pose(req, after_grasp_state, traj_to_end_pose, final_point_1, final_point_2, tip_1, tip_2, v, C_up, D_up, elinks1, elinks2))
        return false;
   
    trajectories.push_back(crc->gripper_trajectory(clopema_robot::GRIPPER_OPEN));
    trajectories.push_back(traj_to_end_pose);
    trajectories.push_back(crc->gripper_trajectory(clopema_robot::GRIPPER_CLOSE));

    publish_grasping_point(grasp_point_1, grasp_point_2, final_point_1, final_point_2, req.table_desk);
    return true;
}

bool grasp_and_fold_cb(clopema_robot::GraspAndFold::Request& req, clopema_robot::GraspAndFold::Response& res) {

    if(req.poses_1.empty()) { //make sure poses are stored in poses_1
        req.poses_1 = req.poses_2;
        req.poses_2.clear();
    }

    crc->setStartStateToCurrentState();
    robot_state::RobotState start_state(*crc->getCurrentState());
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);
    start_state.update();

    for(unsigned int i = 0; i < req.poses_1.size(); ++i) {
        ROS_INFO_STREAM("Pose number" << i << "/" << req.poses_1.size());
        std::vector<trajectory_msgs::JointTrajectory> trajectories;
        trajectory_msgs::JointTrajectory gtraj;
        if(!grasp_and_fold(req, i, start_state, trajectories, gtraj, true)) {
            if(!grasp_and_fold(req, i, start_state, trajectories, gtraj, false)) {
                continue;
            }
        }

        res.joint_trajectories.clear();
        res.joint_trajectories.insert(res.joint_trajectories.begin(), trajectories.begin(), trajectories.end());
        break;
    }

    robot_state::robotStateToRobotStateMsg(start_state, res.start_state);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_and_fold");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    crc.reset(new clopema_robot::ClopemaRobotCommander("arms"));
    ros::ServiceServer ser_grasp_and_fold = node.advertiseService("/clopema_planner/grasp_and_fold", grasp_and_fold_cb);

    ros::waitForShutdown();
    spinner.stop();
    return 0;
}


