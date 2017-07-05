// Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
//
// Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
// Author:      Vladimir Petrik <petrivl3@felk.cvut.cz>
// Institute:   Czech Technical University in Prague
// Created on:  Feb 12, 2014

#include <clopema_libs/tf_utils.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>

namespace tf
{

Eigen::Vector3d transform_position(robot_state::RobotState rs,
                        Eigen::Vector3d source_position,
                        std::string source_frame,
                        std::string target_frame)
{
    Eigen::Affine3d trans_target = rs.getFrameTransform(target_frame);
    Eigen::Affine3d trans_source = rs.getFrameTransform(source_frame);
    Eigen::Affine3d trans_target_to_source = trans_target.inverse() * trans_source;

    return trans_target_to_source.inverse() * source_position;
}

std::vector<Eigen::Vector3d> transform_position(robot_state::RobotState rs,
                        std::vector<Eigen::Vector3d> source_positions,
                        std::string source_frame,
                        std::string target_frame)
{
    Eigen::Affine3d trans_target = rs.getFrameTransform(target_frame);
    Eigen::Affine3d trans_source = rs.getFrameTransform(source_frame);
    Eigen::Affine3d trans_target_to_source = trans_target.inverse() * trans_source;


    std::vector<Eigen::Vector3d> target_positions;
    BOOST_FOREACH(Eigen::Vector3d & p, source_positions) {
        target_positions.push_back(trans_target_to_source.inverse() * p);
    }

    return target_positions;
}

void publish_poses(const std::vector<geometry_msgs::Pose>& poses,
                   const std::string& prefix,
                   const std::string& base_frame,
                   const ros::Time& time)
{
    static TransformBroadcaster br;
    ros::Duration(0.5).sleep();
    for (unsigned int i = 0; i < poses.size(); ++i) {
        std::stringstream ss;
        ss << prefix << i;
        tf::Transform pose;
        tf::poseMsgToTF(poses[i], pose);
        br.sendTransform(tf::StampedTransform(pose, time, base_frame, ss.str()));
    }
}



} // End tf
