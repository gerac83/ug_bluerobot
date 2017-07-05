// Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
//
// Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
// Author:      Vladimir Petrik <petrivl3@felk.cvut.cz>
// Institute:   Czech Technical University in Prague
// Created on:  Feb 12, 2014


#ifndef CLOPEMA_LIBS_TF_UTILS_H_
#define CLOPEMA_LIBS_TF_UTILS_H_

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>

// Simpli publish poses to TF
namespace tf
{

    Eigen::Vector3d transform_position(robot_state::RobotState rs,
                                       Eigen::Vector3d source_position,
                                       std::string source_frame,
                                       std::string target_frame);

    std::vector<Eigen::Vector3d> transform_position(robot_state::RobotState rs,
                                                    std::vector<Eigen::Vector3d> source_positions,
                                                    std::string source_frame,
                                                    std::string target_frame);
        /**
     * \brief Publish poses int tf.
     */
    void publish_poses(const std::vector<geometry_msgs::Pose>& poses,
                       const std::string& prefix,
                       const std::string& base_frame,
                       const ros::Time& time);

} // End tf


#endif
