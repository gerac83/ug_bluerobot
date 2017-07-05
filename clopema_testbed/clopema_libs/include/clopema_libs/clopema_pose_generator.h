/*  Useful functions for generating poses.
 *
 *  Created on: Sep 9, 2013
 *      Author: Petrik Vladimir
 */

#ifndef CLOPEMAPOSEGENERATOR_H_
#define CLOPEMAPOSEGENERATOR_H_

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace clopema_pose_generator {

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using std::vector;

/** \brief Make Z axis of given pose oriented towards target point. */
Pose orient_pose(const Pose& pose, const Point& target);

/** \brief Make Z axis of given pose oriented towards target point. */
Pose orient_pose(const Point& pose, const Point& target);

/** \brief Generate grid of poses with z axis pointing to the given target */
vector<Pose> gen_pose_grid_target(const vector<double>& x_range,
        const vector<double>& y_range, const vector<double>& z_range,
        const Point& target);

/** \brief Generate grid of poses with fixed orientation */
vector<Pose> gen_pose_grid(const vector<double>& x_range,
        const vector<double>& y_range, const vector<double>& z_range,
        const Quaternion& orientation);

/** \brief Generate grid of poses with fixed orientation */
vector<Pose> gen_pose_grid(const vector<double>& x_range,
        const vector<double>& y_range, const vector<double>& z_range,
        const vector<Quaternion>& orientation);

/** \brief Generate poses using spherical coordinates.*/
vector<Pose> gen_pose_sphere(const vector<double>& radius,
        vector<double>& theta, const vector<double>& phi,
        const Quaternion& orientation, bool generate_zero_orientation = false);

/** \brief Generate poses using sphere coordinates.*/
vector<Pose> gen_pose_sphere_target(const vector<double>& radius,
        vector<double>& theta, const vector<double>& phi, const Point& target,
        bool generate_zero_orientation = false);

/** Broadcast poses to the tf */
void publish_poses(const vector<Pose>& poses, const std::string& prefix,
        const std::string& base_frame, const ros::Time& time = ros::Time::now());

void convert_msg_to_tf(const vector<Pose>& poses,
        vector<tf::Transform>& tf_poses);
void convert_tf_to_msg(const vector<tf::Transform>& tf_poses,
        vector<Pose>& poses);

void publish_poses_as_markers(const vector<Pose>& poses,
        const std::string& prefix, const std::string& base_frame);

} /* namespace clopema_pose_generator */
#endif /* CLOPEMAPOSEGENERATOR_H_ */
