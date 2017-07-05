/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: Nov 19, 2013
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Filter gripper mass and sensor offsets from the force data.
 */

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using geometry_msgs::WrenchStamped;
using geometry_msgs::Wrench;
using geometry_msgs::Vector3;

Wrench::Ptr offset;
Vector3::Ptr centroid;
double mass;

boost::shared_ptr<tf::TransformListener> listener;
boost::shared_ptr<ros::Publisher> pub_force;

/** \brief Callback method where wrench is filtered. */
void force_cb(const WrenchStamped::ConstPtr& msg);
/** \brief Load centroid and mass from urdf on param server. */
bool load_link_param(const std::string& link_name);
/** \brief Get link transform w.r.t. base_link */
bool get_link_transform(const std::string& link_name, const ros::Time& stamp,
        tf::StampedTransform& transform);

int main(int argc, char **argv) {
    ros::init(argc, argv, "filter_gripper");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);

    listener.reset(new tf::TransformListener);
    ros::Subscriber sub = node.subscribe("/in", 1, &force_cb);
    pub_force = boost::make_shared<ros::Publisher>(
            node.advertise<WrenchStamped>("/out", 1));

    ros::Duration(1.0).sleep(); //to buffer transformer
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}

void force_cb(const WrenchStamped::ConstPtr& msg) {
    if (!centroid) { //Load sensor parameters at the beginning
        load_link_param(msg->header.frame_id);
    }
    tf::StampedTransform T_bf;
    if (!get_link_transform(msg->header.frame_id, msg->header.stamp, T_bf))
        return;
    T_bf.setOrigin(tf::Vector3(0, 0, 0));

    static const double g = -9.80665;
    static const tf::Vector3 g_b(0, 0, g);
    static const tf::Vector3 r_f(centroid->x, centroid->y, centroid->z);

    tf::Vector3 f_f_predicted = T_bf * g_b * mass;
    tf::Vector3 m_f_predicted = (mass * r_f).cross(T_bf * g_b);

    tf::Vector3 f_f_measured, m_f_measured, foff, moff;
    tf::vector3MsgToTF(msg->wrench.force, f_f_measured);
    tf::vector3MsgToTF(msg->wrench.torque, m_f_measured);
    if (!offset) {
        offset.reset(new Wrench);
        foff = f_f_measured - f_f_predicted;
        moff = m_f_measured - m_f_predicted;
        tf::vector3TFToMsg(foff, offset->force);
        tf::vector3TFToMsg(moff, offset->torque);
        ROS_INFO_STREAM("Offset initialized to values: ");
        ROS_INFO_STREAM(*offset);

    } else {
        tf::vector3MsgToTF(offset->force, foff);
        tf::vector3MsgToTF(offset->torque, moff);
    }

    WrenchStamped out = *msg;
    tf::vector3TFToMsg(f_f_measured - foff - f_f_predicted, out.wrench.force);
    tf::vector3TFToMsg(m_f_measured - moff - m_f_predicted, out.wrench.torque);
    pub_force->publish(out);
}

bool get_link_transform(const std::string& link_name, const ros::Time& stamp,
        tf::StampedTransform& transform) {
    try {
        listener->waitForTransform(link_name, "/base_link", stamp,
                ros::Duration(0.5));
        listener->lookupTransform(link_name, "/base_link", stamp, transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    return true;
}

bool load_link_param(const std::string& link_name) {
    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR_STREAM(
                "Cannot parse robot_description from to parameters server");
        return false;
    }

    boost::shared_ptr<const urdf::Link> link = model.getLink(link_name);
    if (!link) {
        ROS_ERROR_STREAM("Link " << link_name << " was not found.");
        return false;
    }

    if (!link->inertial) {
        ROS_ERROR_STREAM("Link has no inertial");
        return false;
    }

    mass = link->inertial->mass;
    Vector3 c;
    c.x = link->inertial->origin.position.x;
    c.y = link->inertial->origin.position.y;
    c.z = link->inertial->origin.position.z;
    ROS_INFO_STREAM(link_name);
    ROS_INFO_STREAM(" - Mass: " << mass);
    ROS_INFO_STREAM("Centroid: " << c.x <<"," << c.y <<"," << c.z );
    centroid.reset(new Vector3(c));
    return true;
}
