/*
 *  Created on: Sep 9, 2013
 *      Author: Vladimir Petrik
 */

#include <clopema_libs/clopema_pose_generator.h>

namespace clopema_pose_generator {

Pose orient_pose(const Pose& pose, const Point& target) {
    return orient_pose(pose.position, target);
}

Pose orient_pose(const Point& pose, const Point& target) {
    tf::Vector3 center, orig;
    tf::pointMsgToTF(pose, orig);
    tf::pointMsgToTF(target, center);
    tf::Transform tr = tf::Transform::getIdentity();
    double angle = (orig - center).angle(tf::Vector3(0, 0, -1));
    tf::Vector3 rotVec = (center - orig).cross(tf::Vector3(0, 0, -1));

    if (rotVec.isZero()) {
        rotVec = tf::Vector3(-1, 0, 0);
    }
    tr.setRotation(tf::Quaternion(rotVec, angle));

    tr.setOrigin(orig);

    Pose p;
    tf::poseTFToMsg(tr, p);
    return p;
}

vector<Pose> gen_pose_grid_target(const vector<double>& x_range,
        const vector<double>& y_range, const vector<double>& z_range,
        const Point& target) {
    vector<Pose> poses;
    for (unsigned int ix = 0; ix < x_range.size(); ++ix) {
        for (unsigned int iy = 0; iy < y_range.size(); ++iy) {
            for (unsigned int iz = 0; iz < z_range.size(); ++iz) {
                geometry_msgs::Point p;
                p.x = x_range[ix];
                p.y = y_range[iy];
                p.z = z_range[iz];
                poses.push_back(orient_pose(p, target));
            }
        }
    }

    return poses;
}

vector<Pose> gen_pose_grid(const vector<double>& x_range,
        const vector<double>& y_range, const vector<double>& z_range,
        const vector<Quaternion>& orientation) {
    vector<Pose> poses;
    for (unsigned int i = 0; i < orientation.size(); ++i) {
        std::vector<geometry_msgs::Pose> tmp;
        tmp = gen_pose_grid(x_range, y_range, z_range, orientation[i]);
        poses.insert(poses.end(), tmp.begin(), tmp.end());
    }
    return poses;
}

/** \brief Generate grid of poses with fixed orientation */
vector<Pose> gen_pose_grid(const vector<double>& x_range,
        const vector<double>& y_range, const vector<double>& z_range,
        const Quaternion& orientation) {
    vector<Pose> poses;
    for (unsigned int ix = 0; ix < x_range.size(); ++ix) {
        for (unsigned int iy = 0; iy < y_range.size(); ++iy) {
            for (unsigned int iz = 0; iz < z_range.size(); ++iz) {
                geometry_msgs::Point p;
                p.x = x_range[ix];
                p.y = y_range[iy];
                p.z = z_range[iz];
                geometry_msgs::Pose pose;
                pose.position = p;
                pose.orientation = orientation;
                poses.push_back(pose);
            }
        }
    }

    return poses;
}

/** \brief Generate poses using spherical coordinates. */
vector<Pose> gen_pose_sphere(const vector<double>& radius,
        vector<double>& theta, const vector<double>& phi,
        const Quaternion& orientation, bool generate_zero_orientation) {
    vector<Pose> poses;
    for (unsigned int iy = 0; iy < theta.size(); ++iy) {
        for (unsigned int iz = 0; iz < phi.size(); ++iz) {
            for (unsigned int ix = 0; ix < radius.size(); ++ix) {
                double r = radius[ix];
                double t = theta[iy];
                double p = phi[iz];
                geometry_msgs::Point point;
                point.x = r * sin(t) * cos(p);
                point.y = r * sin(t) * sin(p);
                point.z = r * cos(t);

                geometry_msgs::Pose pose;
                pose.position = point;
                pose.orientation = orientation;
                poses.push_back(pose);
            }
        }
    }
    if (generate_zero_orientation) {
        for (unsigned int ix = 0; ix < radius.size(); ++ix) {
            double r = radius[ix];
            geometry_msgs::Point point;
            point.x = 0;
            point.y = 0;
            point.z = r;

            geometry_msgs::Pose pose;
            pose.position = point;
            pose.orientation = orientation;
            poses.push_back(pose);
        }
    }

    return poses;
}

/** \brief Generate poses using sphere coordinates. */
vector<Pose> gen_pose_sphere_target(const vector<double>& radius,
        vector<double>& theta, const vector<double>& phi, const Point& target,
        bool generate_zero_orientation) {
    vector<Pose> poses;
    for (unsigned int iy = 0; iy < theta.size(); ++iy) {
        for (unsigned int iz = 0; iz < phi.size(); ++iz) {
            for (unsigned int ix = 0; ix < radius.size(); ++ix) {
                double r = radius[ix];
                double t = theta[iy];
                double p = phi[iz];
                geometry_msgs::Point point;
                const double eps = 0.000001;
                point.x = r * sin(t) * cos(p);
                point.y = r * sin(t) * sin(p);
                point.z = r * cos(t);

                poses.push_back(orient_pose(point, target));
            }
        }
    }

    if (generate_zero_orientation) {
        for (unsigned int ix = 0; ix < radius.size(); ++ix) {
            double r = radius[ix];
            geometry_msgs::Point point;
            point.x = 0;
            point.y = 0;
            point.z = r;

            poses.push_back(orient_pose(point, target));
        }
    }

    return poses;
}

void publish_poses(const vector<Pose>& poses, const std::string& prefix,
        const std::string& base_frame, const ros::Time& time) {
    static tf::TransformBroadcaster br;
    ros::Duration(0.5).sleep();
    for (unsigned int i = 0; i < poses.size(); ++i) {
        std::stringstream ss;
        ss << prefix << i;
        tf::Transform pose;
        tf::poseMsgToTF(poses[i], pose);
        br.sendTransform(
                tf::StampedTransform(pose, time, base_frame, ss.str()));
    }
}

void convert_msg_to_tf(const vector<Pose>& poses,
        vector<tf::Transform>& tf_poses) {
    tf_poses.resize(poses.size());
    for (unsigned int i = 0; i < poses.size(); ++i) {
        tf::Transform f;
        tf::poseMsgToTF(poses[i], f);
        tf_poses[i] = f;
    }
}

void convert_tf_to_msg(const vector<tf::Transform>& tf_poses,
        vector<Pose>& poses) {
    poses.resize(tf_poses.size());
    for (unsigned int i = 0; i < tf_poses.size(); ++i) {
        geometry_msgs::Pose p;
        tf::poseTFToMsg(tf_poses[i], p);
        poses[i] = p;
    }
}

void publish_poses_as_markers(const vector<Pose>& poses,
        const std::string& prefix, const std::string& base_frame) {
    ros::NodeHandle node("~");
    ros::Publisher markers_publisher = node.advertise<
            visualization_msgs::MarkerArray>("poses_visualization", 1);
    ros::Duration(0.5).sleep();

    visualization_msgs::MarkerArray markers;
    for (unsigned int i = 0; i < poses.size(); ++i) {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = base_frame;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = prefix;
        marker.id = i;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the marker action.  Options are ADD and DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose = poses[i];

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.001;
        marker.scale.y = 0.01;
        marker.scale.z = 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        markers.markers.push_back(marker);
    }
    markers_publisher.publish(markers);

}

} /* namespace clopema_pose_generator */
