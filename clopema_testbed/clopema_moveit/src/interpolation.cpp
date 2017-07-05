/**
 * Node providing services for collision aware path interpolation between
 * two poses specified by joints or by the cartesian pose.
 */
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <clopema_moveit/ClopemaJointInterpolation.h>
#include <clopema_moveit/ClopemaLinearInterpolation.h>
#include <clopema_moveit/ClopemaLinearInterpolationDual.h>
#include <kdl/kdl.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

planning_scene_monitor::PlanningSceneMonitorPtr sc;
double linear_step = 0.01;
double joint_step = 0.01;
double linear_rot_step = 0.01;
double max_joint_angle = 0.05;

/** \brief Resolve planning group based on ik link. */
std::string resolve_planning_group(const std::string& ik_link_name) {
    std::string group = "";
    if (strncmp(ik_link_name.c_str(), "r2", strlen("r2")) == 0) {
        group = "r2_arm";
    } else if (strncmp(ik_link_name.c_str(), "r1", strlen("r1")) == 0) {
        group = "r1_arm";
    } else if (strncmp(ik_link_name.c_str(), "xtion1", strlen("xtion1")) == 0) {
        group = "r1_xtion";
    } else if (strncmp(ik_link_name.c_str(), "xtion2", strlen("xtion2")) == 0) {
        group = "r2_xtion";
    }
    return group;
}

bool check_trajectory_for_collisions(
        const trajectory_msgs::JointTrajectory & traj, std::string & error) {
    moveit_msgs::RobotState start_state;
    robot_state::RobotState rs =
            sc->getPlanningScene()->getCurrentStateNonConst();
    robot_state::robotStateToRobotStateMsg(rs, start_state);
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory = traj;
    bool valid = sc->getPlanningScene()->isPathValid(start_state, trajectory,
            "", true);
    if (!valid) {
        error = "Collisions occurred on interpolated path";
    }
    return valid;
}

/** \brief Check trajectory for the density. Return false if it is not dense
 *  enough. */
bool check_trajectory_for_density(const trajectory_msgs::JointTrajectory & traj,
        std::string & error) {
    for (int i = 0; i < ((int) traj.points.size()) - 1; ++i) {
        for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
            if (fabs(
                    traj.points[i].positions[j]
                            - traj.points[i + 1].positions[j])
                    > max_joint_angle) {
                error = "Joint trajectory is not dense enough";
                return false;
            }
        }
    }
    return true;
}

/** \brief check trajectory for collision, density and joint limits.
 *  \param traj trajectory
 *  \param error output error (if any)
 */
bool check_trajectory(const trajectory_msgs::JointTrajectory & traj,
        std::string & error) {
    if (traj.points.empty()) {
        error = "Trajectory is empty";
        return false;
    }

    if (!check_trajectory_for_density(traj, error))
        return false;

    if (!check_trajectory_for_collisions(traj, error))
        return false;

    return true;
}

/** \brief Linear interpolation in joints coordinates.
 *  \param f start position
 *  \param g goal positions
 *  \param step maximum angle in the trajectory
 *  \param traj output trajectory (append to the actual)
 */
bool joints_interpolation(const sensor_msgs::JointState & f,
        const sensor_msgs::JointState & g, double step,
        trajectory_msgs::JointTrajectory & traj) {

    robot_state::RobotState rs =
            sc->getPlanningScene()->getCurrentStateNonConst();
    rs.setVariableValues(f);
    if (!rs.satisfiesBounds()) {
        ROS_ERROR_STREAM("[interpolation] Robot state start is out of bounds");
        return false;
    }
    rs.setVariableValues(g);
    if (!rs.satisfiesBounds()) {
        ROS_ERROR_STREAM("[interpolation] Robot state goal is out of bounds");
        return false;
    }

    //check whatever joints name are equal
    if (f.name.size() != g.name.size()) {
        ROS_ERROR_STREAM("[interpolation]  Joint names are not equal.");
        return false;
    }
    for (size_t i = 0; i < f.name.size(); ++i) {
        if (f.name[i] != g.name[i]) {
            ROS_ERROR_STREAM("[interpolation]  Joint names are not equal.");
            return false;
        }
    }

    //find maximum angle
    double maximum_angle_val = 0;
    for (size_t i = 0; i < f.position.size(); ++i) {
        if (fabs(g.position[i] - f.position[i]) > maximum_angle_val) {
            maximum_angle_val = fabs(g.position[i] - f.position[i]);
        }
    }

    traj.header.frame_id = f.header.frame_id;
    traj.header.stamp = ros::Time::now();
    traj.joint_names = f.name;

    if (maximum_angle_val == 0) {
        return true;
    }

    double t_step = step / maximum_angle_val;
    for (double t = 0; t <= 1.0; t += t_step) {
        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.resize(f.position.size());
        p.velocities.resize(f.position.size());
        p.accelerations.resize(f.position.size());

        for (size_t i = 0; i < f.position.size(); ++i) {
            p.positions[i] = f.position[i]
                    + (g.position[i] - f.position[i]) * t;
        }

        traj.points.push_back(p);
    }
    return true;
}

/** \brief Transform poses to the joints using IKT plugin
 *  \param poses poses which should be transformed
 *  \param joints vector of joint states corresponding to the poses
 */
bool poses_to_joints(const std::vector<tf::Pose> & poses,
        const std::string & ik_base_name, const std::string & ik_link_name,
        trajectory_msgs::JointTrajectory & traj) {

    robot_model::RobotModelConstPtr kinematic_model = sc->getRobotModel();
    if (!kinematic_model) {
        ROS_ERROR_STREAM(
                "Can not load kinematic model, is robot description loaded?");
        return false;
    }
    robot_state::RobotStatePtr kinematic_state(
            new robot_state::RobotState(kinematic_model));
    kinematic_state->updateLinkTransforms();
    Eigen::Affine3d to_base = kinematic_state->getFrameTransform(ik_base_name);

    traj.joint_names = kinematic_state->getVariableNames();
    traj.points.clear();
    traj.header.frame_id = kinematic_model->getModelFrame();

    for (int i = 0; i < poses.size(); ++i) {
        Eigen::Affine3d pose;
        tf::poseTFToEigen(poses[i], pose);
        pose = to_base * pose;
        if (!kinematic_state->setFromIK(
                kinematic_state->getJointModelGroup(
                        resolve_planning_group(ik_link_name)), pose,
                ik_link_name, 10, 0.1)) {
            ROS_ERROR_STREAM(
                    "Can not set IK from pose: " << pose.matrix() << " for ik_link_name: " << ik_link_name);
            return false;
        }
        trajectory_msgs::JointTrajectoryPoint point;
        point.accelerations.resize(kinematic_state->getVariableCount(), 0);
        point.velocities.resize(kinematic_state->getVariableCount(), 0);
        point.positions.resize(kinematic_state->getVariableCount(), 0);
        for (unsigned int j = 0; j < kinematic_state->getVariableCount(); ++j) {
            point.positions[j] = kinematic_state->getVariablePosition(j);
        }
        traj.points.push_back(point);
        std::string error;
        if (!check_trajectory_for_density(traj, error)) {
            ROS_ERROR_STREAM(error << "  " << i << "/" << poses.size());
            return false;
        }
    }

    return true;
}

/** \brief Transform poses to the joints using IKT plugin
 *  \param poses poses which should be transformed
 *  \param joints vector of joint states corresponding to the poses
 */
bool poses_to_joints_dual(const std::vector<tf::Pose> & poses,
        const std::vector<tf::Pose> & poses2, const std::string & ik_base_name,
        const std::string & ik_link_name, const std::string & ik_link_name2,
        trajectory_msgs::JointTrajectory & traj) {

    std::string group = "";
    if (ik_link_name.substr(0, 1) == "r") {
        group = "arms";
    } else if (ik_link_name.substr(0, 1) == "x") {
        group = "xtions";
    } else {
        ROS_ERROR("Invalid link names, unknown group.");
        return false;
    }

    if (poses.size() != poses2.size()) {
        ROS_ERROR("Number of poses must be equal.");
        return false;
    }

    robot_model::RobotModelConstPtr kinematic_model = sc->getRobotModel();
    if (!kinematic_model) {
        ROS_ERROR_STREAM(
                "Can not load kinematic model, is robot description loaded?");
        return false;
    }
    robot_state::RobotStatePtr kinematic_state(
            new robot_state::RobotState(kinematic_model));
    kinematic_state->updateLinkTransforms();
    Eigen::Affine3d to_base = kinematic_state->getFrameTransform(ik_base_name);

    traj.joint_names = kinematic_state->getVariableNames();
    traj.points.clear();
    traj.header.frame_id = kinematic_model->getModelFrame();

    for (int i = 0; i < poses.size(); ++i) {
        {
            Eigen::Affine3d pose;
            tf::poseTFToEigen(poses[i], pose);
            pose = to_base * pose;
            if (!kinematic_state->setFromIK(
                    kinematic_state->getJointModelGroup(
                            resolve_planning_group(ik_link_name)), pose,
                    ik_link_name, 10, 0.1)) {
                ROS_ERROR_STREAM(
                        "Can not set IK from pose: " << pose.matrix() << " for ik_link_name: " << ik_link_name);
                return false;
            }
        }
        kinematic_state->updateLinkTransforms();
        {
            Eigen::Affine3d pose;
            tf::poseTFToEigen(poses2[i], pose);
            pose = to_base * pose;
            if (!kinematic_state->setFromIK(
                    kinematic_state->getJointModelGroup(
                            resolve_planning_group(ik_link_name)), pose,
                    ik_link_name, 10, 0.1)) {
                ROS_ERROR_STREAM(
                        "Can not set IK from pose: " << pose.matrix() << " for ik_link_name: " << ik_link_name2);
                return false;
            }
        }
        kinematic_state->updateLinkTransforms();
        trajectory_msgs::JointTrajectoryPoint point;
        point.accelerations.resize(kinematic_state->getVariableCount(), 0);
        point.velocities.resize(kinematic_state->getVariableCount(), 0);
        point.positions.resize(kinematic_state->getVariableCount(), 0);
        for (unsigned int j = 0; j < kinematic_state->getVariableCount(); ++j) {
            point.positions[j] = kinematic_state->getVariablePosition(j);
        }
        traj.points.push_back(point);
        std::string error;
        if (!check_trajectory_for_density(traj, error)) {
            ROS_ERROR_STREAM(error << "  " << i << "/" << poses.size());
            return false;
        }
    }
    return true;
}

/** \brief Compute set of poses between f and g pose
 *  \param f start pose
 *  \param g goal pose
 *  \param step Length of the step in meters.
 *  \param poses Interpolated poses are pushed to this vector
 */
bool linear_interpolation(const tf::Pose & f, const tf::Pose & g,
        double step_lin, double step_rot, std::vector<tf::Pose> & poses) {
    double d = (f.getOrigin() - g.getOrigin()).length();
    double angle = f.getRotation().angleShortestPath(g.getRotation());

    if ((d == 0) && (angle == 0)) {
        tf::Pose pose;
        pose.setRotation(f.getRotation().slerp(g.getRotation(), 1.0));
        pose.setOrigin(f.getOrigin().lerp(g.getOrigin(), 1.0));
        poses.push_back(pose);
        return true;
    }

    //pick the smaller step for the interpolation
    double t_step = step_lin / d;
    double r_step = step_rot / angle;
    double final_step = r_step;
    if (t_step < r_step)
        final_step = t_step;

    for (double t = 0; t <= 1.0; t += final_step) {
        tf::Pose pose;
        pose.setRotation(f.getRotation().slerp(g.getRotation(), t));
        pose.setOrigin(f.getOrigin().lerp(g.getOrigin(), t));
        poses.push_back(pose);
    }

    {
        tf::Pose pose;
        pose.setRotation(f.getRotation().slerp(g.getRotation(), 1.0));
        pose.setOrigin(f.getOrigin().lerp(g.getOrigin(), 1.0));
        poses.push_back(pose);
    }

    return true;
}

/** \brief Compute set of poses between f and g pose
 *  \param f start poses
 *  \param g goal poses
 *  \param step Length of the step in meters.
 *  \param poses Interpolated poses are pushed to this vector
 */
bool linear_interpolation_dual(const tf::Pose & f, const tf::Pose & g,
        double step_lin, double step_rot, std::vector<tf::Pose> & poses,
        const tf::Pose & f2, const tf::Pose & g2,
        std::vector<tf::Pose> & poses2) {
    double d = (f.getOrigin() - g.getOrigin()).length();
    double d2 = (f2.getOrigin() - g2.getOrigin()).length();
    if (d < d2)
        d = d2;

    double angle = f.getRotation().angleShortestPath(g.getRotation());
    double angle2 = f2.getRotation().angleShortestPath(g2.getRotation());
    if (angle < angle2)
        angle = angle2;

    if ((d == 0) && (angle == 0)) {
        tf::Pose pose;
        pose.setRotation(f.getRotation().slerp(g.getRotation(), 1.0));
        pose.setOrigin(f.getOrigin().lerp(g.getOrigin(), 1.0));
        poses.push_back(pose);

        tf::Pose pose2;
        pose2.setRotation(f2.getRotation().slerp(g2.getRotation(), 1.0));
        pose2.setOrigin(f2.getOrigin().lerp(g2.getOrigin(), 1.0));
        poses2.push_back(pose2);
        return true;
    }

    //pick the smaller step for the interpolation
    double t_step = step_lin / d;
    double r_step = step_rot / angle;
    double final_step = r_step;
    if (t_step < r_step)
        final_step = t_step;

    for (double t = 0; t <= 1.0; t += final_step) {
        tf::Pose pose;
        pose.setRotation(f.getRotation().slerp(g.getRotation(), t));
        pose.setOrigin(f.getOrigin().lerp(g.getOrigin(), t));
        poses.push_back(pose);

        tf::Pose pose2;
        pose2.setRotation(f2.getRotation().slerp(g2.getRotation(), t));
        pose2.setOrigin(f2.getOrigin().lerp(g2.getOrigin(), t));
        poses2.push_back(pose2);
    }

    {
        tf::Pose pose;
        pose.setRotation(f.getRotation().slerp(g.getRotation(), 1.0));
        pose.setOrigin(f.getOrigin().lerp(g.getOrigin(), 1.0));
        poses.push_back(pose);

        tf::Pose pose2;
        pose2.setRotation(f2.getRotation().slerp(g2.getRotation(), 1.0));
        pose2.setOrigin(f2.getOrigin().lerp(g2.getOrigin(), 1.0));
        poses2.push_back(pose2);
    }

    return true;
}

bool joint_interpolation_cb(
        clopema_moveit::ClopemaJointInterpolationRequest & req,
        clopema_moveit::ClopemaJointInterpolationResponse & res) {
    ROS_WARN_STREAM("Joint interpolation service is deprecated, use ClopemaRobot alternative.");

    res.joint_trajectory.points.clear();
    res.joint_trajectory.joint_names.clear();

    if (req.poses.size() < 2) {
        res.error = "Not enough poses.";
        return true;
    }
    for (size_t i = 0; i < req.poses.size() - 1; ++i) {
        if (!joints_interpolation(req.poses[i], req.poses[i + 1], joint_step,
                res.joint_trajectory)) {
            res.error = "Joints can't be interpolated.";
            return true;
        }
    }

    if (!check_trajectory(res.joint_trajectory, res.error)) {
        res.joint_trajectory.points.clear();
        res.error = "Collision occurred.";
        return true;
    }

    return true;
}

bool linear_interpolation_cb(
        clopema_moveit::ClopemaLinearInterpolationRequest & req,
        clopema_moveit::ClopemaLinearInterpolationResponse & res) {

    ROS_WARN_STREAM("Linear interpolation service is deprecated, use ClopemaRobot - cartesian path alternative.");
    if (req.ik_link_name.empty()) {
        res.error = "IK link not specified";
        return true;
    }
    if (req.header.frame_id.empty()) {
        res.error = "Frame_id not specified";
        return true;
    }

    res.joint_trajectory.points.clear();
    res.joint_trajectory.joint_names.clear();

    std::vector<tf::Pose> poses;
    for (size_t i = 0; i < req.poses.size() - 1; ++i) {
        tf::Pose f, g;
        tf::Quaternion q;
        f.setOrigin(
                tf::Vector3(req.poses[i].position.x, req.poses[i].position.y,
                        req.poses[i].position.z));
        g.setOrigin(
                tf::Vector3(req.poses[i + 1].position.x,
                        req.poses[i + 1].position.y,
                        req.poses[i + 1].position.z));
        tf::quaternionMsgToTF(req.poses[i].orientation, q);
        f.setRotation(q);
        tf::quaternionMsgToTF(req.poses[i + 1].orientation, q);
        g.setRotation(q);

        if (!linear_interpolation(f, g, linear_step, linear_rot_step, poses)) {
            res.error = "Poses can't be interpolated.";
            return true;
        }
    }

    if (!poses_to_joints(poses, req.header.frame_id, req.ik_link_name,
            res.joint_trajectory)) {
        res.error =
                "Can't find constraint aware IKT for all interpolated poses.";
        return true;
    }

    if (!check_trajectory(res.joint_trajectory, res.error)) {
        res.joint_trajectory.points.clear();
        return true;
    }

    return true;
}

bool linear_interpolation_dual_cb(
        clopema_moveit::ClopemaLinearInterpolationDualRequest & req,
        clopema_moveit::ClopemaLinearInterpolationDualResponse & res) {
    ROS_WARN_STREAM("Linear interpolation service is deprecated, use ClopemaRobot - cartesian path alternative.");

    if (req.ik_link_name_1.empty() || req.ik_link_name_2.empty()) {
        res.error = "IK link not specified";
        return true;
    }
    if (req.header.frame_id.empty()) {
        res.error = "Frame_id not specified";
        return true;
    }

    if (req.poses_1.size() != req.poses_2.size()) {
        res.error = "Number of poses must be equal.";
        return true;
    }

    res.joint_trajectory.points.clear();
    res.joint_trajectory.joint_names.clear();

    std::vector<tf::Pose> poses;
    std::vector<tf::Pose> poses2;
    for (int i = 0; i < ((int) req.poses_1.size()) - 1; ++i) {
        tf::Pose f, g, f2, g2;
        tf::Quaternion q;
        f.setOrigin(
                tf::Vector3(req.poses_1[i].position.x,
                        req.poses_1[i].position.y, req.poses_1[i].position.z));
        g.setOrigin(
                tf::Vector3(req.poses_1[i + 1].position.x,
                        req.poses_1[i + 1].position.y,
                        req.poses_1[i + 1].position.z));
        tf::quaternionMsgToTF(req.poses_1[i].orientation, q);
        f.setRotation(q);
        tf::quaternionMsgToTF(req.poses_1[i + 1].orientation, q);
        g.setRotation(q);

        f2.setOrigin(
                tf::Vector3(req.poses_2[i].position.x,
                        req.poses_2[i].position.y, req.poses_2[i].position.z));
        g2.setOrigin(
                tf::Vector3(req.poses_2[i + 1].position.x,
                        req.poses_2[i + 1].position.y,
                        req.poses_2[i + 1].position.z));
        tf::quaternionMsgToTF(req.poses_2[i].orientation, q);
        f2.setRotation(q);
        tf::quaternionMsgToTF(req.poses_2[i + 1].orientation, q);
        g2.setRotation(q);

        if (!linear_interpolation_dual(f, g, linear_step, linear_rot_step,
                poses, f2, g2, poses2)) {
            res.error = "Poses can't be interpolated.";
            return true;
        }
    }

    if (!poses_to_joints_dual(poses, poses2, req.header.frame_id,
            req.ik_link_name_1, req.ik_link_name_2, res.joint_trajectory)) {
        res.error =
                "Can't find constraint aware IKT for all interpolated poses.";
        return true;
    }

    if (!check_trajectory(res.joint_trajectory, res.error)) {
        res.joint_trajectory.points.clear();
        return true;
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "plan_interpolation");
    ros::NodeHandle n("~");

    n.param<double>("max_joint_angle", max_joint_angle, 0.05);
    n.param<double>("linear_step", linear_step, 0.01);
    n.param<double>("joint_step", joint_step, 0.01);
    n.param<double>("linear_rot_step", linear_rot_step, 0.01);

    sc.reset(
            new planning_scene_monitor::PlanningSceneMonitor(
                    "robot_description"));
    sc->startSceneMonitor();
    sc->startStateMonitor();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceServer service_j = n.advertiseService(
            "/clopema_planner/joint_interpolation", joint_interpolation_cb);
    ros::ServiceServer service_l = n.advertiseService(
            "/clopema_planner/linear_interpolation", linear_interpolation_cb);
    ros::ServiceServer service_ld = n.advertiseService(
            "/clopema_planner/linear_interpolation_dual",
            linear_interpolation_dual_cb);
    ros::waitForShutdown();
    sc.reset();

    return 0;
}
