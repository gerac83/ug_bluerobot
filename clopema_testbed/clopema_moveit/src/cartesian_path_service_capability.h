/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: Nov 1, 2013
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 */

#ifndef MOVEIT_MOVE_GROUP_CARTESIAN_PATH_SERVICE_DUAL_CAPABILITY_
#define MOVEIT_MOVE_GROUP_CARTESIAN_PATH_SERVICE_DUAL_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <clopema_moveit/GetCartesianPathDual.h>

namespace move_group {

using namespace robot_state;
static const std::string CARTESIAN_PATH_SERVICE_NAME =
        "compute_cartesian_path_dual"; // name of the service that computes cartesian paths

class MoveGroupCartesianPathServiceDual: public MoveGroupCapability {
public:

    MoveGroupCartesianPathServiceDual();

    virtual void initialize();

    bool computeService(clopema_moveit::GetCartesianPathDual::Request &req,
            clopema_moveit::GetCartesianPathDual::Response &res);
private:
    bool transform_poses(EigenSTL::vector_Affine3d& waypoints,
            std::vector<geometry_msgs::Pose> waypoints_in,
            clopema_moveit::GetCartesianPathDual::Request& req,
            const std::string& link_name);

    double computeCartesianPath(const RobotStatePtr& rs,
            const JointModelGroup *group, std::vector<RobotStatePtr> &traj,
            const LinkModel *link1, const JointModelGroup *jmg1,
            const LinkModel *link2, const JointModelGroup *jmg2,
            const Eigen::Affine3d &target1, const Eigen::Affine3d &target2,
            bool global_reference_frame, double max_step, double jump_threshold,
            const GroupStateValidityCallbackFn &validCallback =
                    GroupStateValidityCallbackFn(),
            const kinematics::KinematicsQueryOptions &options =
                    kinematics::KinematicsQueryOptions());

    double computeCartesianPath(const RobotStatePtr& rs,
            const JointModelGroup *group, std::vector<RobotStatePtr> &traj,
            const LinkModel *link1, const JointModelGroup *jmg1,
            const LinkModel *link2, const JointModelGroup *jmg2,
            const EigenSTL::vector_Affine3d &waypoints1, const EigenSTL::vector_Affine3d &waypoints2,
            bool global_reference_frame, double max_step, double jump_threshold,
            const GroupStateValidityCallbackFn &validCallback =
                    GroupStateValidityCallbackFn(),
            const kinematics::KinematicsQueryOptions &options =
                    kinematics::KinematicsQueryOptions());

    ros::ServiceServer cartesian_path_service_;
    ros::Publisher display_path_;
    bool display_computed_paths_;
};

}

#endif
