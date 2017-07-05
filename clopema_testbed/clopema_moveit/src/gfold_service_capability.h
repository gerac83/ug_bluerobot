/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: Nov 1, 2013
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 */

#ifndef MOVEIT_MOVE_GROUP_GFOLD_SERVICE_CAPABILITY_
#define MOVEIT_MOVE_GROUP_GFOLD_SERVICE_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <clopema_moveit/GetCartesianPathDual.h>
#include <clopema_moveit/GFold.h>
#include <kdl/frames.hpp>

namespace move_group {

using namespace robot_state;
using geometry_msgs::Pose;
using KDL::Vector;
using std_msgs::Header;
static const std::string GFOLD_SERVICE_NAME = "/compute_gfold";
const std::string SRV_CARTESIAN_PATH_SERVICE_DUAL_NAME =
        "/compute_cartesian_path_dual";

class GFoldService: public MoveGroupCapability {
public:

    GFoldService();

    virtual void initialize();

private:
    /** GFold problem which should be solved */
    struct Problem {
        Pose start_pose_1, start_pose_2;
        Pose final_pose_1, final_pose_2;
        std::string final_pose_1_link, final_pose_2_link;
        Vector pa, pb, g11, g21, g12, g22, g13, g23, g14, g24, g15, g25;
        Header header;
    };

    /** \brief service callback function */
    bool computeService(clopema_moveit::GFold::Request& req,
            clopema_moveit::GFold::Response &res);

    /** \brief transform pose to the tip frame */
    void transform_to_tip(const robot_state::RobotState& rs,
            geometry_msgs::Pose& pose, const std::string& frame,
            const std::string& tip);

    /** \brief transform point to the tip frame */
    void transform_to_tip(const robot_state::RobotState& rs,
            geometry_msgs::Point& point, const std::string& frame,
            const std::string& tip);

    /** \brief Compute gfold poses - i.e. solve the problem */
    bool compute_gfold_poses(Problem& prob);

    bool interpolate_gfold(const Problem& problem,
            const robot_state::RobotState& state,
            trajectory_msgs::JointTrajectory& traj, int arm,
            const std::vector<std::string>& allow_collision_1, const std::vector<std::string>& allow_collision_2);

    ros::ServiceServer gfold_service_;
    ros::Publisher display_path_;
    bool display_computed_paths_;

};

}

#endif
