/**
* Copyright (c) CTU in Prague  - All Rights Reserved
* Created on: Oct 15, 2013
*     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
*  Institute: Czech Technical University in Prague
*/

#ifndef CLOPEMA_ARM_KINEMATICS_PLUGIN_H
#define CLOPEMA_ARM_KINEMATICS_PLUGIN_H

#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "../../src/clopema_ik_universal.cpp"
#include <clopema_kinematics/ClopemaGetAllIkt.h>

class ClopemaArmKinematicsPlugin: public kinematics::KinematicsBase {
public:

    /** Constructor */
    ClopemaArmKinematicsPlugin() :
            ik_solver_(0) {
    }

    /** Desctructor */
    ~ClopemaArmKinematicsPlugin() {
        if (ik_solver_)
            delete ik_solver_;
    }

    bool getAllIKT_cb(clopema_kinematics::ClopemaGetAllIktRequest& req, clopema_kinematics::ClopemaGetAllIktResponse& res);

    bool getPositionsIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, std::vector<std::vector<double> > &solutions, moveit_msgs::MoveItErrorCodes& error_code) const;

    /**
     * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param solution the solution vector
     * @param error_code an error code that encodes the reason for failure or success
     * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code, const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param timeout The amount of time (in seconds) available to the solver
     * @param solution the solution vector
     * @param error_code an error code that encodes the reason for failure or success
     * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout, std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code, const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param timeout The amount of time (in seconds) available to the solver
     * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
     * @param solution the solution vector
     * @param error_code an error code that encodes the reason for failure or success
     * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout, const std::vector<double> &consistency_limits, std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code, const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param timeout The amount of time (in seconds) available to the solver
     * @param solution the solution vector
     * @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
     * @param solution_callback A callback solution for the IK solution
     * @param error_code an error code that encodes the reason for failure or success
     * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout, std::vector<double> &solution, const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code, const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param timeout The amount of time (in seconds) available to the solver
     * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
     * @param solution the solution vector
     * @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
     * @param solution_callback A callback solution for the IK solution
     * @param error_code an error code that encodes the reason for failure or success
     * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout, const std::vector<double> &consistency_limits, std::vector<double> &solution, const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose
     * @param link_names A set of links for which FK needs to be computed
     * @param joint_angles The state for which FK is being computed
     * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
     * @return True if a valid solution was found, false otherwise
     */
    virtual bool getPositionFK(const std::vector<std::string> &link_names, const std::vector<double> &joint_angles, std::vector<geometry_msgs::Pose> &poses) const;

    /**
     * @brief  Initialization function for the kinematics
     * @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for; For example, rhe name of the ROS parameter that contains the robot description;
     * @param group_name The group for which this solver is being configured
     * @param base_frame The base frame in which all input poses are expected.
     * This may (or may not) be the root frame of the chain that the solver operates on
     * @param tip_frame The tip of the chain
     * @param search_discretization The discretization of the search when the solver steps through the redundancy
     * @return True if initialization was successful, false otherwise
     */
    virtual bool initialize(const std::string& robot_description, const std::string& group_name, const std::string& base_frame, const std::string& tip_frame, double search_discretization);

    /**
     * @brief Set the parameters for the solver
     * @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for; For example, rhe name of the ROS parameter that contains the robot description;
     * @param group_name The group for which this solver is being configured
     * @param base_frame The base frame in which all input poses are expected.
     * This may (or may not) be the root frame of the chain that the solver operates on
     * @param tip_frame The tip of the chain
     * @param search_discretization The discretization of the search when the solver steps through the redundancy
     */
    virtual void setValues(const std::string& robot_description, const std::string& group_name, const std::string& base_frame, const std::string& tip_frame, double search_discretization);
    /**
     * @brief  Return all the joint names in the order they are used internally
     */
    virtual const std::vector<std::string>& getJointNames() const;

    /**
     * @brief  Return all the link names in the order they are represented internally
     */
    virtual const std::vector<std::string>& getLinkNames() const;

    /** @brief Get KDL segment index */
    int getKDLSegmentIndex(const KDL::Chain &chain, const std::string &name) const;

private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;

    std::vector<double> joint_min_vector_;
    std::vector<double> joint_max_vector_;
    std::vector<bool> joint_has_limits_vector_;

    clopema_kinematics::IKSolver* ik_solver_;
    size_t num_joints_;
    std::vector<int> free_params_;

    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;
    KDL::Chain kdl_chain_;
    ros::ServiceServer srv_get_ik_all;
};

#endif
