/**
 * Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
 *
 * Authors:     Libor Wagner <wagnelib@cmp.felk.cvut.cz>
 *              Vladimir Petrik <petrivl3@felk.cvut.cz>
 * Institute:   Czech Technical University in Prague
 * Created on:  Oct 17, 2013
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <clopema_robot/robot_commander.h>
#include <clopema_moveit/GetCartesianPathDual.h>
#include <clopema_moveit/ClopemaGraspFromTable.h>
#include <clopema_moveit/ClopemaGraspFromTableDual.h>
#include <clopema_moveit/GFold.h>
#include <clopema_robot/GraspAndFold.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPositionIK.h>
#include <boost/foreach.hpp>
#include <clopema_moveit/TrajCollision.h>
#include <clopema_robot/TimeParametrization.h>
#include <industrial_msgs/SetDrivePower.h>
#include <clopema_robot/SetSpeed.h>

namespace clopema_robot {
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;

ClopemaRobotCommander::ClopemaRobotCommander (const std::string &group, float planning_time) : MoveGroup(group) {
    overwrite_time_parameterization = true;
    std::string req_planner;
    if (!ros::param::get(PLANNER_PARAM, req_planner)) {
        req_planner = "RRTConnectkConfigDefault";
    }
    if (req_planner == "RRTstarmoddedkConfigDefault") {
        set_number_of_diff_planners(2); //To use RRTConnectkConfigDefault if not successfull
        set_number_of_plan_attemps(5);
    } else {
        set_number_of_diff_planners(1);
        set_number_of_plan_attemps(1);
    }

    ROS_INFO_STREAM("Used planner: " << req_planner);
    setPlannerId(req_planner);
    
    this->planning_time = planning_time;
    setPlanningTime(planning_time);
    setGoalJointTolerance(0.00001);
}

void ClopemaRobotCommander::setServoPowerOff(bool force) {
    industrial_msgs::SetDrivePower srv;
    srv.request.drive_power = false;
    ros::service::call("/clopema_controller/set_drive_power", srv);
}

void ClopemaRobotCommander::setRobotSpeed(float speed) {
//     ROS_WARN_STREAM("SetRobotSpeed function is deprected with the new driver");
    clopema_robot::SetSpeed srv;
    srv.request.speed = speed;
    if(!ros::service::call("/move_group/set_robot_speed", srv)) {
        ROS_ERROR_STREAM("Cannot call service set robot speed");
    }
    ROS_INFO_STREAM("Speed: " << srv.response.current_speed);
}

float ClopemaRobotCommander::getRobotSpeed() {
//     ROS_WARN_STREAM("GetRobotSpeed function is deprected with the new driver");
    clopema_robot::SetSpeed srv;
    srv.request.speed = -1;
    if(!ros::service::call("/move_group/set_robot_speed", srv)) {
        ROS_ERROR_STREAM("Cannot call service set robot speed");
    }
    return srv.response.current_speed;
}

bool ClopemaRobotCommander::getRobotIO(unsigned int address) {
    ROS_WARN_STREAM("This function is deprected with the new driver");
}

void ClopemaRobotCommander::setRobotIO(unsigned int address, bool value) {
    ROS_WARN_STREAM("This function is deprected with the new driver");
}

void ClopemaRobotCommander::setGripperState(const std::string &gripper_action,
        int state, bool wait) {
    if (!wait) {
        ROS_WARN_STREAM("Set gripper state have to wait for the result");
    }
    trajectory_msgs::JointTrajectory traj;
    if (gripper_action == ACTION_R1_GRIPPER) {
        traj.joint_names.push_back("r1_joint_grip");
    } else if (gripper_action == ACTION_R2_GRIPPER) {
        traj.joint_names.push_back("r2_joint_grip");
    } else {
        return;
    }
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(0);
    traj.points[0].accelerations.resize(1);
    traj.points[0].velocities.resize(1);

    if (state == GRIPPER_OPEN)
        traj.points[0].positions.push_back(1.0);
    else
        traj.points[0].positions.push_back(0.0);

    ClopemaRobotCommander::Plan plan;
    plan.trajectory_.joint_trajectory = traj;
    execute(plan);
}

double ClopemaRobotCommander::computeCartesianPathDual(
        const std::vector<geometry_msgs::Pose> &waypoints1,
        const std::string& link1,
        const std::vector<geometry_msgs::Pose> &waypoints2,
        const std::string& link2, double step, double jump_threshold,
        moveit_msgs::RobotTrajectory &msg, bool avoid_collisions, const std::string& group) {
    clopema_moveit::GetCartesianPathDual::Request req;
    clopema_moveit::GetCartesianPathDual::Response res;

    if (getStartState())
        robot_state::robotStateToRobotStateMsg(*getStartState(),
                req.start_state);
    if(group.empty())
        req.group_name = this->getName();
    else
        req.group_name = group;
    req.header.frame_id = getPoseReferenceFrame();
    req.header.stamp = ros::Time::now();
    req.waypoints_1 = waypoints1;
    req.link_name_1 = link1;
    req.link_name_2 = link2;
    req.waypoints_2 = waypoints2;
    req.max_step = step;
    req.jump_threshold = jump_threshold;
    req.avoid_collisions = avoid_collisions;

    if (ros::service::call(SRV_CARTESIAN_PATH_SERVICE_DUAL_NAME, req, res)) {
        if (res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            msg = res.solution;
            return res.fraction;
        } else
            return -1.0;
    }

    return -1.0;
}

bool ClopemaRobotCommander::grasp_from_table_plan(
        const std::vector<geometry_msgs::Pose>& poses, const std::string& tip,
        const std::string& table_desk,
        std::vector<trajectory_msgs::JointTrajectory>& trajectories,
        const std::vector<geometry_msgs::Pose>& final_poses,
        const std::string& base_frame, double offset_minus, double offset_plus,
        double table_minus, double table_plus, double grasping_angle) {

    clopema_moveit::ClopemaGraspFromTable srv;
    srv.request.frame_id = base_frame;
    srv.request.ik_link = tip;
    srv.request.offset_minus = offset_minus;
    srv.request.offset_plus = offset_plus;
    srv.request.offset_table_minus = table_minus;
    srv.request.offset_table_plus = table_plus;
    srv.request.grasping_angle = grasping_angle;
    srv.request.table_desk = table_desk;
    srv.request.poses = poses;
    srv.request.final_poses = final_poses;

    if (getStartState())
        robot_state::robotStateToRobotStateMsg(*getStartState(),
                srv.request.start_state);

    if (!ros::service::call(SRV_GRASP_FROM_TABLE, srv)) {
        ROS_ERROR_STREAM("Cannot call grasp_from_table service");
        return false;
    }
    if (!srv.response.error.empty()) {
        return false;
    }
    trajectories = srv.response.joint_trajectories;
    return true;
}

bool ClopemaRobotCommander::grasp_from_table_plan(
        const geometry_msgs::Pose& pose, const std::string& tip,
        const std::string& table_desk,
        std::vector<trajectory_msgs::JointTrajectory>& trajectories,
        const geometry_msgs::Pose& final_pose, const std::string& base_frame,
        double offset_minus, double offset_plus, double table_minus,
        double table_plus, double grasping_angle) {

    std::vector<geometry_msgs::Pose> poses;
    std::vector<geometry_msgs::Pose> final_poses;
    poses.push_back(pose);
    if (!(final_pose.orientation.x == 0 && final_pose.orientation.y == 0
            && final_pose.orientation.z == 0 && final_pose.orientation.w == 0))
        final_poses.push_back(final_pose);

    return grasp_from_table_plan(poses, tip, table_desk, trajectories,
            final_poses, base_frame, offset_minus, offset_plus, table_minus,
            table_plus, grasping_angle);
}

bool ClopemaRobotCommander::grasp_from_table_dual_plan(
        const std::vector<geometry_msgs::Pose>& poses_1, const std::string& tip_1,
        const std::vector<geometry_msgs::Pose>& poses_2, const std::string& tip_2,
        const std::string& table_desk,
        std::vector<trajectory_msgs::JointTrajectory>& trajectories,
        const std::vector<geometry_msgs::Pose>& final_poses_1, const std::vector<geometry_msgs::Pose>& final_poses_2,
        const std::string& base_frame, double offset_minus, double offset_plus, double table_minus, double table_plus, double grasping_angle) {
            
        
    clopema_moveit::ClopemaGraspFromTableDual srv;
    srv.request.frame_id = base_frame;
    srv.request.ik_link_1 = tip_1;
    srv.request.ik_link_2 = tip_2;
    srv.request.offset_minus = offset_minus;
    srv.request.offset_plus = offset_plus;
    srv.request.offset_table_minus = table_minus;
    srv.request.offset_table_plus = table_plus;
    srv.request.grasping_angle = grasping_angle;
    srv.request.table_desk = table_desk;
    srv.request.poses_1 = poses_1;
    srv.request.final_poses_1 = final_poses_1;
    srv.request.poses_2 = poses_2;
    srv.request.final_poses_2 = final_poses_2;

    if (getStartState())
        robot_state::robotStateToRobotStateMsg(*getStartState(),
                srv.request.start_state);

    if (!ros::service::call(SRV_GRASP_FROM_TABLE_DUAL, srv)) {
        ROS_ERROR_STREAM("Cannot call grasp_from_table_dual service");
        return false;
    }
    if (!srv.response.error.empty()) {
        return false;
    }
    trajectories = srv.response.joint_trajectories;
    return true;
}

bool ClopemaRobotCommander::grasp_from_table_dual_plan(
        const geometry_msgs::Pose& pose_1, const std::string& tip_1,
        const geometry_msgs::Pose& pose_2, const std::string& tip_2,
        const std::string& table_desk, std::vector<trajectory_msgs::JointTrajectory>& trajectories,
        const geometry_msgs::Pose& final_pose_1, const geometry_msgs::Pose& final_pose_2, const std::string& base_frame,
        double offset_minus, double offset_plus, double table_minus, double table_plus, double grasping_angle) {
            
    std::vector<geometry_msgs::Pose> poses_1, poses_2;
    std::vector<geometry_msgs::Pose> final_poses_1, final_poses_2;
    poses_1.push_back(pose_1);
    poses_2.push_back(pose_2);
    if (!(final_pose_1.orientation.x == 0 && final_pose_1.orientation.y == 0
            && final_pose_1.orientation.z == 0 && final_pose_1.orientation.w == 0))
        final_poses_1.push_back(final_pose_1);
    if (!(final_pose_2.orientation.x == 0 && final_pose_2.orientation.y == 0
            && final_pose_2.orientation.z == 0 && final_pose_2.orientation.w == 0))
        final_poses_2.push_back(final_pose_2);

    return grasp_from_table_dual_plan(poses_1, tip_1, poses_2, tip_2, table_desk, trajectories,
            final_poses_1, final_poses_2, base_frame, offset_minus, offset_plus, table_minus,
            table_plus, grasping_angle);
        }

bool ClopemaRobotCommander::transform_to_tip(const geometry_msgs::Pose& pose_in,
        const std::string& ee_link, geometry_msgs::Pose& pose_out,
        std::string& tip_name) {
    std::string ik_link_name = ee_link;
    if (strncmp(ik_link_name.c_str(), "r2", strlen("r2")) == 0) {
        tip_name = "r2_tip_link";
    } else if (strncmp(ik_link_name.c_str(), "r1", strlen("r1")) == 0) {
        tip_name = "r1_tip_link";
    } else if (strncmp(ik_link_name.c_str(), "xtion1", strlen("xtion1")) == 0) {
        tip_name = "xtion1_link_ee";
    } else if (strncmp(ik_link_name.c_str(), "xtion2", strlen("xtion2")) == 0) {
        tip_name = "xtion2_link_ee";
    }

    if (ee_link == tip_name) {
        pose_out = pose_in;
        return true;
    }

    Eigen::Affine3d gripper =
            getCurrentState()->getFrameTransform(tip_name).inverse()
                    * getCurrentState()->getFrameTransform(ee_link);

    Eigen::Affine3d e;
    tf::poseMsgToEigen(pose_in, e);
    e = e * gripper.inverse();
    tf::poseEigenToMsg(e, pose_out);

    return true;
}

bool ClopemaRobotCommander::setJointValueTarget(
        const geometry_msgs::Pose& eef_pose,
        const std::string& end_effector_link) {
    geometry_msgs::Pose pose;
    std::string ik_link;
    transform_to_tip(eef_pose, end_effector_link, pose, ik_link);
    return MoveGroup::setJointValueTarget(pose, ik_link);
}

bool ClopemaRobotCommander::setJointValueTarget(
        const geometry_msgs::PoseStamped& eef_pose,
        const std::string& end_effector_link) {
    geometry_msgs::PoseStamped pose = eef_pose;
    std::string ik_link;
    transform_to_tip(eef_pose.pose, end_effector_link, pose.pose, ik_link);
    return MoveGroup::setJointValueTarget(pose, ik_link);
}

bool ClopemaRobotCommander::setJointValueTarget(const Eigen::Affine3d& eef_pose,
        const std::string& end_effector_link) {
    geometry_msgs::Pose msg;
    tf::poseEigenToMsg(eef_pose, msg);
    return setJointValueTarget(msg, end_effector_link);
}

void ClopemaRobotCommander::setPathConstraints(
        const moveit_msgs::Constraints& constraint, bool add_empty_joint) {
    if (!add_empty_joint) {
        MoveGroup::setPathConstraints(constraint);
        return;
    }
    moveit_msgs::Constraints c = constraint;
    fillEmptyJointConstraints(c);
    MoveGroup::setPathConstraints(c);
}

void ClopemaRobotCommander::fillEmptyJointConstraints(
        moveit_msgs::Constraints& constraint) {
    moveit_msgs::JointConstraint empty_joint_constraints;
    empty_joint_constraints.joint_name = "r1_joint_grip";
    empty_joint_constraints.position = 0;
    empty_joint_constraints.tolerance_above = 1.5;
    empty_joint_constraints.tolerance_below = 1.5;
    empty_joint_constraints.weight = 1;
    constraint.joint_constraints.push_back(empty_joint_constraints);
}

bool ClopemaRobotCommander::computeGFold(
        const geometry_msgs::Point& final_pose_1, const std::string& ik_link_1,
        const geometry_msgs::Point& final_pose_2, const std::string& ik_link_2,
        trajectory_msgs::JointTrajectory& traj, const std::string& base_frame,
        const std::vector<std::string>& enable_collision_1, const std::vector<std::string>& enable_collision_2) {

    clopema_moveit::GFold srv;
    srv.request.final_point_1 = final_pose_1;
    srv.request.final_point_2 = final_pose_2;
    srv.request.frame_id = base_frame;
    srv.request.ik_link_1 = ik_link_1;
    srv.request.ik_link_2 = ik_link_2;
    if (getStartState())
        robot_state::robotStateToRobotStateMsg(*getStartState(),
                srv.request.start_state);
    srv.request.allow_collision_1 = enable_collision_1;
    srv.request.allow_collision_2 = enable_collision_2;
        
    if (!ros::service::call(SRV_GFOLD, srv)) {
        ROS_ERROR_STREAM("Cannot call gfold service");
        return false;
    }
    if (srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        traj = srv.response.joint_trajectory;
    else {
        ROS_INFO_STREAM("Cannot interpolate gfold");
        return false;
    }
    return true;
}

bool ClopemaRobotCommander::compute_ik(
        const std::vector<std::string>& ik_link_names,
        const std::vector<geometry_msgs::PoseStamped>& poses,
        moveit::core::RobotStatePtr& rs, const std::string& group) {

    moveit_msgs::GetPositionIK srv;
    srv.request.ik_request.avoid_collisions = true;
    if (group.empty())
        srv.request.ik_request.group_name = getName();
    else
        srv.request.ik_request.group_name = group;
    srv.request.ik_request.ik_link_names = ik_link_names;
    srv.request.ik_request.pose_stamped_vector = poses;
    if (!rs) {
        if (getStartState())
            rs.reset(new robot_state::RobotState(*getStartState()));
        else
            rs.reset(new robot_state::RobotState(*getCurrentState()));
    }

    robot_state::robotStateToRobotStateMsg(*rs,
            srv.request.ik_request.robot_state);
    srv.request.ik_request.timeout = ros::Duration(1.0);

    if (!ros::service::call(SRV_GET_IK, srv)) {
        ROS_ERROR_STREAM("Cannot call IKT service");
        return false;
    }

    if (srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        robot_state::robotStateMsgToRobotState(srv.response.solution, *rs);
        return true;
    } else {
        ROS_INFO_STREAM("IK return: " << srv.response.error_code.val);
        return false;
    }
}

bool ClopemaRobotCommander::compute_ik(
        const std::vector<std::string>& ik_link_names,
        const std::vector<geometry_msgs::Pose>& poses,
        moveit::core::RobotStatePtr& rs, const std::string& group) {
    std::vector<geometry_msgs::PoseStamped> pss(poses.size());
    for (unsigned int i = 0; i < pss.size(); ++i) {
        pss[i].header.frame_id = getPoseReferenceFrame();
        pss[i].pose = poses[i];
    }
    return compute_ik(ik_link_names, pss, rs, group);
}

bool ClopemaRobotCommander::compute_ik(const std::string& ik_link_name,
        const geometry_msgs::PoseStamped& pose, moveit::core::RobotStatePtr& rs,
        const std::string& group) {
    std::vector<std::string> ik_link_names;
    std::vector<geometry_msgs::PoseStamped> poses;
    ik_link_names.push_back(ik_link_name);
    poses.push_back(pose);
    return compute_ik(ik_link_names, poses, rs, group);
}

bool ClopemaRobotCommander::compute_ik(const std::string& ik_link_name,
        const geometry_msgs::Pose& pose, moveit::core::RobotStatePtr& rs,
        const std::string& group) {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = getPoseReferenceFrame();
    ps.pose = pose;
    return compute_ik(ik_link_name, ps, rs, group);
}

void ClopemaRobotCommander::setStartState(
        const moveit_msgs::RobotState& start_state) {
    robot_state::RobotState rs(*getCurrentState());
    robot_state::robotStateMsgToRobotState(start_state, rs);
    setStartState(rs);
}

void ClopemaRobotCommander::setStartState(
        const robot_state::RobotState& start_state) {
    this->start_state.reset(new robot_state::RobotState(start_state));
    MoveGroup::setStartState(start_state);
}

void ClopemaRobotCommander::setStartStateToCurrentState() {
    start_state.reset();
    MoveGroup::setStartStateToCurrentState();
}

robot_state::RobotStatePtr ClopemaRobotCommander::getStartState() {
    return start_state;
}

void ClopemaRobotCommander::set_number_of_plan_attemps(unsigned int n) {
    if (n > 0)
        number_of_plan_attemps = n;
}

void ClopemaRobotCommander::set_number_of_diff_planners(unsigned int n) {
    if (n > 0)
        number_of_planners = n;
}

void ClopemaRobotCommander::setPlannerId(const std::string& planner_id) {
    MoveGroup::setPlannerId(planner_id);
    this->planner_id = planner_id;
}

bool ClopemaRobotCommander::plan(Plan &plan, bool verbose) {
    bool state = false;
    std::string init_planner_id = this->planner_id;

    std::vector<std::string> planners;
    planners.push_back(this->planner_id);
    std::vector<std::string> pt = get_planner_types();
    pt.erase(std::remove(pt.begin(), pt.end(), planners[0]), pt.end());
    for (unsigned int i = 0; i < number_of_planners - 1; ++i) {
        if (pt.size() == i)
            break;
        planners.push_back(pt[i]);
    }

    BOOST_FOREACH(const std::string& plan_type, planners) {
        if(verbose) {
            ROS_INFO_STREAM("Using planner: " << plan_type);
        }

        bool no_path_const = true;
        no_path_const &= getPathConstraints().joint_constraints.empty();
        no_path_const &= getPathConstraints().position_constraints.empty();
        no_path_const &= getPathConstraints().orientation_constraints.empty();
        no_path_const &= getPathConstraints().visibility_constraints.empty();

        if(!no_path_const) {
            ROS_INFO_STREAM("Path constraints specified, using RRT connect planner");
            setPlannerId("RRTConnectkConfigDefault");
        } else {
            setPlannerId(plan_type);
        }

        for (unsigned int i = 0; i < number_of_plan_attemps; ++i) {
            if(verbose) {
                ROS_INFO_STREAM("Planner attemp:" << i);
            }
            state = MoveGroup::plan(plan);
            if (state)
                break;
        }
        if(state)
            break;
    }
    setPlannerId(init_planner_id);
    return state;
}

bool ClopemaRobotCommander::move() {
    ClopemaRobotCommander::Plan p;
    if(!plan(p)) {
        return false;
    }
    return execute(p);
}


std::vector<std::string> ClopemaRobotCommander::get_planner_types() {
    std::vector<std::string> planners;
    planners.push_back("RRTstarmoddedkConfigDefault");
    planners.push_back("RRTConnectkConfigDefault");
    return planners;
}

trajectory_msgs::JointTrajectory ClopemaRobotCommander::gripper_trajectory(int open, const std::string& group) {
    std::string grp = group;
    if(grp.empty())
        grp = this->getName();

    trajectory_msgs::JointTrajectory traj;
    int num_joints = 1;
    if (grp == "r1_arm") {
        traj.joint_names.push_back("r1_joint_grip");
    } else if (grp == "r2_arm") {
        traj.joint_names.push_back("r2_joint_grip");
    } else if (grp == "arms") {
        traj.joint_names.push_back("r1_joint_grip");
        traj.joint_names.push_back("r2_joint_grip");
        num_joints = 2;
    }
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(0);
    traj.points[0].accelerations.resize(num_joints);
    traj.points[0].velocities.resize(num_joints);

    if (open == clopema_robot::GRIPPER_OPEN)
        traj.points[0].positions.resize(num_joints, 1.0);
    else
        traj.points[0].positions.resize(num_joints, 0.0);

    return traj;
}

bool ClopemaRobotCommander::grasp_and_fold(const std::vector< geometry_msgs::Pose >& poses_1, const std::vector< geometry_msgs::Pose >& poses_2,
                                           const geometry_msgs::Point& final_point_1, const geometry_msgs::Point& final_point_2, const std::string& table_desk,
                                           std::vector< trajectory_msgs::JointTrajectory >& trajectories, const std::string& base_frame, double offset_minus,
                                           double offset_plus, double table_minus, double table_plus, double grasping_angle) {
    clopema_robot::GraspAndFold srv;
    srv.request.frame_id = base_frame;
    srv.request.offset_minus = offset_minus;
    srv.request.offset_plus = offset_plus;
    srv.request.offset_table_minus = table_minus;
    srv.request.offset_table_plus = table_plus;
    srv.request.grasping_angle = grasping_angle;
    srv.request.table_desk = table_desk;
    srv.request.poses_1 = poses_1;
    srv.request.poses_2 = poses_2;
    srv.request.final_point_1 = final_point_1;
    srv.request.final_point_2 = final_point_2;
    if (getStartState())
        robot_state::robotStateToRobotStateMsg(*getStartState(),
                srv.request.start_state);
    
    if (!ros::service::call(SRV_GRASP_AND_FOLD, srv)) {
        ROS_ERROR_STREAM("Cannot call grasp_and_fold service");
        return false;
    }
    
    trajectories = srv.response.joint_trajectories;
    if(trajectories.empty()) {
        return false;
    }
    return true;
}

bool ClopemaRobotCommander::grasp_and_fold(const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2,
                                           const geometry_msgs::Point& final_point_1, const geometry_msgs::Point& final_point_2, const std::string& table_desk,
                                           std::vector< trajectory_msgs::JointTrajectory >& trajectories, const std::string& base_frame, double offset_minus,
                                           double offset_plus, double table_minus, double table_plus, double grasping_angle) {
    std::vector<geometry_msgs::Pose> poses_1, poses_2;
    poses_1.push_back(pose_1);
    poses_2.push_back(pose_2);
    return grasp_and_fold(poses_1, poses_2, final_point_1, final_point_2, table_desk, trajectories, base_frame,
                          offset_minus, offset_plus, table_minus, table_plus, grasping_angle);
}

bool ClopemaRobotCommander::check_trajectory(const moveit_msgs::RobotTrajectory& rtraj, 
                                                const std::vector< std::string >& enable_collision_1, const std::vector< std::string >& enable_collision_2) {

    clopema_moveit::TrajCollision srv;
    srv.request.enable_collision_1 = enable_collision_1;
    srv.request.enable_collision_2 = enable_collision_2;
    srv.request.rtraj = rtraj;
    if (getStartState())
        robot_state::robotStateToRobotStateMsg(*getStartState(), srv.request.start_state);
    
    if (!ros::service::call(SRV_CHECK_TRAJECTORY, srv)) {
        ROS_ERROR_STREAM("Cannot call trajectory_check service");
        return false;
    }
    
    return srv.response.valid;
}


bool ClopemaRobotCommander::plan_put_on_table(trajectory_msgs::JointTrajectory& traj, const std::string& table_desk, double table_offset_start, double table_offset_end,  double dist_x_start, double dist_x_stop
    ,const Eigen::Quaterniond& rot_left) {
    robot_state::RobotState rs(*getCurrentState());
    if (getStartState())
        rs = robot_state::RobotState(*getStartState());
    Eigen::Affine3d ee1, ee2, to_r750, table;
    to_r750 = rs.getGlobalLinkTransform("r750").inverse();
    ee1 = to_r750 * rs.getGlobalLinkTransform("r1_ee");
    ee2 = to_r750 * rs.getGlobalLinkTransform("r2_ee");
    table = to_r750 * rs.getGlobalLinkTransform(table_desk);
    const std::string original_pose_reference = getPoseReferenceFrame();
    setPoseReferenceFrame("r750");

    double l = (ee1.inverse() * ee2).translation().norm();
    ee1.translation().x() = dist_x_start;
    ee1.translation().y() = -l / 2;
    //ee1.translation().z() = 1.0;
    ee2.translation().x() = dist_x_start;
    ee2.translation().y() = l / 2;
    //ee2.translation().z() = 1.0;

    std::vector<geometry_msgs::Pose> wp1, wp2;
    wp1.resize(3);
    wp2.resize(3);
    tf::poseEigenToMsg(ee1, wp1[0]);
    tf::poseEigenToMsg(ee2, wp2[0]);
    
    ee1.translation().z() = (table.translation().z() + table_offset_start);
    ee2.translation().z() = (table.translation().z() + table_offset_start);
    tf::poseEigenToMsg(ee1, wp1[1]);
    tf::poseEigenToMsg(ee2, wp2[1]);

    ee1.translation().x() = dist_x_stop;
    ee2.translation().x() = dist_x_stop;
    ee1.translation().z() = (table.translation().z() + table_offset_end);
    ee2.translation().z() = (table.translation().z() + table_offset_end);
    tf::poseEigenToMsg(rot_left * ee1, wp1[2]);
    tf::poseEigenToMsg(rot_left * ee2, wp2[2]);
    tf::pointEigenToMsg(ee1.translation(), wp1[2].position);
    tf::pointEigenToMsg(ee2.translation(), wp2[2].position);
    
    moveit_msgs::RobotTrajectory rtraj;
    double d = computeCartesianPathDual(wp1, "r1_ee", wp2, "r2_ee", 0.01, 1.5, rtraj, true);
    if(fabs(d - 1.0) > 0.005) {
        ROS_ERROR_STREAM("Cannot put on table, interpolated fraction: " << d);
        return false;
    }
    traj = rtraj.joint_trajectory;
    setPoseReferenceFrame(original_pose_reference);
    return true;
}

bool ClopemaRobotCommander::execute_traj(const trajectory_msgs::JointTrajectory& traj) {
    clopema_robot::ClopemaRobotCommander::Plan plan;
    plan.trajectory_.joint_trajectory = traj;
    return execute(plan);
}

bool ClopemaRobotCommander::execute_traj(const std::vector< trajectory_msgs::JointTrajectory >& trajs) {
    BOOST_FOREACH(const trajectory_msgs::JointTrajectory & t, trajs) {
        if(!execute_traj(t))
            return false;
    }
    return true;
}

bool ClopemaRobotCommander::smooth_garment_on_line(const geometry_msgs::Point& sa, const geometry_msgs::Point& sb, const std::string& table, 
                                                   std::vector< trajectory_msgs::JointTrajectory >& all_trajs, int number_of_touches, double offset_z, double above_table_down, double above_table_up) {
    using namespace Eigen;
    std::string old_reference = getPoseReferenceFrame();
    robot_state::RobotState start_state(*getCurrentState());
    if(getStartState())
        start_state = *getStartState();
    Vector3d pa, pb;
    tf::pointMsgToEigen(sa, pa);
    tf::pointMsgToEigen(sb, pb);
   
    robot_state::RobotState rs(*getCurrentState());  //Transform pa ,pb to the table reference frame
    if(!rs.knowsFrameTransform(table)) {
        ROS_ERROR_STREAM("Table frame not known");
        return false;
    }
    Affine3d base = Affine3d::Identity();
    if(rs.knowsFrameTransform(getPoseReferenceFrame())) {
        base = rs.getFrameTransform(getPoseReferenceFrame());
    }
    pa = rs.getFrameTransform(table).inverse() * base * pa;
    pb = rs.getFrameTransform(table).inverse() * base * pb;
    setPoseReferenceFrame(table);
    
    Vector3d v = pb - pa;
    v.normalize();
    double start_angle = acos(v.dot(Vector3d::UnitX()));
    std::vector<geometry_msgs::Quaternion> quaternions;
    for(double angle = M_PI_2; angle < 3 * M_PI_2; angle += M_PI / 8.0) {
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, angle + start_angle);
        quaternions.push_back(q);
        q = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, -angle + start_angle);
        quaternions.push_back(q);
    }
    
    { //TODO: fix that
        Vector3d v = pa - pb;
        v.normalize();
        double start_angle = acos(v.dot(Vector3d::UnitX()));
        for(double angle = M_PI_2; angle < 3 * M_PI_2; angle += M_PI / 8.0) {
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, angle + start_angle);
            quaternions.push_back(q);
            q = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, -angle + start_angle);
            quaternions.push_back(q);
        }
        
    }

    bool success = false;
    BOOST_FOREACH(const geometry_msgs::Quaternion & q, quaternions) {
        if(success) {
            break;
        }
        all_trajs.clear();
        setStartState(start_state);
        for(int i = 0; i < number_of_touches; ++i) {
            robot_state::RobotState state(*getStartState());
            BOOST_FOREACH(trajectory_msgs::JointTrajectory & t, all_trajs) {
                state.setVariablePositions(t.joint_names, t.points.back().positions);
            }
            setStartState(state);
            double k = i / ((double)(number_of_touches - 1));
            Vector3d point = pa + k * (pb - pa);
            geometry_msgs::Pose p;
            tf::pointEigenToMsg(point, p.position);
            p.orientation = q;

            std::vector<trajectory_msgs::JointTrajectory> traj;
            if(grasp_from_table_plan(p, "r2_ee", table, traj, geometry_msgs::Pose(), table, -offset_z, offset_z + 0.0001, -above_table_down, above_table_up, M_PI / 8.0)) {
                ROS_INFO_STREAM("Touch number successfull: " << i);
                success = true;
                traj.erase(traj.begin() + 3);
                traj.erase(traj.begin() + 1);
                all_trajs.insert(all_trajs.end(), traj.begin(), traj.end());
            } else {
                success = false;
                all_trajs.clear();
                break;
            }
        }
    }

    setStartState(start_state);
    setPoseReferenceFrame(old_reference);
    return success;
}

double ClopemaRobotCommander::computeCartesianPath(const std::vector< geometry_msgs::Pose >& waypoints, double eef_step, double jump_threshold, 
                                                   moveit_msgs::RobotTrajectory& trajectory, bool avoid_collisions) {
    std::string ee = this->getEndEffectorLink();
    if(ee.empty()) {
        ROS_WARN_STREAM("End effector is not specified for this group");
        return -1;
    }
    return computeCartesianPath(waypoints, ee, eef_step, jump_threshold, trajectory, avoid_collisions);
}

double ClopemaRobotCommander::computeCartesianPath(const std::vector< geometry_msgs::Pose >& waypoints, const std::string& ik_link, double eef_step, double jump_threshold,
                                                   moveit_msgs::RobotTrajectory& trajectory, bool avoid_collisions) {
    return computeCartesianPathDual(waypoints, ik_link, waypoints, ik_link, eef_step, jump_threshold, trajectory, avoid_collisions, "arms");
}

bool ClopemaRobotCommander::move_to_named_target(const std::string& named_target) {
    if(!setNamedTarget(named_target)) {
        ROS_WARN_STREAM("Cannot set named target: " << named_target);
        return false;
    }
    ClopemaRobotCommander::Plan plan;
    if(!this->plan(plan)) {
        ROS_WARN_STREAM("Cannot plan to named target: " << named_target);
        return false;
    }
    if(!execute(plan)) {
        ROS_WARN_STREAM("Cannot execute trajectory to named target: " << named_target);
        return false;
    }
    return true;
}

moveit::planning_interface::MoveItErrorCode ClopemaRobotCommander::asyncExecute(const moveit::planning_interface::MoveGroup::Plan& plan) {
    moveit::planning_interface::MoveGroup::Plan p = plan;
    if(overwrite_time_parameterization) {
        if(!add_time_parametrization(p))
            return false;
    }
    return MoveGroup::asyncExecute(p);
}

moveit::planning_interface::MoveItErrorCode ClopemaRobotCommander::execute(const moveit::planning_interface::MoveGroup::Plan& plan) {
    moveit::planning_interface::MoveGroup::Plan p = plan;
    if(overwrite_time_parameterization) {
        if(!add_time_parametrization(p)) {
            return false;   
        }
    }
    return MoveGroup::execute(p);
}

bool ClopemaRobotCommander::add_time_parametrization(moveit::planning_interface::MoveGroup::Plan& plan) {
    clopema_robot::TimeParametrization srv;
    srv.request.group = this->getName();
    if(start_state) {
        robot_state::robotStateToRobotStateMsg(*start_state, srv.request.start_state);
    } else {
        robot_state::robotStateToRobotStateMsg(*getCurrentState(), srv.request.start_state);
    }
    srv.request.trajectory = plan.trajectory_;
    
    if(!ros::service::call("/move_group/add_time_parametrization", srv)) {
        ROS_ERROR_STREAM("Cannot call service for time parametrization");
        return false;
    }
    
    if(!srv.response.success) {
        ROS_WARN_STREAM("Cannot parametrize trajectory");
        return false;
    }
    plan.trajectory_ = srv.response.parametrized_trajectory;
    return true;
}

void ClopemaRobotCommander::set_start_state_from_traj(const trajectory_msgs::JointTrajectory& msg) {
    if(msg.points.empty()) {
        ROS_WARN_STREAM("Not valid trajectory to set start state from");
        return;
    }
    robot_state::RobotState rs(*getCurrentState());
    rs.setVariablePositions(msg.joint_names, msg.points.back().positions);
    setStartState(rs);
}

}
