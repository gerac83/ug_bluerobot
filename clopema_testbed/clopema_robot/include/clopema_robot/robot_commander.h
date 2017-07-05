/**
 * Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
 *
 * Authors:     Libor Wagner <wagnelib@cmp.felk.cvut.cz>
 *              Vladimir Petrik <petrivl3@felk.cvut.cz>
 * Institute:   Czech Technical University in Prague
 * Created on:  Oct 17, 2013
 */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>

#ifndef ROBOT_COMMANDER_H
#define ROBOT_COMMANDER_H

namespace clopema_robot
{
    const std::string SRV_SET_ROBOT_SPEED = "/set_robot_speed";
    const std::string SRV_GET_ROBOT_SPEED = "/get_robot_speed";
    const std::string SRV_SET_SERVO_POWER_OFF = "/joint_trajectory_action/set_power_off";
    const std::string SRV_GET_ROBOT_IO = "/read_io";
    const std::string SRV_SET_ROBOT_IO = "/write_io";
    const std::string SRV_CARTESIAN_PATH_SERVICE_DUAL_NAME = "/compute_cartesian_path_dual";
    const std::string SRV_GRASP_FROM_TABLE = "/clopema_planner/grasp_from_table";
    const std::string SRV_GRASP_FROM_TABLE_DUAL = "/clopema_planner/grasp_from_table_dual";
    const std::string SRV_GFOLD = "/compute_gfold";
    const std::string SRV_GRASP_AND_FOLD = "/clopema_planner/grasp_and_fold";
    const std::string SRV_GET_IK = "/compute_ik";
    const std::string SRV_CHECK_TRAJECTORY = "/check_trajectory";

    const std::string ACTION_R1_GRIPPER = "/r1_gripper/command";
    const std::string ACTION_R2_GRIPPER = "/r2_gripper/command";
    
    const std::string PLANNER_PARAM = "/clopema_planner";

    const int GRIPPER_OPEN = 1;
    const int GRIPPER_CLOSE = 0;
    
    const std::string NAMED_TARGET_EXT_MINUS_90 = "ext_minus_90";
    const std::string NAMED_TARGET_EXT_PLUS_90 = "ext_plus_90";
    const std::string NAMED_TARGET_EXT_HOME = "home_ext";
    const std::string NAMED_TARGET_ARMS_XTION1_ABOVE_TABLE = "xtion1_above_table";
    const std::string NAMED_TARGET_ARMS_HOME = "home_arms";
    const std::string NAMED_TARGET_ARMS_XTION1_ABOVE_T3 = "xtion1_above_t3";    //without external axis moving

    class ClopemaRobotCommander : public moveit::planning_interface::MoveGroup
    {

        public:
            typedef boost::shared_ptr<ClopemaRobotCommander> Ptr;
            typedef boost::shared_ptr<ClopemaRobotCommander const> ConstPtr;
            /**
             * Initialize robot commander for particular group.
             */
            ClopemaRobotCommander (const std::string &group, float planning_time = 0.5);

            /**
             * Shut off servo power.
             */
            void setServoPowerOff(bool force=false);


            /**
             * Set robot speed, possible value can be from 0 to 1. This value is
             * then converted to robot speed units (i.e.  speed*10000).
             *
             * Note that there is a speed limit defined by the I001 variable on
             * the teach pendant. This limit is in robot speed units and will
             * overide the speed set by this function.
             */
            void setRobotSpeed(float speed);

            /**
             * Get robot speed.
             */
            float getRobotSpeed();

            /**
             * Read robot IO. The address is controller address of the IO
             * register.
             */
            bool getRobotIO(unsigned int address);

            /**
             * Write robot IO.
             */
            void setRobotIO(unsigned int address, bool value);

            /**
             * Set gripper state (open or close).
             */
            void setGripperState(const std::string &gripper_action, int state, bool wait=true);

            /** \brief Compute cartesian path for the both arms. */
            double computeCartesianPathDual(const std::vector<geometry_msgs::Pose> &waypoints1, const std::string& link1,
                    const std::vector<geometry_msgs::Pose> &waypoints2, const std::string& link2,
                    double step, double jump_threshold,
                    moveit_msgs::RobotTrajectory &msg, bool avoid_collisions = true, const std::string& group = "");

            /** \brief Compute trajectories for grasping garment from the table. */
            bool grasp_from_table_plan(
                    const std::vector<geometry_msgs::Pose>& poses, const std::string& tip,
                    const std::string& table_desk,
                    std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                    const std::vector<geometry_msgs::Pose>& final_poses,
                    const std::string& base_frame = "base_link",
                    double offset_minus = 0.02, double offset_plus = 0.02,
                    double table_minus = 0, double table_plus = 0.1, double grasping_angle = M_PI / 6);

            /** \brief Compute trajectories for grasping garment from the table. */
            bool grasp_from_table_plan(
                    const geometry_msgs::Pose& pose, const std::string& tip,
                    const std::string& table_desk,
                    std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                    const geometry_msgs::Pose& final_pose = geometry_msgs::Pose(),
                    const std::string& base_frame = "base_link",
                    double offset_minus = 0.02, double offset_plus = 0.02,
                    double table_minus = 0, double table_plus = 0.1, double grasping_angle = M_PI / 6);
            
            /** \brief Compute trajectories for grasping garment from the table. */
            bool grasp_from_table_dual_plan(
                    const std::vector<geometry_msgs::Pose>& poses_1, const std::string& tip_1,
                    const std::vector<geometry_msgs::Pose>& poses_2, const std::string& tip_2,
                    const std::string& table_desk,
                    std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                    const std::vector<geometry_msgs::Pose>& final_poses_1,
                    const std::vector<geometry_msgs::Pose>& final_poses_2,
                    const std::string& base_frame = "base_link",
                    double offset_minus = 0.02, double offset_plus = 0.02,
                    double table_minus = 0, double table_plus = 0.1, double grasping_angle = M_PI / 6);

            /** \brief Compute trajectories for grasping garment from the table. */
            bool grasp_from_table_dual_plan(
                    const geometry_msgs::Pose& pose_1, const std::string& tip_1,
                    const geometry_msgs::Pose& pose_2, const std::string& tip_2,
                    const std::string& table_desk,
                    std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                    const geometry_msgs::Pose& final_pose_1 = geometry_msgs::Pose(),
                    const geometry_msgs::Pose& final_pose_2 = geometry_msgs::Pose(),
                    const std::string& base_frame = "base_link",
                    double offset_minus = 0.02, double offset_plus = 0.02,
                    double table_minus = 0, double table_plus = 0.1, double grasping_angle = M_PI / 6);
            
            /** \brief Grasp from table and fold using gfold */
            bool grasp_and_fold(
                const std::vector<geometry_msgs::Pose>& poses_1, const std::vector<geometry_msgs::Pose>& poses_2,
                const geometry_msgs::Point& final_point_1, const geometry_msgs::Point& final_point_2,
                const std::string& table_desk, std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                const std::string& base_frame = "base_link", double offset_minus = 0.02, double offset_plus = 0.02,
                double table_minus = 0, double table_plus = 0.1, double grasping_angle = M_PI / 6);

            /** \brief Grasp from table and fold using gfold */
            bool grasp_and_fold(
                const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2,
                const geometry_msgs::Point& final_point_1, const geometry_msgs::Point& final_point_2,
                const std::string& table_desk, std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                const std::string& base_frame = "base_link", double offset_minus = 0.02, double offset_plus = 0.02,
                double table_minus = 0, double table_plus = 0.1, double grasping_angle = M_PI / 6);
            
            /** \brief Transform pose from ee_link to tip_link */
            bool transform_to_tip(const geometry_msgs::Pose& pose_in, const std::string& ee_link,
                    geometry_msgs::Pose& pose_out, std::string& tip_name);

            /** \brief Set the joint state goal for a particular joint by computing IK. This is different from setPoseTarget() in that
              a single IK state is computed to be the goal of the planner, rather than sending the pose itself to the planner. */
            bool setJointValueTarget(const geometry_msgs::Pose &eef_pose, const std::string &end_effector_link = "");

            /** \brief Set the joint state goal for a particular joint by computing IK. This is different from setPoseTarget() in that
              a single IK state is computed to be the goal of the planner, rather than sending the pose itself to the planner. */
            bool setJointValueTarget(const geometry_msgs::PoseStamped &eef_pose, const std::string &end_effector_link = "");

            /** \brief Set the joint state goal for a particular joint by computing IK. This is different from setPoseTarget() in that
              a single IK state is computed to be the goal of the planner, rather than sending the pose itself to the planner. */
            bool setJointValueTarget(const Eigen::Affine3d &eef_pose, const std::string &end_effector_link = "");

            /** \brief Given a vector of real values in the same order as expected by the group, set those as the joint state goal */
            bool setJointValueTarget(const std::vector<double> &group_variable_values) {
                return MoveGroup::setJointValueTarget(group_variable_values);
            }

            /** \brief Given a map of joint names to real values, set those as the joint state goal */
            bool setJointValueTarget(const std::map<std::string, double> &variable_values) {
                return MoveGroup::setJointValueTarget(variable_values);
            }

            /** \brief Set the joint state goal from corresponding joint values from the specified state.
              Values from state for joints not in this MoveGroup's group are ignored. */
            bool setJointValueTarget(const robot_state::RobotState &robot_state) {
                return MoveGroup::setJointValueTarget(robot_state);
            }

            /** \brief Set the joint state goal for a particular joint */
            bool setJointValueTarget(const std::string &joint_name, const std::vector<double> &values) {
                return MoveGroup::setJointValueTarget(joint_name, values);
            }

            /** \brief Set the joint state goal for a particular joint */
            bool setJointValueTarget(const std::string &joint_name, double value) {
                return MoveGroup::setJointValueTarget(joint_name, value);
            }

            /** \brief Set the joint state goal for a particular joint */
            bool setJointValueTarget(const sensor_msgs::JointState &state) {
                return MoveGroup::setJointValueTarget(state);
            }

            bool setPathConstraints(const std::string &constraint) {
                return MoveGroup::setPathConstraints(constraint);
            }

            /** \brief Shadow MoveGroup constraints because we add empty joint constraint to force planning in
             *         joint space instead of cartesian. */
            void setPathConstraints(const moveit_msgs::Constraints &constraint, bool add_empty_joint = true);

            /** \brief Fill empty joint constraints so the planner will be forced to use joint constraint */
            void fillEmptyJointConstraints(moveit_msgs::Constraints &constraint);

            /** Compute one geometry based fold. */
            bool computeGFold(const geometry_msgs::Point& final_pose_1, const std::string& ik_link_1,
                    const geometry_msgs::Point& final_pose_2, const std::string& ik_link_2,
                    trajectory_msgs::JointTrajectory& traj, const std::string& base_frame = "base_link",
                    const std::vector<std::string>& enable_collision_1 = std::vector<std::string>(),
                    const std::vector<std::string>& enable_collision_2 = std::vector<std::string>());

            /** Compute IKT on the 'rs' state. If not specified current state is used.
             * Name of the group is used unless you specify other using 'group' parameter.*/
            bool compute_ik(const std::vector<std::string>& ik_link_names,
                    const std::vector<geometry_msgs::PoseStamped>& poses,
                    moveit::core::RobotStatePtr& rs, const std::string& group = "");

            /** Compute IKT on the 'rs' state. If not specified current state is used.
             * Name of the group is used unless you specify other using 'group' parameter.
             * Poses are in the reference frame */
            bool compute_ik(const std::vector<std::string>& ik_link_names,
                    const std::vector<geometry_msgs::Pose>& poses,
                    moveit::core::RobotStatePtr& rs, const std::string& group = "");

            /** Compute IKT on the 'rs' state. If not specified current state is used.
             * Name of the group is used unless you specify other using 'group' parameter.*/
            bool compute_ik(const std::string& ik_link_name,
                    const geometry_msgs::PoseStamped& pose,
                    moveit::core::RobotStatePtr& rs, const std::string& group = "");

            /** Compute IKT on the 'rs' state. If not specified current state is used.
             * Name of the group is used unless you specify other using 'group' parameter.*/
            bool compute_ik(const std::string& ik_link_name,
                    const geometry_msgs::Pose& pose,
                    moveit::core::RobotStatePtr& rs, const std::string& group = "");

            /** \brief If a different start state should be considered instead of the current state of the robot, this function sets that state */
            void setStartState(const moveit_msgs::RobotState &start_state);

            /** \brief If a different start state should be considered instead of the current state of the robot, this function sets that state */
            void setStartState(const robot_state::RobotState &start_state);

            /** \brief Set the starting state for planning to be that reported by the robot's joint state publication */
            void setStartStateToCurrentState();

            /** Return start_state if it was set before, null otherwise */
            robot_state::RobotStatePtr getStartState();

            /** \brief Set number of planning attemps */
            void set_number_of_plan_attemps(unsigned int n);

            /** \brief Set number of different planners which will be used.
             *  \details First actual planner is used then different n is tried.
             *          First succesful plann is returned. */
            void set_number_of_diff_planners(unsigned int n);

            /** \brief Plan N times with M different planners, where N,M can be set by set_number_of* functions*/
            bool plan(Plan &plan, bool verbose = false);
            
            /** \brief Call plan and then execute */
            bool move();

            /** \brief Set planner with will be used for futher planning */
            void setPlannerId(const std::string& planner_id);

            /** \brief Get all planners config */
            std::vector<std::string> get_planner_types();
            
            /** \brief Get trajectory to change state of the gripper or grippers 
                \details Default group is used if not specified, possible groups are: {r1_arm, r2_arm, arms} */
            trajectory_msgs::JointTrajectory gripper_trajectory(int open, const std::string& group = "");
            
            /** \brief Check trajectory for collision */
            bool check_trajectory(const moveit_msgs::RobotTrajectory& rtraj,
                    const std::vector<std::string>& enable_collision_1 = std::vector<std::string>(),
                    const std::vector<std::string>& enable_collision_2 = std::vector<std::string>());
            
            /** \brief Plan trajectories for putting garment on the table which is located in front of the robot
             *  \details r750 is reference frame so the putting the garment on the table is independent on the rotation of ext_axis 
             *           The current distance and rotation of the gripper is supposed to by constant durring the whole movement */
            bool plan_put_on_table(trajectory_msgs::JointTrajectory& traj, const std::string& table_desk = "t2_desk", double table_offset_start = 0.15, double table_offset_end = 0.05,  double dist_x_start = 0.40, double dist_x_stop = 1.25,
                const Eigen::Quaterniond& rot_left = Eigen::Quaterniond::Identity());
            
            /** \brief Wrapper to move robot without constructing Plan */
            bool execute_traj(const trajectory_msgs::JointTrajectory& traj);
            /** \brief Wrapper to move robot without constructing Plan */
            bool execute_traj(const std::vector<trajectory_msgs::JointTrajectory>& trajs);
            
            /** \brief Smooth garment on the table specified by frame 'table' by touching with first gripper on the line (sa, sb).
            *  \param number_of_touches Number of position on the line (sa, sb)
            *  \param offset_z The point is moved by offset_z to not touch with ee directly
            *  \param above_table_down Touching position above the table
            *  \param above_table_up Planning position above the table */
            bool smooth_garment_on_line(const geometry_msgs::Point& sa, const geometry_msgs::Point& sb, const std::string& table, std::vector<trajectory_msgs::JointTrajectory>& all_trajs,
                            int number_of_touches = 3, double offset_z = 0.0, double above_table_down = 0.0, double above_table_up = 0.1);
            
            /** \brief Compute cartesian path wrapper to use our implementation instead of MoveIt */
            double computeCartesianPath(const std::vector <geometry_msgs::Pose >& waypoints, double eef_step, double jump_threshold, moveit_msgs::RobotTrajectory& trajectory,
                                        bool avoid_collisions = true);
            /** \brief Compute cartesian path wrapper to use our implementation instead of MoveIt */
            double computeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints, const std::string& ik_link, double eef_step, double jump_threshold, 
                                        moveit_msgs::RobotTrajectory& trajectory, bool avoid_collisions = true);
            
            /** \brief Wrapper to set named target and move to it - pring warning on problem */
            bool move_to_named_target(const std::string& named_target);
            
            /** \brief Given a \e plan, execute it without waiting for completion. Return true on success.
             *  \details Compute new time parametrization if necessary (i.e. is not correctly set) */
            moveit::planning_interface::MoveItErrorCode asyncExecute(const Plan &plan);

            /** \brief Given a \e plan, execute it while waiting for completion. Return true on success. 
             *  \details Compute new time parametrization if necessary (i.e. is not correctly set) */
            moveit::planning_interface::MoveItErrorCode execute(const Plan &plan);
            
            /** \brief Use iterative time parametrization utility to recompute plan timestamps */
            bool add_time_parametrization(Plan& plan);
            
            /** \brief Set start state from the last point of the trajectory */
            void set_start_state_from_traj(const trajectory_msgs::JointTrajectory& msg);
            
        public:
            /** \brief Recompute time parametrization for each incoming trajectory - true by default */
            bool overwrite_time_parameterization;
        private:
            unsigned int number_of_plan_attemps;
            unsigned int number_of_planners;
            std::string planner_id;
            robot_state::RobotStatePtr start_state;
            float planning_time;
    };
}
#endif // ROBOT_COMMANDER_H
