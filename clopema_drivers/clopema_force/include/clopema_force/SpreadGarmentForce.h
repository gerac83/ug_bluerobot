/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2014-02-25
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Stop the motion if force reach some level
 */


#ifndef SPREADGARMENTFORCE_H
#define SPREADGARMENTFORCE_H

#include <ros/ros.h>
#include <clopema_robot/robot_commander.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <industrial_msgs/RobotStatus.h>


/** \brief Spread garment until the force is applied to the second sensor */
class SpreadGarmentForce {
    public:
        SpreadGarmentForce(const clopema_robot::ClopemaRobotCommander::Ptr& gcrc): node("~") {
            if(gcrc)
                crc = gcrc;
            else
                crc.reset(new clopema_robot::ClopemaRobotCommander("arms"));
            sub_force = node.subscribe("/r2_force_data_filtered", 1, &SpreadGarmentForce::force_cb, this);
            sub_robot_state = node.subscribe("/clopema_controller/robot_status" , 1, &SpreadGarmentForce::robot_state_cb, this);
            robot_speed = 0.02;
            threshold = 1.0;
            is_robot_moving = false;
        }

        ~SpreadGarmentForce() {
            sub_force.shutdown();
            sub_robot_state.shutdown();
            {
                boost::unique_lock<boost::mutex> l(mutex_wrench); 
            }
        }

        /** \brief Start robot movement to spread the garment, will lowerize the speed and then set back */
        bool start_movement(double max_distance = 0.5) {
            double cur_speed = crc->getRobotSpeed();
            crc->setRobotSpeed(robot_speed);
            crc->setStartStateToCurrentState();
            std::vector<geometry_msgs::Pose> wp1(1), wp2(1);
            robot_state::RobotState rs(*crc->getCurrentState());
            Eigen::Affine3d edir = rs.getGlobalLinkTransform("r1_ee").inverse() * rs.getGlobalLinkTransform("r2_ee");
            double sign_dir = edir.translation()[1] > 0.0 ? 1 : -1;
            tf::poseEigenToMsg(rs.getGlobalLinkTransform("r1_ee") * Eigen::Translation3d(0.0, - sign_dir * max_distance, 0.0), wp1[0]);
            
            edir = rs.getGlobalLinkTransform("r2_ee").inverse() * rs.getGlobalLinkTransform("r1_ee");
            sign_dir = edir.translation()[1] > 0.0 ? 1 : -1;
            tf::poseEigenToMsg(rs.getGlobalLinkTransform("r2_ee") * Eigen::Translation3d(0.0, - sign_dir * max_distance, 0.0), wp2[0]);
            
            clopema_robot::ClopemaRobotCommander::Plan plan;
            double d = crc->computeCartesianPathDual(wp1, "r1_ee", wp2, "r2_ee", 0.0005, 1.2, plan.trajectory_);
            if(fabs(d - 1.0) > 0.95) {//start anyway go maximum as can...
                ROS_ERROR_STREAM("Cannot interpolate fraction: " << d);
                return false;
            }
            crc->setRobotSpeed(cur_speed);
            return crc->asyncExecute(plan);
        }

        /** \brief Force callback */
        void force_cb(const geometry_msgs::WrenchStamped& msg) {
            using namespace Eigen;
            Vector3d forces;
            tf::vectorMsgToEigen(msg.wrench.force, forces);
            set_offsets_if_not_set(forces);
            double mag = (forces - *force_offset).norm();
            set_force_mag(mag);
        }
        
        /** \brief Robot State callback for robot movement detection */
        void robot_state_cb(const industrial_msgs::RobotStatus::ConstPtr& rstate)
        {
            is_robot_moving = (rstate->in_motion.val == 1);
        }

        /** \brief Get offsets thread safe */
        Eigen::Vector3d get_offsets() {
            boost::unique_lock<boost::mutex> l(mutex_wrench);
            Eigen::Vector3d ret;
            if(force_offset)
                ret = *force_offset;
            return ret;
        }

        /** \brief If offsets was not set before set it to the forces */
        void set_offsets_if_not_set(const Eigen::Vector3d& forces) {
            boost::unique_lock<boost::mutex> l(mutex_wrench);
            if(!force_offset)
                force_offset.reset(new Eigen::Vector3d(forces));
        }

        /** \brief Reset offsets based on the current sensor values */
        void reset_offsets() {
            boost::unique_lock<boost::mutex> l(mutex_wrench);
            force_offset.reset();
        }

        /** \brief Thread safe set force */
        void set_force_mag(double mag) {
            boost::unique_lock<boost::mutex> l(mutex_wrench);
            force_mag = mag;
        }

        /** \brief Thread safe get force */
        double get_force_mag() {
            boost::unique_lock<boost::mutex> l(mutex_wrench);
            return force_mag;
        }
        
        /** \brief Thread safe get moving flag */
        bool get_moving() {
            using namespace industrial_msgs;
            RobotStatusConstPtr rs = ros::topic::waitForMessage<RobotStatus>("/clopema_controller/robot_status", ros::Duration(0.2));
            if (!rs) {
                ROS_WARN("Robot status not received.");
                return false;
            }
            
            if (rs->in_motion.val != 1) {
                return false;
            }
            
            return true;
        }


        /** \brief Decide whether should stop based on the force feedback */
        bool should_stop_force(double threshold) {
            if(get_force_mag() > threshold) {
                return true;
            }
            return false;
        }

        /** \brief Wait to compete movement and check the force sensor repeatly
         *  \return false if movement was interrupted by force, true otherwise */
        bool wait_to_complete_movement(const ros::Duration& period = ros::Duration(0.05)) {
            ROS_INFO_STREAM("Waiting for max force.");

            while(true) {
                if(should_stop_force(threshold)) {
                    crc->stop();
                    return false;
                }
                period.sleep();
            }
            crc->stop();
            return true;
        }
        

    private:
        ros::NodeHandle node;
        boost::shared_ptr<Eigen::Vector3d> force_offset;
        double force_mag;
        boost::mutex mutex_wrench;
        ros::Subscriber sub_force;
        ros::Subscriber sub_robot_state;
        bool is_robot_moving;
    public:
        clopema_robot::ClopemaRobotCommander::Ptr crc;
        double threshold, robot_speed;
};

#endif // SPREADGARMENTFORCE_H

