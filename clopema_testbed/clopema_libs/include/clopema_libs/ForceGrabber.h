/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 05/14/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: ForceGrabber library
 */

#ifndef FORCEGRABBER_H
#define FORCEGRABBER_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread.hpp>

namespace clopema_libs {

    using namespace Eigen;

    class ForceGrabber {
        public:
            ForceGrabber(bool wait_for_init = false);
            ~ForceGrabber();

            /** \brief Return true whether wrenches values were received */
            bool is_ready();
            /** \brief Reset filtered wrench offsets - i.e. set offset to current values */
            void reset_filtered_offsets();
            /** \brief Get filtered force minus offset */
            Vector3d get_filtered_f1();
            /** \brief Get filtered force minus offset */
            Vector3d get_filtered_f2();
            /** \brief Get filtered torque minus offset */
            Vector3d get_filtered_t1();
            /** \brief Get filtered torque minus offset */
            Vector3d get_filtered_t2();
            /** \brief Save filtered forces and torques values for both hand
              * \param warn print warning if values were not received */
            bool save_filtered(const std::string& filename, bool warn = true);
            /** \brief Save downsampled forces and torques values for both hand
             *  \param append whether create new file or append to existing
              * \param warn print warning if values were not received */
            bool save_downsampled(const std::string& filename, bool append = false, bool warn = true);
            
            /** \brief Return true whether wrenches values were received */
            bool is_ready_downsampled();
            /** \brief Get downsampled force minus offset */
            Vector3d get_downsampled_f1();
            /** \brief Get downsampled force minus offset */
            Vector3d get_downsampled_f2();
            /** \brief Get downsampled torque minus offset */
            Vector3d get_downsampled_t1();
            /** \brief Get downsampled torque minus offset */
            Vector3d get_downsampled_t2();

        private:
            void cb_fitered_1(const geometry_msgs::WrenchStampedConstPtr& msg);
            void cb_fitered_2(const geometry_msgs::WrenchStampedConstPtr& msg);
            void cb_downsampled_1(const geometry_msgs::WrenchStampedConstPtr& msg);
            void cb_downsampled_2(const geometry_msgs::WrenchStampedConstPtr& msg);

        private:
            ros::NodeHandle node;
            ros::Subscriber sub_filtered_1, sub_filtered_2, sub_downsampled_1, sub_downsampled_2;
            boost::mutex mutex_wrench, mutex_downsampled;
            boost::shared_ptr<Vector3d> filtered_f1, filtered_f2, filtered_t1, filtered_t2;
            boost::shared_ptr<Vector3d> downsampled_f1, downsampled_f2, downsampled_t1, downsampled_t2;
            boost::shared_ptr<Vector3d> filtered_offset_f1, filtered_offset_f2, filtered_offset_t1, filtered_offset_t2;
    };
}
#endif // FORCEGRABBER_H
