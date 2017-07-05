#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <moveit/robot_state/robot_state.h>
#include <image_geometry/pinhole_camera_model.h>

namespace clopema_libs {

    class Segmentation {
        public:
            Segmentation();
            ~Segmentation();

            /** \brief Create RegionOfInterest for the specified table in the state 'rs'. */
            static std::vector<cv::Point2i> create_table_roi(robot_state::RobotState& rs, const std::string& table, const std::string& sensor_frame,
                    const image_geometry::PinholeCameraModel& cm, double offset = 0.0);
            
            /** \brief Convert vector of cv::Point2i into the vector of geometry_msgs::Point */
            static std::vector<geometry_msgs::Point> cv_to_geometry_points(const std::vector<cv::Point2i>& vec);
            
            /** \brief Convert vector of cv::Point2i into the vector of geometry_msgs::Point */
            static std::vector<cv::Point2i> geometry_to_cv_points(const std::vector<geometry_msgs::Point>& vec);
            
    };

}
#endif // SEGMENTATION_H

