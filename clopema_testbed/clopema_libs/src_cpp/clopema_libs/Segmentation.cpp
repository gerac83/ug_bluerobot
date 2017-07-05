#include <clopema_libs/Segmentation.h>
#include <boost/foreach.hpp>

namespace clopema_libs {
    Segmentation::Segmentation() {

    }

    Segmentation::~Segmentation() {

    }

    std::vector< cv::Point2i > Segmentation::create_table_roi(moveit::core::RobotState& rs, const std::string& table, const std::string& sensor_frame,
            const image_geometry::PinholeCameraModel& cm, double offset) {
        using namespace Eigen;
        rs.update();

        std::vector<cv::Point2i> res;
        Affine3d base_2_sensor = rs.getFrameTransform(sensor_frame);
        Affine3d base_2_desk = rs.getFrameTransform(table + "_desk");

        for(int i = 1; i < 5; ++i) {
            std::stringstream ss;
            ss << table << "_leg_" << i;
            Affine3d base_2_leg = rs.getFrameTransform(ss.str());
            Vector3d v = (base_2_sensor.inverse() * base_2_leg).translation();
            Vector3d vdesk = (base_2_sensor.inverse() * base_2_desk).translation();
            Vector3d off_dir = vdesk - v;
            off_dir.normalize();

            v = v + offset * off_dir;
            cv::Point3d pt_cv(v(0), v(1), v(2));
            cv::Point2d uv = cm.project3dToPixel(pt_cv);
            res.push_back(cv::Point2i(uv.x, uv.y));
        }
        std::reverse(res.begin(), res.end());
        return res;
    }

    std::vector< geometry_msgs::Point > Segmentation::cv_to_geometry_points(const std::vector< cv::Point2i >& vec) {
        std::vector< geometry_msgs::Point > res;
        BOOST_FOREACH(const cv::Point2i & p, vec) {
            geometry_msgs::Point point;
            point.x = p.x;
            point.y = p.y;
            res.push_back(point);
        }
        return res;
    }

    std::vector< cv::Point2i > Segmentation::geometry_to_cv_points(const std::vector< geometry_msgs::Point >& vec) {
        std::vector< cv::Point2i > res;
        BOOST_FOREACH(const geometry_msgs::Point & p, vec) {
            cv::Point2i point;
            point.x = p.x;
            point.y = p.y;
            res.push_back(point);
        }
        return res;
    }





}
