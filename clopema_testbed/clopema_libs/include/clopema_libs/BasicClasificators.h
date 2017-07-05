#ifndef BASICCLASIFICATORS_H
#define BASICCLASIFICATORS_H
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

class BasicClasificators
{
public:
    /** \brief Classification based on the garment golor (foreground extracted based on depth)
     *  \param rgb,depth ~ xtion3 values
     *  \param max_depth maximum depth in meters
     *  \return class where 0 is dark and 1 is light */
    static int ClassificationLuminance (const cv::Mat& rgb, const cv::Mat& depth,
                                        double max_depth = 1.0, double light_thresh = 80.0, bool verbose = false);

    /** \brief Measuring of weight of garmend held by gripper
    *  \param f_empty force measured with empty gripper
    *  \param f_hold force measured with tha garment held by gripper
    *  \return measured weight in kg */
    static float  MeasureWeight (const Eigen::Vector3d& f_empty, const Eigen::Vector3d& f_hold);
};

#endif // BASICCLASIFICATORS_H
