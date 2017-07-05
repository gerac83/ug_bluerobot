#include <clopema_libs/BasicClasificators.h>
#include <ros/ros.h>
#include <fstream>
#include <clopema_libs/ForceGrabber.h>

using namespace Eigen;
using namespace cv;

/* ------------------------------------------------------------------------- */
int BasicClasificators::ClassificationLuminance (const cv::Mat& rgb, const cv::Mat& depth,
        double max_depth, double light_thresh, bool verbose)
{

    Mat mask;
    inRange (depth, 1, (int) max_depth * 1000, mask);

    Mat garment = Mat::zeros (rgb.size(), rgb.type());
    rgb.copyTo (garment, mask);

    Mat garmentGray;
    cv::cvtColor (garment, garmentGray, CV_BGR2GRAY);

    if (verbose) {
        imwrite ("/tmp/gsmask.png", mask);
        imwrite ("/tmp/gssegmented.png", garment);
        imwrite ("/tmp/gsgray.png", garmentGray);
    }

    Scalar cv_mean =  mean (garmentGray, mask);
    double mn = cv_mean[0];

    if (verbose)
        ROS_INFO_STREAM ("Mean: " << mn);

    if (verbose) {
        std::ofstream ofs;
        ofs.open ("/tmp/gsclass.csv", std::ofstream::out | std::ofstream::app);
        ofs << mn << std::endl;
        ofs.close();
    }
    int cls = 0;
    if (mn > light_thresh)
        cls = 0;
    else
        cls = 1;
    return cls;
}

/* ------------------------------------------------------------------------- */
float BasicClasificators::MeasureWeight (const Vector3d& f_empty, const Vector3d& f_hold)
{
    float gravity = 9.8;
    
//     ROS_INFO_STREAM("Empty: " << f_empty);
//     ROS_INFO_STREAM("Hold:  " << f_hold);
    
    float from_z = std::abs(f_hold[2] - f_empty[2]) / gravity;
    float from_xyz = ((f_hold - f_empty) / gravity).norm();
    
//     ROS_INFO_STREAM("From z:   " << from_z);
//     ROS_INFO_STREAM("From xyz: " << from_xyz);
    
    return from_xyz;
}

