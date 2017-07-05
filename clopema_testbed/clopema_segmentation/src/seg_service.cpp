
/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 05/16/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Example of using segmentation class
 */

#include <ros/ros.h>
#include <segmentation/SegmentationGMMGrabCut.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <clopema_segmentation/Segmentation.h>
#include <clopema_libs/Segmentation.h>
#include <sensor_msgs/image_encodings.h>

// #define DEBUG 1

SegmentationGMMGrabCut seg;

void test(const std::string& filename) {
    cv::Mat img = cv::imread(filename);
    cv::Mat out_mask;
    seg.segment(img, out_mask);

    cv::Mat out_img;
    img.copyTo(out_img);
    out_img.setTo(cv::Scalar::all(255));
    img.copyTo(out_img, out_mask);
    imshow("img", img);
    imshow("out_mask", out_mask * 255);
    imshow("out", out_img);
    cv::moveWindow("img", 0, 0);
    cv::moveWindow("gmm", 810, 0);
    cv::moveWindow("out_mask", 0, 550);
    cv::moveWindow("out", 810, 550);
    cvWaitKey(100);
}

bool cb_segmentation(clopema_segmentation::SegmentationRequest& req, clopema_segmentation::SegmentationResponse& res) {
    cv::Mat img;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(req.image, "bgr8");
    } catch(cv_bridge::Exception& ex) {
        ROS_ERROR("Unable to convert %s image to cv", ex.what());
        return false;
    }
    cv_ptr->image.copyTo(img);

    seg.set_strategy(req.strategy);
    ROS_INFO_STREAM("Segmenting with strategy: " << req.strategy);
    ros::Time start = ros::Time::now();
    cv::Mat out_mask;
    if(req.roi.empty()) {
        seg.segment(img, out_mask);
    } else {
        cv::Mat mask_in(img.size(), CV_8UC1);
        mask_in.setTo(cv::Scalar::all(0));
        std::vector<cv::Point2i> roi = clopema_libs::Segmentation::geometry_to_cv_points(req.roi);
        cv::fillConvexPoly(mask_in, &roi[0], roi.size(), cv::Scalar(255), 8, 0);
        seg.segment(img, out_mask, mask_in);
    }
    ROS_INFO_STREAM("Time: " << (ros::Time::now().toSec() - start.toSec()));

    cv_bridge::CvImage bridge_mask;
    bridge_mask.image = out_mask;
    bridge_mask.encoding = sensor_msgs::image_encodings::MONO8;
    bridge_mask.header.stamp = ros::Time::now();
    bridge_mask.toImageMsg(res.mask);

#ifdef DEBUG
    cv::Mat out_img;
    img.copyTo(out_img, out_mask);
    imshow("img", img);
    imshow("out_mask", out_mask * 255);
    imshow("out", out_img);
    cv::moveWindow("img", 0, 0);
    cv::moveWindow("gmm", 810, 0);
    cv::moveWindow("out_mask", 0, 550);
    cv::moveWindow("out", 810, 550);
    cvWaitKey(50);
#endif
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "segmentation_service");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);

    std::string calibration = ros::package::getPath("clopema_description");
    std::string partner = getenv("CLOPEMA_PARTNER");
    std::string cname;
    if(!node.getParam("calibration_name", cname)) {
        ROS_ERROR("No name given, using default!");
        cname = "seg_gc_gmm";
    }
    calibration += "/calibration_" + partner + "/" + cname + ".bin";

    ROS_INFO_STREAM("File: " << calibration);

    if(!seg.load(calibration)) {
        ROS_WARN_STREAM("Cannot load learned model, creating new one");
        return -1;
    }
#ifdef DEBUG
    seg.show_gmm = true;
#endif
    seg.set_strategy(SegmentationGMMGrabCut::EXTRACT_LARGEST);
    ROS_INFO_STREAM("Segmentation is ready");

//     std::string segpkg = ros::package::getPath("segmentation");
//     test(segpkg + "/data/segmentation_capture/cap1table2/rgb.png");
//     cv::waitKey();
//     test(segpkg + "/data/segmentation_capture/cap2table2/rgb.png");
//     cv::waitKey();
//     test(segpkg + "/data/segmentation_capture/cap3table2/rgb.png");
//     cv::waitKey();
//     test(segpkg + "/data/segmentation_capture/cap4table2/rgb.png");
//     cv::waitKey();

    ros::ServiceServer ser_segmentation = node.advertiseService("/" + cname, cb_segmentation);

    spinner.start();
    ros::waitForShutdown();

    return 0;
}
