/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 05/19/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Learn Segmentation model from the selected images
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <segmentation/SegmentationGMMGrabCut.h>

namespace fs = boost::filesystem;

void load_images_from_dir(const std::string& folder, std::vector<cv::Mat>& imgs) {
    using namespace cv;
    fs::path targetDir(folder);
    if(fs::is_regular_file(targetDir)) {
        fs::path p = targetDir;
        Mat tmp = imread(p.c_str());
        if(tmp.empty()) {
            ROS_WARN_STREAM("Cannot read: " << p.c_str());
            return;
        }
        Mat tmp_reshaped = tmp.reshape(0, tmp.rows * tmp.cols);
        imgs.push_back(tmp_reshaped);
    } else {
        fs::directory_iterator it(targetDir), eod;
        BOOST_FOREACH(fs::path const & p, std::make_pair(it, eod)) {
            if(fs::is_regular_file(p)) {
                Mat tmp = imread(p.c_str());
                if(tmp.empty()) {
                    ROS_WARN_STREAM("Cannot read: " << p.c_str());
                    continue;
                }
                Mat tmp_reshaped = tmp.reshape(0, tmp.rows * tmp.cols);
                imgs.push_back(tmp_reshaped);
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "segmentation_learn");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);

    std::string templates;
    int num_gauss;
    node.param("num_gauss", num_gauss, 2);
    node.param<std::string>("templates", templates, "");
    if(templates.empty()) {
        ROS_ERROR_STREAM("Parameter 'templates' has to be specified for the learning (directory with the images or the image itself)");
        ROS_INFO_STREAM("rosrun segmentation segmentation_learn _templates:=[path]");
        return -1;
    }

    using namespace cv;
    std::vector<Mat> imgs;
    load_images_from_dir(templates, imgs);

    Mat img_temp;
    vconcat(imgs, img_temp);
    if(img_temp.empty()) {
        ROS_ERROR_STREAM("Template is empty");
        return -1;
    }

    SegmentationGMMGrabCut seg(num_gauss);
    seg.learn(img_temp);
//     seg.estimate_pbg_sure_percent = 90;
//     seg.estimate_pbg_prob_percent = 10;
    seg.estimate_pbg(img_temp);

    std::string calibration = ros::package::getPath("clopema_description");
    std::string partner = getenv("CLOPEMA_PARTNER");
    std::string cname;
    if(!node.getParam("calibration_name", cname)) {
        cname = "seg_gc_gmm";
    }
    calibration += "/calibration_" + partner + "/" + cname + ".bin";
    if(!seg.save(calibration)) {
        ROS_ERROR_STREAM("Cannot save calibration to the file: " << calibration);
        return -1;
    } else {
        ROS_INFO_STREAM("Calibration saved to: " << calibration);
    }

    return 0;
}
