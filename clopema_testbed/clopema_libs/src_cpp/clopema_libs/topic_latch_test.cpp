// Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
//
// Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
// Institute:   Czech Technical University in Prague
// Created on:  Feb 13, 2014

#include <ros/ros.h>
#include <clopema_libs/topic_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

using namespace clopema::topic_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_latch_demo");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(2);
    spinner.start();


    TopicLatch<sensor_msgs::Image> tl(n, "/xtion2/rgb/image_raw");
    sensor_msgs::Image image_msg = tl.getNext(5);

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("Image", cv_ptr->image);
    cv::waitKey(0);


    return 0;
}

