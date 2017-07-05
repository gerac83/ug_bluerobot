
/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 05/16/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Example of using segmentation class
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <clopema_segmentation/Segmentation.h>

void img_cb(const sensor_msgs::ImageConstPtr& msg) {
    clopema_segmentation::Segmentation srv;
    srv.request.image = *msg;
    if(!ros::service::call("/seg_gc_gmm", srv))
        return;
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(srv.response.mask, "bgr8");
    } catch(cv_bridge::Exception& ex) {
        ROS_ERROR("Unable to convert %s image to cv", ex.what());
        return;
    }

    imshow("img", cv_ptr->image);
    cvWaitKey(50);
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "segmentation_example");
    ros::NodeHandle node("~");
    ros::Subscriber sub_img = node.subscribe("image", 1, &img_cb);
    ros::spin();

    return 0;
}
