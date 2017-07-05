#ifndef GETIMAGESERVER_H
#define GETIMAGESERVER_H

#include <ros/ros.h>
#include <clopema_drivers_msgs/clopema_drivers.h>
#include <actionlib/server/simple_action_server.h>
#include <clopema_drivers_msgs/GetCameraImageAction.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <clopema_stereo/GPhotoRos.h>

class GetImageServer {
public:

    typedef actionlib::SimpleActionServer<clopema_drivers_msgs::GetCameraImageAction>::Result CaptureResult;
    
    GetImageServer(std::string action, std::string device, std::string frame_id, std::string calib_file);

    void capture(const clopema_drivers_msgs::GetCameraImageGoalConstPtr& goal);
    bool preempt_is_requested();

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<clopema_drivers_msgs::GetCameraImageAction> as_;
    clopema_drivers_msgs::GetCameraImageResult result;

    std::string device;
    std::string frame_id;
    std::string calib_file;

    image_transport::ImageTransport it_;
    ros::Subscriber sub_;
    image_transport::Publisher image_pub_;
    boost::mutex photo_mutex_;
    ros::Publisher camera_info_pub_;

    DSLR_cam camera;
    sensor_msgs::CameraInfo info;
    cv_bridge::CvImage cvi;

};

#endif // GETIMAGESERVER_H
