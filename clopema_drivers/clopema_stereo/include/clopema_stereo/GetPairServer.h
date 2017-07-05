#ifndef GETPAIRSERVER_H
#define GETPAIRSERVER_H

#include <clopema_drivers_msgs/clopema_drivers.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <clopema_drivers_msgs/GetCameraImageAction.h>
#include <actionlib/client/simple_action_client.h>
#include <clopema_drivers_msgs/GetStereoPairAction.h>

class GetPairServer {
public:

    typedef actionlib::SimpleActionServer<clopema_drivers_msgs::GetStereoPairAction> GetStereoServer;
    typedef actionlib::SimpleActionClient<clopema_drivers_msgs::GetCameraImageAction> GetImageClient;

    GetPairServer(std::string action_left, std::string action_right);

    void get_pair(const clopema_drivers_msgs::GetStereoPairGoalConstPtr& goal);
    void send_capture_goal(GetImageClient& client, float scale);
    bool get_capture_result(GetImageClient& client, sensor_msgs::Image& img, sensor_msgs::CameraInfo& iminf);

    bool preempt_is_requested();

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<clopema_drivers_msgs::GetStereoPairAction> as_;
    clopema_drivers_msgs::GetStereoPairResult result;

    std::string action_left;
    std::string action_right;
};

#endif