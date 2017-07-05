#include <ros/ros.h>
#include <clopema_drivers_msgs/GetStereoPairAction.h>
#include <actionlib/client/simple_action_client.h>
#include <clopema_drivers_msgs/clopema_drivers.h>

using namespace std;
using namespace sensor_msgs;
using namespace clopema_drivers::stereo;

int main(int argc, char **argv) {
    clopema_drivers_msgs::GetStereoPairResultConstPtr result;

    ros::init(argc, argv, "capturing_stereo_client");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    spinner.stop();

    /// Create actionlib client and goal
    actionlib::SimpleActionClient<clopema_drivers_msgs::GetStereoPairAction>
    client(ACTION_GET_PAIR, true);
    

    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    clopema_drivers_msgs::GetStereoPairGoal goal;
    goal.scale = 1;

    ros::Time start_time;
    start_time = ros::Time::now();

    client.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = client.waitForResult(ros::Duration(60.0));

    actionlib::SimpleClientGoalState state = client.getState();


    if(finished_before_timeout) {
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Getting stereo pair stereo successed.");
        else
            ROS_ERROR_STREAM("Getting stereo pair aborted: " + state.getText());
    } else
        ROS_INFO("Action did not finish before the time out.");

    ros::Duration elapsed = ros::Time::now() - start_time;
    ROS_INFO("Elapsed time: %f", elapsed.toSec());

    return 0;
}

