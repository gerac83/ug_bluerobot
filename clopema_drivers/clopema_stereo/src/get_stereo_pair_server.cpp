#include <clopema_stereo/GetPairServer.h>

using namespace std;
using namespace sensor_msgs;
using namespace clopema_drivers::stereo;

/* ------------------------------------------------------------------------- */
GetPairServer::GetPairServer(string action_left, string action_right) :
    as_(nh_, ACTION_GET_PAIR, boost::bind(&GetPairServer::get_pair, this, _1), false),
    action_left(action_left), action_right(action_right) {

    as_.start();
    ROS_INFO("Capture Server started.");
}

/* ------------------------------------------------------------------------- */
bool GetPairServer::preempt_is_requested() {
    if(as_.isPreemptRequested()) {
        as_.setPreempted();
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------- */
void GetPairServer::get_pair(const clopema_drivers_msgs::GetStereoPairGoalConstPtr& goal) {

    if(preempt_is_requested())
        return;

    sensor_msgs::Image im_left, im_right;
    sensor_msgs::CameraInfo iminf_left, iminf_right;

    GetImageClient client_left(action_left, true);
    GetImageClient client_right(action_right, true);

    send_capture_goal(client_left, goal->scale);
    send_capture_goal(client_right, goal->scale);

    if(preempt_is_requested())
        return;

    if(get_capture_result(client_left, im_left, iminf_left) &&
            get_capture_result(client_right, im_right, iminf_right)) {

        result.img_left = im_left;
        result.img_right = im_right;
        result.iminfo_left = iminf_left;
        result.iminfo_right = iminf_right;
        as_.setSucceeded(result);
    } else
        as_.setAborted();
}

/* ------------------------------------------------------------------------- */
void GetPairServer::send_capture_goal(GetImageClient& client, float scale) {

    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    clopema_drivers_msgs::GetCameraImageGoal goal;
    goal.scale = scale;

    client.sendGoal(goal);
}

/* ------------------------------------------------------------------------- */
bool GetPairServer::get_capture_result(GetPairServer::GetImageClient& client, Image& img, CameraInfo& iminf) {

    clopema_drivers_msgs::GetCameraImageResultConstPtr get_img_result;

    //wait for the action to return
    bool finished_before_timeout = client.waitForResult(ros::Duration(60.0));

    actionlib::SimpleClientGoalState state = client.getState();

    if(finished_before_timeout) {
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            get_img_result = client.getResult();
            img = get_img_result->img;
            iminf = get_img_result->iminfo;
            return true;
        }
    }

    return false;
}


/* ------------------------------------------------------------------------- */
int main(int argc, char** argv) {

    string action_left = ACTION_CAPTURE_LEFT;
    string action_right = ACTION_CAPTURE_RIGHT;

    if(argc == 3 && argv[1][0] == '\\' && argv[2][0] == '\\') {
        action_left = argv[1];
        action_right = argv[2];
    }
    ros::init(argc, argv, "get_stereo_pair_server");

    ROS_INFO("Starting action!");
    GetPairServer cs_(action_left, action_right);
    ros::spin();

    return 0;
}
