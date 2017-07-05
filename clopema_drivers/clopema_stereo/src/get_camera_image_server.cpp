#include <clopema_stereo/GetImageServer.h>
#include <clopema_stereo/Utils.h>

using namespace std;
using namespace sensor_msgs;
using namespace clopema_drivers::stereo;

/* ------------------------------------------------------------------------- */
GetImageServer::GetImageServer(string action, string device, string frame_id, string calib_file) :
    as_(nh_, action, boost::bind(&GetImageServer::capture, this, _1), false),
    it_(nh_),
    camera(NULL),
    device(device), frame_id(frame_id), calib_file(calib_file) {

    // PARSE USB PORT FROM DEVICE
    // ==========================================================================================
    string usb_param = getOtuputFromCommand("readlink " + device);
    stringReplace(usb_param, "bus/usb/", "usb:");
    stringReplace(usb_param, "/", ",");
    // ==========================================================================================

    photo_mutex_.lock();
    camera = camera_init();
    camera->port = usb_param.c_str();

    ROS_INFO("PORT: %s", camera->port);

    if(camera_open(camera) < GP_OK) {
        ROS_ERROR("stereo_cam exception: camera is not available");
        ros::shutdown();
    }
    photo_mutex_.unlock();

    ROS_INFO("Camera opened... ");


    as_.start();
    ROS_INFO("Capture Server started.");
}

/* ------------------------------------------------------------------------- */
bool GetImageServer::preempt_is_requested() {
    if(as_.isPreemptRequested()) {
        as_.setPreempted();
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------- */
void GetImageServer::capture(const clopema_drivers_msgs::GetCameraImageGoalConstPtr& goal) {

    if(preempt_is_requested())
        return;

    try {
        camera->mode = 0;

        photo_mutex_.lock();
        // Capture image
        if(camera_capture(camera) < GP_OK) {
            ROS_ERROR("stereo_cam exception: cannot capture image");
            ros::shutdown();
            return;
        }
        photo_mutex_.unlock();

        Mat bgr, rgb;   // image from camera

        bgr = cvarrToMat(camera->imgBGR);
        rgb = cvarrToMat(camera->imgRGB);
        cvtColor(bgr, rgb, CV_BGR2RGB);

        cv::resize(rgb, rgb, cv::Size(0,0), goal->scale, goal->scale);

        // convert OpenCV image to ROS message
        cvi.header.stamp = ros::Time::now();
        cvi.header.frame_id = frame_id;
        cvi.encoding = "rgb8";
        cvi.image = rgb;

        info.header.stamp = cvi.header.stamp;
        info.width = rgb.cols;
        info.height = rgb.rows;

        sensor_msgs::Image img;
        cvi.toImageMsg(img);
        result.img = img;
        result.iminfo = info;

        as_.setSucceeded(result);

    } catch(cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("stereo_cam exception: " <<  e.what());
        as_.setAborted(CaptureResult(), e.what());
        return;
    }
}

/* ------------------------------------------------------------------------- */
int main(int argc, char** argv) {

    string action;
    string device;
    string frame_id;
    string calib_file = "";

    if(argc > 4)
        calib_file = argv[4];
    if(argc > 3) {
        action = argv[1];
        device = argv[2];
        frame_id = argv[3];

        ros::init(argc, argv, frame_id + "_capture_node");

        ROS_INFO("Starting action!");
        GetImageServer cs_(action, device, frame_id, calib_file);
        ros::spin();
    } else
        ROS_ERROR_STREAM("Not enough of parameteres!");

    return 0;
}
