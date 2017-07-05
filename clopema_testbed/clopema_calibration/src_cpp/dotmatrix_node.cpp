#include "dotmatrix.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";

DMError DMLoadChainFromFile(DMDetector detector, const char * path);
void FreePgmImage(DMImageRef * ref);

class Dotmatrix
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    DMDetector det_;


    public:
    Dotmatrix(const char * dmalg) : it_(nh_)
    {
        // Prepare dotmatrix
        det_ = DMCreateDetector();
        if (DMLoadChainFromFile(det_, dmalg) != DMERROR_OK) {
            ROS_ERROR("Failed to read algorithm chain from file %s", dmalg);
        }

        image_pub_ = it_.advertise("out", 1);
        image_sub_ = it_.subscribe("in", 1, &Dotmatrix::imageCb, this);


        cv::namedWindow(WINDOW);
    }

    ~Dotmatrix()
    {
        cv::destroyWindow(WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // Convert image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Convert Image to GrayScale
        cv::Mat gray;
        if (cv_ptr->image.channels() > 1) { 
            cv::cvtColor(cv_ptr->image, gray, CV_RGB2GRAY);
        } else {
            gray = cv_ptr->image;
            cv::cvtColor(gray, cv_ptr->image, CV_GRAY2RGB);
        }

        // Convert image to DMImage
        DMImageRef ref;

        cv::Size size = gray.size();
        unsigned char * ptr = new unsigned char[size.width*size.height];

        ref.data   = (void *) ptr;
        ref.width  = size.width;
        ref.height = size.height;
        ref.memW   = gray.cols;

        for (int i = 0; i < size.width * size.height; i++) {
            *(ptr++) = (unsigned char)(gray.data[i]);
        }

        if (DMProcessImage(det_,&ref) != DMERROR_OK) {
            //    ROS_ERROR("Failed to process image");
        }

        // collect found dots
        const DMDot * dots;
        int           num;

        DMGetDetectedDots(det_,&num,&dots);

        for (int i = 0; i < num; i++) {
            cv::circle(cv_ptr->image, cv::Point(dots[i].imgCol, dots[i].imgRow), 2, CV_RGB(255,0,0));
        }

        // free image
        FreePgmImage(&ref);

        cv::imshow(WINDOW, cv_ptr->image);
        cv::waitKey(3);

        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

void FreePgmImage(DMImageRef * ref)
{
    delete [] (unsigned char *)ref->data;
}

DMError DMLoadChainFromFile(DMDetector detector, const char * path)
{
    char line[80];
    char * command;
    int  s;

    // Open file
    FILE * f = fopen(path,"rt");

    if (!f) {
        printf("Error opening file: %s\n", path);
    }

    DMError ret = DMERROR_OK;
    DMError err = DMERROR_OK;

    while(fgets(line, 80, f) != NULL)
    {
        switch (line[0]) {
            case '\0':
            case '\n':
            case '#':
                break;
            case 'I':
                s = strlen(line);
                command = (char*) malloc(s - 2);
                strncpy(command, line+2, s - 3);
                command[s-3] = '\0';
                err = DMAddImageAlgorithm(detector, command);
                ROS_INFO("DMAddImageAlgorithm(det, \"%s\") = %d", command, err);
                break;
            case 'S':
                s = strlen(line);
                command = (char*) malloc(s - 2);
                strncpy(command, line+2, s - 3);
                command[s-3] = '\0';
                err = DMAddShapeAlgorithm(detector, command);
                ROS_INFO("DMAddShapeAlgorithm(det, \"%s\") = %d", command, err);
                break;
            case 'D':
                s = strlen(line);
                command = (char*) malloc(s - 2);
                strncpy(command, line+2, s - 3);
                command[s-3] = '\0';
                err = DMAddDetectionAlgorithm(detector, command);
                ROS_INFO("DMAddDetectionAlgorithm(det, \"%s\") = %d", command, err);
                break;
            default:
                printf("Unrecognised command: '%s'\n", line);
                err = DMERROR_INVALID_SYNTAX;
                break;
        }

        if (err != DMERROR_OK) {
            printf("Error: %d\n", err);
            ret = err;
        }
    }

    fclose(f);

    return ret;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dotmatrix");
    Dotmatrix ic = Dotmatrix(argv[1]);
    ros::spin();
    return 0;
}
