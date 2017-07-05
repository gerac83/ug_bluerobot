#ifndef CLOPEMADRIVERSNAMES_H
#define CLOPEMADRIVERSNAMES_H

#include <string>
using namespace std;

namespace clopema_drivers {
    namespace stereo {
        static const string ACTION_CAPTURE_LEFT =   "/clopema_drivers/stereo/GetCameraImageLeft";
        static const string ACTION_CAPTURE_RIGHT =  "/clopema_drivers/stereo/GetCameraImageRight";
        static const string ACTION_GET_PAIR =       "/cloepma_drivers/stereo/GetStereoPair";
        static const string DEV_PARAM_LEFT =        "/left_camera_device";
        static const string DEV_PARAM_RIGHT =       "/right_camera_device";
        static const string CAMERA_NAME_L =         "left_camera";
        static const string CAMERA_NAME_R =         "right_camera";
        static const string MODE_FULL =             "full";
        static const string MODE_PREVIEW =          "preview";
    }
}

#endif // CLOPEMADRIVERSNAMES_H
// 