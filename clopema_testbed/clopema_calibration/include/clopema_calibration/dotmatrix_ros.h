// Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
//
// Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
// Institute:   Czech Technical University in Prague
// Created on:  Apr 7, 2014


// ---- Includes ------------------------------------------------------------ //

#include "ros/ros.h"
#include "dotmatrix.h"
#include "sensor_msgs/Image.h"
#include "clopema_calibration/Dotmatrix.h"
#include "clopema_calibration/DotmatrixDot.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

// ---- Dotmatrix Class ----------------------------------------------------- //

class Dotmatrix
{

    public:
        Dotmatrix()
        {
            det_ = DMCreateDetector();
        }

        ~Dotmatrix()
        {
            DMDestroyDetector(det_);
        }

        int reset()
        {
            DMDestroyDetector(det_);
            det_ = DMCreateDetector();
        }

        DMError process_image(sensor_msgs::Image image,
                clopema_calibration::Dotmatrix &dotmatrix)
        {
            DMError err = DMERROR_OK;
            // Check Initialization?

            // Convert image to format accepted by DMA process

            // Convert image to OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return DMERROR_CANT_READ_FILE;
            }

            // Convert Image to GrayScale
            cv::Mat gray;
            if (cv_ptr->image.channels() > 1) { 
                ROS_INFO("Converting RGB image to gray-scale.");
                cv::cvtColor(cv_ptr->image, gray, CV_RGB2GRAY);
            } else {
                gray = cv_ptr->image;
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

            // Run DMA set_algorithm_from_file
            err = DMProcessImage(det_,&ref);
            if (err != DMERROR_OK) {
                ROS_ERROR("Failed to process image, err: %d", err);
                return err;
            }

            // collect found dots
            const DMDot * dots;
            const DMDot * gauge;
            int           ndots;
            int           ngauge;

            DMGetDetectedDots(det_,&ndots,&dots);

            // collect arranged dots
            DMGetArrangedDots(gauge_,&ngauge,&gauge);



            for (int j = 0; j < ngauge; ++j) {
                for(int i = 0; i < ndots; ++i)
                {
                    if (gauge[j].posRow == dots[i].posRow && gauge[j].posCol == dots[i].posCol) {

                        clopema_calibration::DotmatrixDot dot;
                        dot.imcol = dots[i].imgCol;
                        dot.imrow = dots[i].imgRow;
                        dot.imarea = dots[i].imgArea;
                        dot.gaugecol = gauge[j].posCol;
                        dot.gaugerow = gauge[j].posRow;
                        dotmatrix.dots.push_back(dot);
                        break;
                    }
                }
            }

            return err;
        }

        DMError add_image_algorithm(std::string command)
        {
            DMError err = DMERROR_OK;
            err = DMAddImageAlgorithm(det_, command.c_str());
            ROS_INFO("DMAddImageAlgorithm(det, \"%s\") = %d", command.c_str(), err);
            return err;
        }

        DMError add_shape_algorithm(std::string command)
        {
            DMError err = DMERROR_OK;
            err = DMAddShapeAlgorithm(det_, command.c_str());
            ROS_INFO("DMAddShapeAlgorithm(det, \"%s\") = %d", command.c_str(), err);
            return err;
        }

        DMError add_detection_algorithm(std::string command)
        {
            DMError err = DMERROR_OK;
            err = DMAddDetectionAlgorithm(det_, command.c_str());
            ROS_INFO("DMAddDetectionAlgorithm(det, \"%s\") = %d", command.c_str(), err);
            return err;
        }

        DMError set_gauge_from_file(std::string file_name)
        {
            DMError err;;
            gauge_ = DMCreateGauge();
            err = DMLoadGdfFile(gauge_,file_name.c_str());
            return err;
        }

        DMError set_algorithm_from_file(std::string file_name)
        {
            char line[80];
            char * command;
            int  s;

            // Open file
            FILE * f = fopen(file_name.c_str(),"rt");

            if (!f) {
                printf("Error opening file: %s\n", file_name.c_str());
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
                        err = DMAddImageAlgorithm(det_, command);
                        ROS_INFO("DMAddImageAlgorithm(det, \"%s\") = %d", command, err);
                        break;
                    case 'S':
                        s = strlen(line);
                        command = (char*) malloc(s - 2);
                        strncpy(command, line+2, s - 3);
                        command[s-3] = '\0';
                        err = DMAddShapeAlgorithm(det_, command);
                        ROS_INFO("DMAddShapeAlgorithm(det, \"%s\") = %d", command, err);
                        break;
                    case 'D':
                        s = strlen(line);
                        command = (char*) malloc(s - 2);
                        strncpy(command, line+2, s - 3);
                        command[s-3] = '\0';
                        err = DMAddDetectionAlgorithm(det_, command);
                        ROS_INFO("DMAddDetectionAlgorithm(det, \"%s\") = %d", command, err);
                        break;
                    default:
                        ROS_WARN("Unrecognised command: '%s'\n", line);
                        err = DMERROR_INVALID_SYNTAX;
                        break;
                }

                if (err != DMERROR_OK) {
                    ROS_ERROR("Error: %d\n", err);
                    ret = err;
                }
            }

            fclose(f);

            return ret;
        }



        DMDetector det_;
        DMGauge gauge_;
};
