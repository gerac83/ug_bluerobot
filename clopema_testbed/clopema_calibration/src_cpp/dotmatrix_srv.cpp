// Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
//
// Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
// Institute:   Czech Technical University in Prague
// Created on:  Apr 7, 2014

// ---- Includes ------------------------------------------------------------ //
#include <ros/ros.h>
#include <clopema_calibration/dotmatrix_ros.h>
#include <clopema_calibration/DetectDotmatrix.h>
#include <dynamic_reconfigure/server.h>
#include <clopema_calibration/DotmatrixConfig.h>
#include "boost/format.hpp"

// ---- Constants ----------------------------------------------------------- //

const std::string node_name = "dotmatrix";

Dotmatrix dma;
// ---- Helper functions definitions ---------------------------------------- //


bool srv_callback(clopema_calibration::DetectDotmatrix::Request& req,
              clopema_calibration::DetectDotmatrix::Response& response);

void dr_callback(clopema_calibration::DotmatrixConfig &config, uint32_t level);

// ---- Main ---------------------------------------------------------------- //

int main(int argc, char **argv)
{
    DMError err;

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");


    // Get parameters
    std::string algorithm_path;
    std::string gauge_path;

    if (!n.getParam("algorithm", algorithm_path))
    {
        ROS_INFO("Algorithm was not set from a file, usig the dynamic reconfigure.");

        // Setup dynamic reconfigure
        dynamic_reconfigure::Server<clopema_calibration::DotmatrixConfig> server;
        dynamic_reconfigure::Server<clopema_calibration::DotmatrixConfig>::CallbackType f;

        f = boost::bind(&dr_callback, _1, _2);
        server.setCallback(f);
    } else {
        // Prevent parameter to stay on a parameter server
        n.deleteParam("algorithm");

        // Set algorithm from a file
        err = dma.set_algorithm_from_file(algorithm_path);
        if (err != DMERROR_OK) {
            ROS_ERROR("Unable to set algorithm from file, err: %d", err);
            return 1;
        }
        ROS_INFO("Algorithm set from file: %s", algorithm_path.c_str());
    }

    if (!n.getParam("gauge", gauge_path))
    {
        ROS_ERROR("Gauge was not set.");
        return 1;
    }

    // Set gauge from a file
    dma.set_gauge_from_file(gauge_path);
    if (err != DMERROR_OK) {
        ROS_ERROR("Unable to set gauge from file, err: %d", err);
        return 1;
    }
    ROS_INFO("Gauge set from file: %s", gauge_path.c_str());

    // Register service
    ros::ServiceServer service = n.advertiseService("detect", srv_callback);
    ROS_INFO("Dotmatrix service is ready.");

    ros::spin();

    return 0;
}

// ---- Helper functions implementations ------------------------------------ //

bool srv_callback(clopema_calibration::DetectDotmatrix::Request& req,
              clopema_calibration::DetectDotmatrix::Response& res)
{

    DMError err = dma.process_image(req.image, res.dotmatrix);

    return true;
}

void dr_callback(clopema_calibration::DotmatrixConfig &config, uint32_t level)
{
    dma.reset();
    dma.add_shape_algorithm((boost::format("TraceBorder INSIDE BELOW %d") % config.trace_border_thr).str());
    dma.add_shape_algorithm((boost::format("MinArea %d") % config.min_area_1).str());
    dma.add_shape_algorithm((boost::format("MaxArea %d") % config.max_area).str());
    dma.add_shape_algorithm((boost::format("TriangleNormal %f %f") % config.triangle_normal_minarm % config.triangle_normal_minangle).str());
    dma.add_shape_algorithm((boost::format("MaxDerivSubpix %f %f %f %f") % config.max_deriv_subpix_res % config.max_deriv_subpix_size % config.max_deriv_subpix_sigma % config.max_deriv_subpix_corr).str());
    dma.add_shape_algorithm((boost::format("RequireBelief")).str());
    dma.add_shape_algorithm((boost::format("MinArea %d") % config.min_area_2).str());
    dma.add_shape_algorithm((boost::format("IsElliptic %f %f %f") % config.is_elliptic_minskew % config.is_elliptic_maxskew % config.is_elliptic_maxdist).str());
    dma.add_detection_algorithm((boost::format("DotMatrix %1.1f") % config.dotmatrix_maxgriddist).str());
}
