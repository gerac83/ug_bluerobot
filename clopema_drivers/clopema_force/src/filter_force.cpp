/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: Nov 13, 2013
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *
 *  Description: Filter data from topic '/in' by computing mean value during the period.
 *      Result is published to the topic '/out'.
 */

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace geometry_msgs;

boost::shared_ptr<ros::Publisher> pub_out;
WrenchStampedPtr summation;
unsigned int number_of_received_msg;
double period;

void force_cb(const WrenchStampedConstPtr& msg) {
    if (!summation) {
        summation = boost::make_shared<WrenchStamped>(*msg);
        number_of_received_msg = 1;
        return;
    }

    number_of_received_msg++;
    summation->wrench.force.x += msg->wrench.force.x;
    summation->wrench.force.y += msg->wrench.force.y;
    summation->wrench.force.z += msg->wrench.force.z;
    summation->wrench.torque.x += msg->wrench.torque.x;
    summation->wrench.torque.y += msg->wrench.torque.y;
    summation->wrench.torque.z += msg->wrench.torque.z;

    double td = msg->header.stamp.toSec() - summation->header.stamp.toSec();
    if (fabs(td) >= period) {
        summation->header.stamp = ros::Time(
                (msg->header.stamp.toSec() + summation->header.stamp.toSec())
                        / 2.0);
        summation->wrench.force.x /= number_of_received_msg;
        summation->wrench.force.y /= number_of_received_msg;
        summation->wrench.force.z /= number_of_received_msg;
        summation->wrench.torque.x /= number_of_received_msg;
        summation->wrench.torque.y /= number_of_received_msg;
        summation->wrench.torque.z /= number_of_received_msg;
        pub_out->publish(summation);
        summation.reset();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "filter_force");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    node.param<double>("period", period, 0.08);

    ros::Subscriber sub = node.subscribe("/in", 1, force_cb);

    pub_out = boost::make_shared<ros::Publisher>(
            node.advertise<WrenchStamped>("/out", 1));

    ros::waitForShutdown();
    spinner.stop();
    return 0;
}
