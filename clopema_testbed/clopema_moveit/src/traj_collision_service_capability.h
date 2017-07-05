/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2014-01-14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Check trajectory for collision service (moveit capability - plugin)
 */

#ifndef MOVEIT_MOVE_GROUP_TRAJ_COLLISION_SERVICE_CAPABILITY_
#define MOVEIT_MOVE_GROUP_TRAJ_COLLISION_SERVICE_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <clopema_moveit/TrajCollision.h>

namespace move_group {

static const std::string TRAJ_COLLISION_SERVICE_NAME = "/check_trajectory";

class TrajCollisionService: public MoveGroupCapability {
public:

    TrajCollisionService();
    virtual void initialize();

private:

    /** \brief service callback function */
    bool computeService(clopema_moveit::TrajCollision::Request& req, clopema_moveit::TrajCollision::Response &res);

    ros::ServiceServer collision_service_;
};

}

#endif
