/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2014-01-14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Check trajectory for collision service
 */

#include "traj_collision_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>

move_group::TrajCollisionService::TrajCollisionService(): MoveGroupCapability("TrajCollisionService") {
}

void move_group::TrajCollisionService::initialize() {
    this->collision_service_ = root_node_handle_.advertiseService(TRAJ_COLLISION_SERVICE_NAME, &TrajCollisionService::computeService, this);
}

bool move_group::TrajCollisionService::computeService(clopema_moveit::TrajCollision::Request& req, clopema_moveit::TrajCollision::Response& res) {
     using namespace std;
     vector<string> allow_collision_1 = req.enable_collision_1;
     vector<string> allow_collision_2 = req.enable_collision_2;
     
     planning_scene::PlanningScenePtr ps;
     ps = planning_scene::PlanningScene::clone(planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_));
     
     bool valid = false;
     for(unsigned int i = 0; i < allow_collision_1.size(); ++i) {
        ps->getAllowedCollisionMatrixNonConst().setEntry(allow_collision_1[i], allow_collision_2[i], true);
     }
     
     valid = ps->isPathValid(req.start_state, req.rtraj, "", true);
     
     for(unsigned int i = 0; i < allow_collision_1.size(); ++i) {
         ps->getAllowedCollisionMatrixNonConst().setEntry(allow_collision_1[i], allow_collision_2[i], false);
     }
     
     res.valid = valid;
     return true;
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::TrajCollisionService, move_group::MoveGroupCapability)


