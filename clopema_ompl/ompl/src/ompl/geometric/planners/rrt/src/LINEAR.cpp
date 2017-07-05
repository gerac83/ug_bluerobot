/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, modified for the CloPeMa project by Tom Jankovec */

#include <ompl/base/Planner.h>
#include "ompl/geometric/planners/rrt/LINEAR.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <ompl/tools/config/SelfConfig.h>

ompl::geometric::LINEAR::LINEAR(const base::SpaceInformationPtr &si) : base::Planner(si, "LINEAR") {}

ompl::base::PlannerStatus ompl::geometric::LINEAR::solve(const base::PlannerTerminationCondition &ptc){
    
    std::vector<ompl::base::State *> state_vec, goal_vec;
    
    checkValidity();
    
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal){
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
        state_vec.push_back(si_->cloneState(st));

    if (state_vec.size() == 0){
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample()){
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }
    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goal_vec.size() || goal_vec.empty()){
        const base::State *st = goal_vec.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st){
            goal_vec.push_back(si_->cloneState(st));
        }

        if (goal_vec.empty()){
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }
    
    OMPL_INFORM("All is well, proceeding to planning");
    
    const ompl::base::PathPtr straight = pdef_->isStraightLinePathValid();
    
    if(straight){
        pdef_->addSolutionPath(straight);
        OMPL_INFORM("Found a straight path");
        return base::PlannerStatus::EXACT_SOLUTION;
    }
    OMPL_ERROR("Straight path not found");
    return base::PlannerStatus::CRASH;
}
void ompl::geometric::LINEAR::clear(void){
    Planner::clear();
}
void ompl::geometric::LINEAR::setup(void){
    OMPL_ERROR("entered setup");
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
}
void ompl::geometric::LINEAR::getPlannerData(base::PlannerData &data) const{
Planner::getPlannerData(data);
}
