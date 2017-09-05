/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan */

#include "ompl/geometric/planners/rrt/RRTstarmodded.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include <algorithm>
#include <limits>
#include <map>
#include <boost/math/constants/constants.hpp>

ompl::geometric::RRTstarmodded::RRTstarmodded(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTstarmodded") {
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    delayCC_ = true;
    lastGoalMotion_ = NULL;

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    distanceDirection_ = FROM_NEIGHBORS;

    Planner::declareParam<double>("range", this, &RRTstarmodded::setRange, &RRTstarmodded::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstarmodded::setGoalBias, &RRTstarmodded::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstarmodded::setDelayCC, &RRTstarmodded::getDelayCC, "0,1");

    addPlannerProgressProperty("iterations INTEGER",
                               std::bind(&RRTstarmodded::getIterationCount, this));
    addPlannerProgressProperty("collision checks INTEGER",
                               std::bind(&RRTstarmodded::getCollisionCheckCount, this));
    addPlannerProgressProperty("best cost REAL",
                               std::bind(&RRTstarmodded::getBestCost, this));
}

ompl::geometric::RRTstarmodded::~RRTstarmodded() {
    freeMemory();
}

void ompl::geometric::RRTstarmodded::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if(!nn_)
//        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    //nn_->setDistanceFunction(std::bind(&RRTstarmodded::distanceFunction, this, _1, _2));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });


    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if(pdef_) {
        if(pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
            opt_.reset(new base::PathLengthOptimizationObjective(si_));
        }
    } else {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void ompl::geometric::RRTstarmodded::clear() {
    Planner::clear();
    sampler_.reset();
//     freeMemory(); //NOT free memory because nn_ is still valid so the planner runs are afected
//     if(nn_)
//         nn_->clear();

    lastGoalMotion_ = NULL;
    goalMotions_.clear();

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());

}

bool ompl::geometric::RRTstarmodded::add_start_state(ompl::geometric::RRTstarmodded::Motion*& added_motion) {
    //return pointer to motion if start state was added to tree or NULL otherwise
    pis_.restart();
    while(const base::State *st = pis_.nextStart()) {
        Motion *rmotion = new Motion(si_);
        si_->copyState(rmotion->state, st);

        //check whether can be added
        bool start_state_added = true;
        base::State *xstate  = si_->allocState();
        base::State *dstate = rmotion->state; //destnation state

        Motion *nmotion = nn_->nearest(rmotion); // find state to add to the tree
        double d = si_->distance(nmotion->state, rmotion->state);
        if(d > maxDistance_) {
            si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, xstate);
            dstate = xstate;
            start_state_added = false;
        }

        bool tmp_bool;
        start_state_added = add_new_state(nmotion, dstate, added_motion, tmp_bool);

        si_->freeState(xstate);
        if(rmotion->state)
            si_->freeState(rmotion->state);
        delete rmotion;

        if(start_state_added) {
            return true;
        }
    }
    return false;
}

bool ompl::geometric::RRTstarmodded::add_new_state(ompl::geometric::RRTstarmodded::Motion*& nmotion, ompl::base::State*& dstate, ompl::geometric::RRTstarmodded::Motion*& motion, bool& check_for_solution) {
    ++collisionChecks_;
    if(!si_->checkMotion(nmotion->state, dstate)) {
        return false;
    }

    bool symDist = si_->getStateSpace()->hasSymmetricDistance();
    bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
    bool symCost = opt_->isSymmetric();
    double k_rrg           = boost::math::constants::e<double>() +
                             (boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

    std::vector<Motion*>       nbh;
    std::vector<base::Cost>    costs;
    std::vector<base::Cost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;

    std::vector<int>           valid;
    unsigned int               rewireTest = 0;
    unsigned int               statesGenerated = 0;

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    // create a motion
    motion = new Motion(si_);
    si_->copyState(motion->state, dstate);
    motion->parent = nmotion;
    motion->incCost = opt_->motionCost(nmotion->state, motion->state);
    motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

    // Find nearby neighbors of the new motion - k-nearest RRT*
    unsigned int k = std::ceil(k_rrg * log((double)(nn_->size() + 1)));
    nn_->nearestK(motion, k, nbh);

    rewireTest += nbh.size();
    statesGenerated++;

    // cache for distance computations
    //
    // Our cost caches only increase in size, so they're only
    // resized if they can't fit the current neighborhood
    if(costs.size() < nbh.size()) {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
    }

    // cache for motion validity (only useful in a symmetric space)
    //
    // Our validity caches only increase in size, so they're
    // only resized if they can't fit the current neighborhood
    if(symDist && symInterp) {
        if(valid.size() < nbh.size())
            valid.resize(nbh.size());
        std::fill(valid.begin(), valid.begin() + nbh.size(), 0);
    }

    // Finding the nearest neighbor to connect to
    // By default, neighborhood states are sorted by cost, and collision checking
    // is performed in increasing order of cost
    if(delayCC_) {
        // calculate all costs and distances
        for(std::size_t i = 0 ; i < nbh.size(); ++i) {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for(std::size_t i = 0; i < nbh.size(); ++i)
            sortedCostIndices[i] = i;
        std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(),
                  compareFn);

        // collision check until a valid motion is found
        //
        // ASYMMETRIC CASE: it's possible that none of these
        // neighbors are valid. This is fine, because motion
        // already has a connection to the tree through
        // nmotion (with populated cost fields!).
        for(std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                i != sortedCostIndices.begin() + nbh.size();
                ++i) {
            if(nbh[*i] != nmotion)
                ++collisionChecks_;
            if(nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state)) {
                motion->incCost = incCosts[*i];
                motion->cost = costs[*i];
                motion->parent = nbh[*i];
                if(symDist && symInterp)
                    valid[*i] = 1;
                break;
            } else if(symDist && symInterp)
                valid[*i] = -1;
        }
    } else { // if not delayCC
        motion->incCost = opt_->motionCost(nmotion->state, motion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
        // find which one we connect the new state to
        for(std::size_t i = 0 ; i < nbh.size(); ++i) {
            if(nbh[i] != nmotion) {
                incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                if(opt_->isCostBetterThan(costs[i], motion->cost)) {
                    ++collisionChecks_;
                    if(si_->checkMotion(nbh[i]->state, motion->state)) {
                        motion->incCost = incCosts[i];
                        motion->cost = costs[i];
                        motion->parent = nbh[i];
                        if(symDist && symInterp)
                            valid[i] = 1;
                    } else if(symDist && symInterp)
                        valid[i] = -1;
                }
            } else {
                incCosts[i] = motion->incCost;
                costs[i] = motion->cost;
                if(symDist && symInterp)
                    valid[i] = 1;
            }
        }
    }

    // add motion to the tree
    nn_->add(motion);
    motion->parent->children.push_back(motion);

    // rewire tree if needed
    //
    // Set directionality of distance function to be FROM new
    // state TO neighbors, since this is how the routing
    // should occur in tree rewiring
    if(!symDist) {
        distanceDirection_ = TO_NEIGHBORS;
        nn_->nearestK(motion, k, nbh);
        rewireTest += nbh.size();
    }

    for(std::size_t i = 0; i < nbh.size(); ++i) {
        if(nbh[i] != motion->parent) {
            base::Cost nbhIncCost;
            if(symDist && symCost)
                nbhIncCost = incCosts[i];
            else
                nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
            base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
            if(opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost)) {
                bool motionValid;
                if(symDist && symInterp) {
                    if(valid[i] == 0) {
                        ++collisionChecks_;
                        motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                    } else
                        motionValid = (valid[i] == 1);

                } else {
                    ++collisionChecks_;
                    motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                }
                if(motionValid) {
                    // Remove this node from its parent list
                    removeFromParent(nbh[i]);

                    // Add this node to the new parent
                    nbh[i]->parent = motion;
                    nbh[i]->incCost = nbhIncCost;
                    nbh[i]->cost = nbhNewCost;
                    nbh[i]->parent->children.push_back(nbh[i]);

                    // Update the costs of the node's children
                    updateChildCosts(nbh[i]);

                    check_for_solution = true;
                }
            }
        }
    }
    return true;
}

void ompl::geometric::RRTstarmodded::make_new_root(ompl::geometric::RRTstarmodded::Motion* root, ompl::geometric::RRTstarmodded::Motion* last_node) {
    if(root->parent != 0) {
        make_new_root(root->parent, root);
    }
    root->parent = last_node;
    if(last_node != 0) {
        removeFromParent(last_node);
        last_node->children.push_back(root);
        root->incCost = last_node->incCost;
    } else {
        root->incCost = opt_->identityCost();
        root->cost = opt_->identityCost();
        updateChildCosts(root);
    }

    return;
}

ompl::base::PlannerStatus ompl::geometric::RRTstarmodded::solve(const base::PlannerTerminationCondition & ptc) {
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    if(nn_->size() == 0) { //Add one start state only
        if(const base::State *st = pis_.nextStart()) {
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->cost = opt_->identityCost(); //parent is start state!
            nn_->add(motion);
        }
    }

    if(nn_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if(!sampler_)
        sampler_ = si_->allocStateSampler();

    const unsigned int SAMPLING_SIZE = 1000;
    bool first_run = false;
    unsigned int loop_count = 0;
    if(nn_->size() < SAMPLING_SIZE) {
        first_run = true;
    }
    if(!sample_states(SAMPLING_SIZE)) {
        OMPL_ERROR("Cannot sample initial states in the tree");
        throw Exception("Cannot sample states");
    }

    const ompl::base::PathPtr straight = pdef_->isStraightLinePathValid();
    if(straight) {
        pdef_->addSolutionPath(straight);
        OMPL_INFORM("Found a straight path not calling starmodded");
        return base::PlannerStatus::EXACT_SOLUTION;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution       = lastGoalMotion_;

    base::Cost bestCost    = opt_->infiniteCost();
    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion        = new Motion(si_);
    base::State *rstate    = rmotion->state;
    base::State *xstate    = si_->allocState();

    if(solution)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(), solution->cost.value());


    bool start_state_added = false;
    Motion* start_state_pointer = NULL;
    while(ptc == false || first_run) {
        if(first_run) {
            loop_count++;
            if(loop_count > 50) {
                first_run = false;
            }
        }

        iterations_++;
        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
        if(goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        if(!start_state_added) {
            start_state_added = add_start_state(start_state_pointer);
            if(start_state_added) {
                make_new_root(start_state_pointer, NULL);
                OMPL_INFORM("Start state succesfully added to the tree");
            }
        }

        bool symDist = si_->getStateSpace()->hasSymmetricDistance();
        if(!symDist)
            distanceDirection_ = FROM_NEIGHBORS;

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if(d > maxDistance_) {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        Motion* motion;
        bool checkForSolution = false;
        if(add_new_state(nmotion, dstate, motion, checkForSolution)) {
            double distanceFromGoal;
            if(goal->isSatisfied(motion->state, &distanceFromGoal)) {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if(checkForSolution) {
                if(start_state_added) {
                    ompl::base::Cost start_cost = start_state_pointer->cost;
                    updateChildCosts(start_state_pointer);
                    for(size_t i = 0; i < goalMotions_.size(); ++i) {
                        ompl::base::Cost tmp_cost = opt_->combineCosts(goalMotions_[i]->cost, start_cost);
//                         OMPL_ERROR("Solution found with cost/best: %f %f %f", tmp_cost.v, bestCost.v, start_cost.v);

                        if(opt_->isCostBetterThan(tmp_cost, bestCost)) {
                            bestCost = tmp_cost;
                            bestCost_ = bestCost;
                        }

                        sufficientlyShort = opt_->isSatisfied(tmp_cost);
                        if(sufficientlyShort) {
                            solution = goalMotions_[i];
                            break;
                        } else if(!solution || opt_->isCostBetterThan(tmp_cost, solution->cost))
                            solution = goalMotions_[i];
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if(goalMotions_.size() == 0 && distanceFromGoal < approximatedist) {
                approximation = motion;
                approximatedist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if(solution && sufficientlyShort)
            break;
    }

    bool approximate = (solution == 0);
    bool addedSolution = false;
    if(approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if(solution != 0 && start_state_added) {
        // construct the solution path

        Motion* end_solution = NULL;

//         std::vector<Motion*> mpath_from_start;
//         start_state_pointer = start_state_pointer->parent; //To not have the start node two times in trajectory
//         while(start_state_pointer != 0) {
//             mpath_from_start.push_back(start_state_pointer);
//             not necessary to go to the parrent //i.e. dynamic time wrapping?
//             Motion* tmp = solution;
//             while(tmp != 0) { //filter trajectory
//                 if(tmp == start_state_pointer) {
//                     end_solution = start_state_pointer;
//                     OMPL_ERROR("Simplification found at: %d", mpath_from_start.size());
//                     break;
//                 }
//                 tmp = tmp->parent;
//             }
//             if(end_solution != 0) {
//                 break;
//             }
//             start_state_pointer = start_state_pointer->parent;
//         }
//         std::reverse(mpath_from_start.begin(), mpath_from_start.end());

        std::vector<Motion*> mpath;
        while(solution != end_solution) {
            mpath.push_back(solution);
            solution = solution->parent;
        }
//         mpath.insert(mpath.end(), mpath_from_start.begin(), mpath_from_start.end());

        // set the solution path
        PathGeometric *geoPath = new PathGeometric(si_);
        for(int i = mpath.size() - 1 ; i >= 0 ; --i)
            geoPath->append(mpath[i]->state);

        base::PathPtr path(geoPath);
        // Add the solution path, whether it is approximate (not reaching the goal), and the
        // distance from the end of the path to the goal (-1 if satisfying the goal).
        //base::PlannerSolution psol(path, approximate, approximate ? approximatedist : -1.0, getName());
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        psol.setApproximate(approximate ? approximatedist : -1.0);
        // Does the solution satisfy the optimization objective?
        psol.optimized_ = sufficientlyShort;

        pdef_->addSolutionPath(psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if(rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::RRTstarmodded::removeFromParent(Motion * m) {
    std::vector<Motion*>::iterator it = m->parent->children.begin();
    while(it != m->parent->children.end()) {
        if(*it == m) {
            it = m->parent->children.erase(it);
            it = m->parent->children.end();
        } else
            ++it;
    }
}

void ompl::geometric::RRTstarmodded::updateChildCosts(Motion * m) {
    for(std::size_t i = 0; i < m->children.size(); ++i) {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RRTstarmodded::freeMemory() {
    if(nn_) {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for(std::size_t i = 0 ; i < motions.size() ; ++i) {
            if(motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::RRTstarmodded::getPlannerData(base::PlannerData & data) const {
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if(nn_)
        nn_->list(motions);

    if(lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for(std::size_t i = 0 ; i < motions.size() ; ++i) {
        if(motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
    data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
    data.properties["collision_checks INTEGER"] =
        boost::lexical_cast<std::string>(collisionChecks_);
}

std::string ompl::geometric::RRTstarmodded::getIterationCount() const {
    return boost::lexical_cast<std::string>(iterations_);
}
std::string ompl::geometric::RRTstarmodded::getCollisionCheckCount() const {
    return boost::lexical_cast<std::string>(collisionChecks_);
}
std::string ompl::geometric::RRTstarmodded::getBestCost() const {
    //return boost::lexical_cast<std::string>(bestCost_.v);
    return boost::lexical_cast<std::string>(bestCost_.value());
}

bool ompl::geometric::RRTstarmodded::sample_states(unsigned int min_size) {
    if(!si_ || !opt_ || !nn_) {
        OMPL_ERROR("Global variables needs to be initialized before the sampling is called");
        return false;
    }

    if(nn_->size() > min_size) {
        return true;
    }

    if(!sampler_) {
        sampler_ = si_->allocStateSampler();
    }

    try {
        checkValidity();
    } catch(...) {
        OMPL_ERROR("State is not valid for sampling");
        return false;
    }

    bool symDist = si_->getStateSpace()->hasSymmetricDistance();
    bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
    Motion *rmotion        = new Motion(si_);
    base::State *rstate    = rmotion->state;
    base::State *xstate    = si_->allocState();

    // e+e/d.  K-nearest RRT*
    double k_rrg           = boost::math::constants::e<double>() +
                             (boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

    std::vector<Motion *>       nbh;

    std::vector<base::Cost>    costs;
    std::vector<base::Cost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;

    std::vector<int>           valid;
    unsigned int               rewireTest = 0;
    unsigned int               statesGenerated = 0;

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    while(nn_->size() < min_size) {
        if(nn_->size() % 100 == 0) {
            OMPL_INFORM("Sampled %d / %d states", nn_->size(), min_size);
        }
        sampler_->sampleUniform(rstate);

        // Set directionality of nearest neighbors computation to be FROM neighbors TO new state
        if(!symDist)
            distanceDirection_ = FROM_NEIGHBORS;

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if(d > maxDistance_) {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if(!nmotion || !nmotion->state || !dstate) {
            continue;
        }

        if(si_->checkMotion(nmotion->state, dstate)) {
            // create a motion
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

            // Find nearby neighbors of the new motion - k-nearest RRT*
            unsigned int k = std::ceil(k_rrg * log((double)(nn_->size() + 1)));
            nn_->nearestK(motion, k, nbh);
            rewireTest += nbh.size();
            statesGenerated++;

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if(costs.size() < nbh.size()) {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }

            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if(symDist && symInterp) {
                if(valid.size() < nbh.size())
                    valid.resize(nbh.size());

                std::fill(valid.begin(), valid.begin() + nbh.size(), 0);
            }

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            if(delayCC_) {
                // calculate all costs and distances
                for(std::size_t i = 0 ; i < nbh.size(); ++i) {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for(std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;

                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(),
                          compareFn);

                // collision check until a valid motion is found
                //
                // ASYMMETRIC CASE: it's possible that none of these
                // neighbors are valid. This is fine, because motion
                // already has a connection to the tree through
                // nmotion (with populated cost fields!).
                for(std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                        i != sortedCostIndices.begin() + nbh.size();
                        ++i) {
                    if(nbh[*i] != nmotion)
                        ++collisionChecks_;

                    if(!si_) continue;

                    if(nbh[*i] == nmotion || (si_->checkMotion(nbh[*i]->state, motion->state))) {
                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
                        motion->parent = nbh[*i];

                        if(symDist && symInterp)
                            valid[*i] = 1;

                        break;
                    } else if(symDist && symInterp)
                        valid[*i] = -1;
                }
            } else { // if not delayCC
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

                // find which one we connect the new state to
                for(std::size_t i = 0 ; i < nbh.size(); ++i) {
                    if(nbh[i] != nmotion) {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);

                        if(opt_->isCostBetterThan(costs[i], motion->cost)) {
                            ++collisionChecks_;

                            if(si_->checkMotion(nbh[i]->state, motion->state)) {
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
                                motion->parent = nbh[i];

                                if(symDist && symInterp)
                                    valid[i] = 1;
                            } else if(symDist && symInterp)
                                valid[i] = -1;
                        }
                    } else {
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;

                        if(symDist && symInterp)
                            valid[i] = 1;
                    }
                }
            }


            nn_->add(motion);// add motion to the tree
            motion->parent->children.push_back(motion);
        }
    }

    si_->freeState(xstate);
    if(rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    return true;
}


