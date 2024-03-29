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

/* Author: Ioan Sucan */
#include "constrained_ompl_planners/MPNetrrt.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::MPNETRRT::MPNETRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "MPNETRRTintermediate" : "MPNETRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &MPNETRRT::setRange, &MPNETRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &MPNETRRT::setGoalBias, &MPNETRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &MPNETRRT::setIntermediateStates, &MPNETRRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;

    // initial a traditional planner for MPNet.
    traditional_planner = std::make_shared<ompl::geometric::RRT> (si);

    ros::NodeHandle n;
    client_ = n.serviceClient<moveit_msgs::GetNextStep>("next_step_predict");
}

ompl::geometric::MPNETRRT::~MPNETRRT()
{
    freeMemory();
}

void ompl::geometric::MPNETRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::MPNETRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::MPNETRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::MPNETRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    // get start and goal configuration.
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // get both start and goal configurations
    const base::State *start_state = pis_.nextStart();
    const base::State *goal_state = pis_.nextGoal(ptc);

    // std::cout << "start configuration" << std::endl;
    // si_->printState(start_state, std::cout);
    // std::cout << "goal configuration" << std::endl;
    // si_->printState(goal_state, std::cout);

    // // convert both start and goal state into two vectors.
    moveit_msgs::GetNextStep srv;
    // for(int i = 0; i < si_->getStateDimension(); i++)
    // {
    //     srv.request.start_configuration[i] = *(si_->getStateSpace()->getValueAddressAtIndex(start_state, i));
    //     srv.request.goal_configuration[i] = *(si_->getStateSpace()->getValueAddressAtIndex(goal_state, i));
    // }

    pis_.restart();

    // run the MPNet to generate the waypoints
    std::vector<base::State *> waypoints_start;
    std::vector<base::State *> waypoints_goal;
    waypoints_start.push_back(si_->cloneState(start_state));
    waypoints_goal.push_back(si_->cloneState(goal_state));

    // send commands to mpnet for waypoints.
    bool goal_reached = false;
    base::State *current_state_candidate = waypoints_start.back();
    base::State *current_state_target = waypoints_goal.back();
    std::vector<base::State *>* waypoints_candidate = &waypoints_start;
    std::vector<base::State *>* waypoints_target = &waypoints_goal;
    // also include a time limit.
    int time_limit = 50; // 50 seconds... TODO: this is a temporary solution.
    //start the timer.
    ros::Time start_time = ros::Time::now();
    // we plan in a bidirectional way.
    while(!goal_reached)
    {
        // timeout guard.
        if ((ros::Time::now() - start_time).toSec() > time_limit)
        {
            std::cout << "time out" << std::endl;
            break;
        }
        // write srv request.
        for(int i = 0; i < si_->getStateDimension(); i++)
        {
            srv.request.start_configuration[i] = *(si_->getStateSpace()->getValueAddressAtIndex(current_state_candidate, i));
            srv.request.goal_configuration[i] = *(si_->getStateSpace()->getValueAddressAtIndex(current_state_target, i));
        }
        // send the request to the service.
        if(client_.call(srv))
        {
            // get the predicted next step.
            // allocate memory for the next step.
            base::State *next_step = si_->allocState();
            for(int i = 0; i < si_->getStateDimension(); i++)
            {
                *(si_->getStateSpace()->getValueAddressAtIndex(next_step, i)) = srv.response.next_step[i];
            }

            // update the waypoints.
            waypoints_candidate->push_back(next_step);
            

            // check if the goal is reached by trying to directly connect the candidate and the target.
            if (si_->checkMotion(current_state_candidate, current_state_target))
            {
                goal_reached = true;
            }
            else
            {
                // swap the target and candidate.
                std::vector<base::State *>* temp = waypoints_candidate;
                waypoints_candidate = waypoints_target;
                waypoints_target = temp;
                current_state_candidate = current_state_target;
                current_state_target = next_step;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service next_step_predict");
            return base::PlannerStatus::ABORT;
        }
    }

    // now we have the waypoints, combine them into a single vector.
    std::vector<base::State *> waypoints;
    for(int i = 0; i < waypoints_start.size(); i++)
    {
        waypoints.push_back(waypoints_start[i]);
    }
    for(int i = waypoints_goal.size() - 1; i >= 0; i--)
    {
        waypoints.push_back(waypoints_goal[i]);
    }
    

    // Verification.

    std::vector<base::State *> solution_path;
    solution_path.push_back(si_->cloneState(waypoints[0]));

    bool solved = true;

    // verify through the waypoints
    for(int i = 0; i < waypoints.size() - 1; i++)
    {
        base::State *current_state = waypoints[i];
        base::State *next_state = waypoints[i+1];
        if(si_->checkMotion(current_state, next_state)) 
        {
            solution_path.push_back(si_->cloneState(next_state));
        }
        else // if direct motion does not exist, then run tranditional planner.
        {
            // we assume the traditional planner does not find solution always here.
            // initialize a local pdef
            auto local_pdef(std::make_shared<ompl::base::ProblemDefinition>(si_));

            // set the start and goal states
            local_pdef->setStartAndGoalStates(current_state, next_state);

            // pass the local problem def to the planner
            traditional_planner->setProblemDefinition(local_pdef);

            traditional_planner->setup();

            ompl::base::PlannerStatus local_solved = traditional_planner->ompl::base::Planner::solve(1.0);

            if (local_solved)
            {
                ompl::base::PathPtr local_path = local_pdef->getSolutionPath();
                for(int j = 0; j < local_path->as<ompl::geometric::PathGeometric>()->getStateCount(); j++)
                {
                    solution_path.push_back(si_->cloneState(local_path->as<ompl::geometric::PathGeometric>()->getState(j)));
                }

                continue;
            }

            solved = false;
            break;
        }
    }

    if (solved)
    {
        /* construct the solution path */
        auto path(std::make_shared<PathGeometric>(si_));

        for (int i = 0; i < solution_path.size(); i++)
            path->append(solution_path[i]);

        pdef_->addSolutionPath(path, false, 0.0, getName());
    }

    // need to free memory
    for (int i = 0; i < solution_path.size(); i++)
        si_->freeState(solution_path[i]);
    
    for (int i = 0; i < waypoints.size(); i++)
        si_->freeState(waypoints[i]);

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::MPNETRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}