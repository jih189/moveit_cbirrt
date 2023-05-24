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
#include "constrained_ompl_planners/CMPNetrrt.h"
#include <limits>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::CMPNETRRT::CMPNETRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "CMPNETRRTintermediate" : "CMPNETRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &CMPNETRRT::setRange, &CMPNETRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &CMPNETRRT::setGoalBias, &CMPNETRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &CMPNETRRT::setIntermediateStates, &CMPNETRRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;

    // initial a traditional planner for constrained based MPNet.
    traditional_planner = std::make_shared<ompl::geometric::RRTConnect> (si);

    // set the models of encoder and mlp
    // Deserialize the ScriptModule from a file using torch::jit::load().
    encoder_model = torch::jit::load("/root/mpnet_fetch/pt_dir/encoder_model.pt");
    mlp_model = torch::jit::load("/root/mpnet_fetch/pt_dir/mlp_model.pt");

    encoder_model.to(at::kCUDA);
    mlp_model.to(at::kCUDA);

    // q_max = {-1.6056, -1.221,-3.14159, -2.251, -3.14159, -2.16, -3.14159};
    // q_min = {1.6056, 1.518, 3.14159, 2.251, 3.14159, 2.16, 3.14159};
}

ompl::geometric::CMPNETRRT::~CMPNETRRT()
{
    freeMemory();
}

void ompl::geometric::CMPNETRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::CMPNETRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::CMPNETRRT::freeMemory()
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

ompl::base::PlannerStatus ompl::geometric::CMPNETRRT::solve(const base::PlannerTerminationCondition &ptc)
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

    // convert both start and goal state into two vectors.
    pis_.restart();

    // get obstacle as pointcloud
    std::vector<torch::jit::IValue> env_inputs;
    env_inputs.push_back(torch::from_blob(obstacle_point_cloud_.data(), {1, 6000}).to(at::kCUDA));
    at::Tensor env_output = encoder_model.forward(env_inputs).toTensor().to(at::kCPU);

    std::vector<base::State *> path1;
    std::vector<base::State *> path2;

    path1.push_back(si_->cloneState(start_state));
    path2.push_back(si_->cloneState(goal_state));

    int step = 0;
    bool target_reached = false;
    std::vector<torch::jit::IValue> mlp_input;

    int numOfAttempt = 30;

    while(step < numOfAttempt)
    {
        base::State * path_1_end = path1.back();
        base::State * path_2_end = path2.back();
        // check whether forward path and backward path are connected or not.
        if(si_->checkMotion(path_1_end, path_2_end))
        {
            target_reached = true;
            break;
        }
        // get start goal tensor
        std::vector<float> path1_vec;
        std::vector<float> path2_vec;
        for(int i = 0; i < si_->getStateDimension(); i++)
        {
            path1_vec.push_back((float)*(si_->getStateSpace()->getValueAddressAtIndex(path_1_end, i)));
            path2_vec.push_back((float)*(si_->getStateSpace()->getValueAddressAtIndex(path_2_end, i)));
        }

        // normalize both start and goal joint
        std::vector<float> normalized_path1_vec = joint_normalize(path1_vec, joint_min, joint_max);
        std::vector<float> normalized_path2_vec = joint_normalize(path2_vec, joint_min, joint_max);

        // create tensor for both start and goal joint
        torch::Tensor path1_tensor = torch::from_blob(normalized_path1_vec.data(), {1, si_->getStateDimension()});
        torch::Tensor path2_tensor = torch::from_blob(normalized_path2_vec.data(), {1, si_->getStateDimension()});

        torch::Tensor mlp_input_tensor;

        if(step % 2 == 0)
            mlp_input_tensor = torch::cat({path1_tensor, path2_tensor, env_output}, 1).to(at::kCUDA);
        else
            mlp_input_tensor = torch::cat({path2_tensor, path1_tensor, env_output}, 1).to(at::kCUDA);

        // predict the next step
        mlp_input.push_back(mlp_input_tensor);

        bool predict_fesible = false;
        base::State * next_state = si_->allocState();

        int sample_step = 0;
        while(!predict_fesible && sample_step < 50)
        {
            torch::Tensor mlp_output = mlp_model.forward(mlp_input).toTensor().to(at::kCPU);
            std::vector<float> normalized_predict_vec(mlp_output.data_ptr<float>(), mlp_output.data_ptr<float>() + mlp_output.numel());
            std::vector<float> predict_vec = joint_unnormalize(normalized_predict_vec, joint_min, joint_max);
            
            for(int i = 0; i < si_->getStateDimension(); i++)
                *(si_->getStateSpace()->getValueAddressAtIndex(next_state, i)) = predict_vec[i];

            // need to shorten the next step if the distance is too far.
            if(step % 2 == 0)
            {
                double d = si_->distance(path_1_end, next_state);
                if( d > maxDistance_ )
                {
                    si_->getStateSpace()->interpolate(path_1_end, next_state, maxDistance_ / d, next_state);
                    if (si_->equalStates(path_1_end, next_state))
                        continue;
                }
            }
            else
            {
                double d = si_->distance(path_2_end, next_state);
                if( d > maxDistance_ )
                {
                    si_->getStateSpace()->interpolate(path_2_end, next_state, maxDistance_ / d, next_state);
                    if (si_->equalStates(path_2_end, next_state))
                        continue;
                }
            }

            // project the state to the manifold
            si_->getStateSpace()->as<base::ProjectedStateSpace>()->getConstraint()->project(next_state);

            // check feasible of the new state
            if(si_->isValid(next_state))
            {
                predict_fesible = true;
            }
            else
            {
                sample_step++;
            }
        }

        if(step % 2 == 0)
            path1.push_back(next_state);
        else
            path2.push_back(next_state);

        mlp_input.clear();

        if(!predict_fesible) // if sample is not possible, then break
            break;

        step++;
    }

    if(!target_reached)
    {
        // no solution, so we need to clean memory.
        for (int i = 0; i < path1.size(); i++)
            si_->freeState(path1[i]);
        for (int i = 0; i < path2.size(); i++)
            si_->freeState(path2[i]);
        // fail to find a possible solution
        return base::PlannerStatus::INFEASIBLE;
    }

    // now we have the waypoints, combine them into a single vector.
    std::vector<base::State *> waypoints = path1;
    waypoints.insert(waypoints.end(), path2.rbegin(), path2.rend());
    // std::cout << "the predicted waypoint length = " << waypoints.size() << std::endl;
    // waypoints.push_back(si_->cloneState(start_state));
    // waypoints.push_back(si_->cloneState(goal_state));
    
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

            traditional_planner->clear();

            // pass the local problem def to the planner
            traditional_planner->setProblemDefinition(local_pdef);

            if(!traditional_planner->isSetup())
                traditional_planner->setup();

            // if the next point is the last one, then we can set with larger planning time. You can set a larger time here to gurantee the success rate.
            ompl::base::PlannerStatus local_solved = traditional_planner->ompl::base::Planner::solve((i + 1 == waypoints.size() - 1) ? 0.4 : 0.2);

            if (local_solved)
            {
                ompl::base::PathPtr local_path = local_pdef->getSolutionPath();
                for(int j = 0; j < local_path->as<ompl::geometric::PathGeometric>()->getStateCount(); j++)
                    solution_path.push_back(si_->cloneState(local_path->as<ompl::geometric::PathGeometric>()->getState(j)));

                continue;
            }
            else
            {
                if( i + 1 == waypoints.size() - 1) // if the next state is the last state, then we can't skip it.
                {
                    solved = false;
                    break;
                }
                else{
                    // over write the next state with the current state so we can skip
                    si_->freeState(waypoints[i+1]);
                    waypoints[i+1] = si_->cloneState(waypoints[i]);
                    continue;
                }
            }
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

void ompl::geometric::CMPNETRRT::getPlannerData(base::PlannerData &data) const
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

void ompl::geometric::CMPNETRRT::setObstaclePointcloud(std::vector<float>& pc)
{
    obstacle_point_cloud_ = pc;
}