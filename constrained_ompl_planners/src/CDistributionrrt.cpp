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

#include "constrained_ompl_planners/CDistributionrrt.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"

#include <fstream>

ompl::geometric::CDISTRIBUTIONRRT::CDISTRIBUTIONRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "CDISTRIBUTIONRRTIntermediate" : "CDISTRIBUTIONRRT")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &CDISTRIBUTIONRRT::setRange, &CDISTRIBUTIONRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<bool>("intermediate_states", this, &CDISTRIBUTIONRRT::setIntermediateStates,
                                &CDISTRIBUTIONRRT::getIntermediateStates, "0,1");

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::CDISTRIBUTIONRRT::~CDISTRIBUTIONRRT()
{
    freeMemory();
}

void ompl::geometric::CDISTRIBUTIONRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::CDISTRIBUTIONRRT::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::CDISTRIBUTIONRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::CDISTRIBUTIONRRT::GrowState ompl::geometric::CDISTRIBUTIONRRT::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                             Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
            return TRAPPED;

        dstate = tgi.xstate;
        reach = false;
    }

    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                                   si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (!validMotion)
        return TRAPPED;

    if (addIntermediateStates_)
    {
        const base::State *astate = tgi.start ? nmotion->state : dstate;
        const base::State *bstate = tgi.start ? dstate : nmotion->state;

        std::vector<base::State *> states;
        const unsigned int count = si_->getStateSpace()->validSegmentCount(astate, bstate);

        if (si_->getMotionStates(astate, bstate, states, count, true, true))
        {
            // if coming from start, don't add the start state (start->goal)
            if (tgi.start)
                si_->freeState(states[0]);
            // if coming from the goal, don't add the start state (goal->start)
            else
                si_->freeState(states[states.size() - 1]);
        }

        // Add states forwards if from start, backwards if from goal
        const auto &add_state = [&](const auto &state)
        {
            auto *motion = new Motion;
            motion->state = state;
            motion->parent = nmotion;
            motion->root = nmotion->root;
            tree->add(motion);
            nmotion = motion;
        };

        if (tgi.start)
            std::for_each(++std::begin(states), std::end(states), add_state);
        else
            std::for_each(++std::rbegin(states), std::rend(states), add_state);

        tgi.xmotion = nmotion;
    }
    else
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tree->add(motion);

        tgi.xmotion = motion;
    }

    return reach ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus ompl::geometric::CDISTRIBUTIONRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    std::vector<Motion *> goal_states;
    const base::State *st = pis_.nextGoal(ptc);

    while(st != nullptr)
    {
        bool is_new_goal = true;
        for(auto s : goal_states)
        {
            if(si_->distance(st, s->state) < 0.1)
            {
                is_new_goal = false;
                break;
            }
        }
        // this is an new goal state
        if( is_new_goal )
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->root = motion->state;

            goal_states.push_back(motion);
            tGoal_->add(motion);
            st = pis_.nextGoal();
        }
        else
        {
            break;
        }
    }

    if (tGoal_->size() == 0)
    {
        OMPL_ERROR("%s: There is no goal states", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));
                

    // initialize all random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, gaussian_distributions_.size() - 1);

    // clear sampling data
    for(std::pair<base::State *, int> &data : sampling_data_)
        si_->freeState(data.first);
    sampling_data_.clear();

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool solved = false;

    while (!ptc)
    {
        TreeData &tree = startTree_ ? tStart_ : tGoal_;
        tgi.start = startTree_;
        startTree_ = !startTree_;
        TreeData &otherTree = startTree_ ? tStart_ : tGoal_;

        // if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        // {
        //     const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
        //     if (st != nullptr)
        //     {
        //         std::cout << "sample state: ";
        //         for(int i = 0; i < si_->getStateDimension(); i++)
        //             std::cout << st->as<ompl::base::WrapperStateSpace::StateType>()->getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << " ";
        //         std::cout << std::endl;

        //         // check if the sample goal state is in the tGoal_ or not. If it is, then ignore.
        //         auto *motion = new Motion(si_);
        //         si_->copyState(motion->state, st);
        //         motion->root = motion->state;
        //         tGoal_->add(motion);
        //     }

        //     if (tGoal_->size() == 0)
        //     {
        //         OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
        //         break;
        //     }
        // }

        /* sample random state */
        /* When the state is not compounded state, then you should use following code to assign state.
        for(int i = 0; i < si_->getStateDimension(); i++)
            rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = sample_value[i];
        */
        
        // All the sampled configuration will be saved into the sampling_data_ with status.
        /*
            Status cases:
            0: valid after project.
            1: invalid due to state outside bounds or collision detect after project.
            2: invalid due to violates path constraint after project.
            3: invalid due to infeasible after project. This should be ignored later.
            4: invalid due to collision caused by the manipulated object after project.
            ---
            5: valid before project.
            6: invalid due to state outside bounds or collision detect before project or sampled from Atlas.
            7: invalid due to violates path constraint before project or sampled from Atlas.
            8: invalid due to infeasible before project or sampled from Atlas. This should be ignored later.
            9: invalid due to collision caused by the manipulated object before project or sampled from Atlas.
        */

        // this while loop may cause the program to be stuck, so we need to add a counter to avoid this.
        int counter = 0;
        int max_counter = 300;
        bool sampleValid = false;
        double invalid_reason = 0;

        while(!sampleValid && counter < max_counter){

            Eigen::VectorXd sample_value;
            counter++;

            if(gaussian_distributions_.size() != 0 && ((double) rand() / (RAND_MAX)) < sample_ratio_) // if random number is less than sample ratio, then sample from distribution sequence
            {
                // if we are using ALEF, all gaussian's covariance should be zeros. Therefore, in this case,
                // sample_from_distribution_sequence will select one gaussian's mean randomly and return it.
                sample_value = ((double) rand() / (RAND_MAX)) >= atlas_distribution_ratio_ ? 
                                        sample_from_distribution_sequence(gaussian_distributions_, dist, gen) : sample_from_atlas();
            }
            else
            {
                sample_value = sample_from_random();
            }

            // set the joint value
            for(int i = 0; i < si_->getStateDimension(); i++)
                rstate->as<ompl::base::WrapperStateSpace::StateType>()->getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = sample_value[i];
            
            // check validity without project and save it into sampling_data
            sampleValid=si_->getStateValidityChecker()->isValid(rstate, invalid_reason);
            sampling_data_.push_back(std::pair<base::State *, int>( si_->cloneState(rstate), ((int) -invalid_reason) + 5));

            if(!sampleValid && invalid_reason == -2) // if the configuration is invalid due to constraint, then project.
            {
                // Eigen::VectorXd x(si_->getStateDimension());

                // // get the sampled state
                // for(int i = 0; i < si_->getStateDimension(); i++)
                //     x[i] = rstate->as<ompl::base::WrapperStateSpace::StateType>()->getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];

                // // use Newton's method to project the sampled state
                // unsigned int iter = 0;
                // double norm = 0;
                // Eigen::VectorXd f(si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->getCoDimension());
                // Eigen::MatrixXd j(si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->getCoDimension(), si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->getAmbientDimension());

                // const double squaredTolerance = si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->getTolerance() * si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->getTolerance();
                // std::cout << "----" << std::endl;
                // si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->function(x, f);
                // // run the iterative behavior to approach the constraint manifold.
                // // while ((norm = f.squaredNorm()) > squaredTolerance && iter++ <  si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->getMaxIterations())
                // while ((norm = f.squaredNorm()) > squaredTolerance && iter++ <  300)
                // {
                //     si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->jacobian(x, j);
                //     x -= (0.05 * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f));
                //     si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->function(x, f);
                //     std::cout << "error : " << f.squaredNorm() << std::endl;
                // }

                // // set value back
                // for(int i = 0; i < si_->getStateDimension(); i++)
                //     rstate->as<ompl::base::WrapperStateSpace::StateType>()->getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = x[i];

                // project rstate to the constraint manifold and save it and its status into the sampling_data
                if(si_->getStateSpace()->as<base::ConstrainedStateSpace>()->getConstraint()->project(rstate))
                {
                    sampleValid=si_->getStateValidityChecker()->isValid(rstate, invalid_reason);
                    sampling_data_.push_back(std::pair<base::State *, int>( si_->cloneState(rstate), (int) -invalid_reason));
                }
                else
                {
                    // fail to project to manifold
                    sampling_data_.push_back(std::pair<base::State *, int>( si_->cloneState(rstate), 2));
                }
            }
        }

        if(!sampleValid)
            continue;

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            tgi.start = startTree_;

            /* if initial progress cannot be done from the otherTree, restore tgi.start */
            GrowState gsc = growTree(otherTree, tgi, rmotion);
            if (gsc == TRAPPED)
                tgi.start = !tgi.start;

            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);

                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
            else
            {
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (tgi.start)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::CDISTRIBUTIONRRT::getPlannerData(base::PlannerData &data) const
{
    // jiaming: we want to use planner data to keep all sampling data information.
    Planner::getPlannerData(data);

    for(auto &data_pair : sampling_data_)
        data.addVertex(base::PlannerDataVertex(data_pair.first, data_pair.second));
}

void ompl::geometric::CDISTRIBUTIONRRT::setDistribution(std::vector<Eigen::VectorXd>& means, 
                            std::vector<Eigen::MatrixXd>& covariances, double sample_ratio,
                            std::shared_ptr<ompl::base::JiamingAtlasStateSpace> atlas_state_space,
                            float atlas_distribution_ratio)
{
    // check whether the length of both means and convariances are equal.
    if(means.size() != covariances.size())
    {
        OMPL_ERROR("The length of means and covariances are not equal.");
        return;
    }

    // if means and covariances are empty, then we use the default distribution.
    if(means.size() == 0)
    {
        OMPL_WARN("The length of means and covariances are zero. Use the default distribution.");
        return;
    }

    // check whether the dimension of means and covariances are equal.
    if(means[0].size() != covariances[0].rows())
    {
        OMPL_ERROR("The dimension of means and covariances are not equal.");
        return;
    }

    // check whether the dimension of means and covariances are equal to the dimension of state space
    if(si_->getStateDimension() != means[0].size())
    {
        OMPL_ERROR("The dimension of means and covariances are not equal to the dimension of state space.");
        return;
    }

    // check whether the sample ratio is valid
    if(sample_ratio < 0 || sample_ratio > 1)
    {
        OMPL_ERROR("The sample ratio is not valid.");
        return;
    }

    std::cout << "receive distributions' mean" << std::endl;
    // set the distribution.
    gaussian_distributions_.clear();
    for(int i = 0; i < means.size(); i++)
    {
        Gaussian g;
        g.mean = means[i];
        g.covariance = covariances[i];
        gaussian_distributions_.push_back(g);

        std::cout << means[i].transpose() << std::endl;
    }

    // set the sample parameters.
    sample_ratio_ = sample_ratio;
    atlas_state_space_ = atlas_state_space;
    atlas_distribution_ratio_ = atlas_distribution_ratio;
    
    std::cout << "Other sampling informations: " << std::endl;
    std::cout << "number of charts = " << atlas_state_space_->getChartCount() << std::endl;
    std::cout << "sampling ratio = " << sample_ratio_ << std::endl;
    std::cout << "atlas distribution ratio = " << atlas_distribution_ratio << std::endl;
}