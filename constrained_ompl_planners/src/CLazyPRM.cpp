/* Author: Jiaming Hu */

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/tools/config/SelfConfig.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/foreach.hpp>
#include <queue>

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>// remove later

#include <ompl/base/goals/GoalStates.h>

#include "constrained_ompl_planners/CLazyPRM.h"
#include "GoalVisitor.hpp"

#define foreach BOOST_FOREACH

namespace ompl
{
    namespace magic
    {
        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS_LAZY = 5;

        /** \brief When optimizing solutions with lazy planners, this is the minimum
            number of path segments to add before attempting a new optimized solution
            extraction */
        static const unsigned int MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION = 5;
    }
}

ompl::geometric::CLazyPRM::CLazyPRM(const base::SpaceInformationPtr &si, bool starStrategy)
  : base::Planner(si, "CLazyPRM")
  , starStrategy_(starStrategy)
  , indexProperty_(boost::get(boost::vertex_index_t(), g_))
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , vertexComponentProperty_(boost::get(vertex_component_t(), g_))
  , vertexValidityProperty_(boost::get(vertex_flags_t(), g_))
  , edgeValidityProperty_(boost::get(edge_flags_t(), g_))
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    Planner::declareParam<double>("range", this, &CLazyPRM::setRange, &CLazyPRM::getRange, "0.:1.:10000.");
    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &CLazyPRM::setMaxNearestNeighbors,
                                            std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this]
                               {
                                   return getIterationCount();
                               });
    addPlannerProgressProperty("best cost REAL", [this]
                               {
                                   return getBestCost();
                               });
    addPlannerProgressProperty("milestone count INTEGER", [this]
                               {
                                   return getMilestoneCountString();
                               });
    addPlannerProgressProperty("edge count INTEGER", [this]
                               {
                                   return getEdgeCountString();
                               });
}

ompl::geometric::CLazyPRM::CLazyPRM(const base::PlannerData &data, bool starStrategy)
  : CLazyPRM(data.getSpaceInformation(), starStrategy)
{
    if (data.numVertices() > 0)
    {
        // mapping between vertex id from PlannerData and Vertex in Boost.Graph
        std::map<unsigned int, Vertex> vertices;
        // helper function to create vertices as needed and update the vertices mapping
        const auto &getOrCreateVertex = [&](unsigned int vertex_index) {
            if (!vertices.count(vertex_index))
            {
                const auto &data_vertex = data.getVertex(vertex_index);
                Vertex graph_vertex = boost::add_vertex(g_);
                stateProperty_[graph_vertex] = si_->cloneState(data_vertex.getState());
                vertexValidityProperty_[graph_vertex] = VALIDITY_UNKNOWN;
                unsigned long int newComponent = componentCount_++;
                vertexComponentProperty_[graph_vertex] = newComponent;
                vertices[vertex_index] = graph_vertex;
            }
            return vertices.at(vertex_index);
        };

        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });

        for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
        {
            Vertex m = getOrCreateVertex(vertex_index);
            std::vector<unsigned int> neighbor_indices;
            data.getEdges(vertex_index, neighbor_indices);
            for (const unsigned int neighbor_index : neighbor_indices)
            {
                Vertex n = getOrCreateVertex(neighbor_index);
                base::Cost weight;
                data.getEdgeWeight(vertex_index, neighbor_index, &weight);
                const Graph::edge_property_type properties(weight);
                const Edge &edge = boost::add_edge(m, n, properties, g_).first;
                edgeValidityProperty_[edge] = VALIDITY_UNKNOWN;
                uniteComponents(m, n);
            }
            nn_->add(m);
        }
    }
}

ompl::geometric::CLazyPRM::~CLazyPRM()
{
    clear();
};

void ompl::geometric::CLazyPRM::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }
    if (!connectionStrategy_)
        setDefaultConnectionStrategy();
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &)
        {
            return true;
        };

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective()){
            opt_ = pdef_->getOptimizationObjective();
	    // set the cost threhold to infinite so once the planner find the solution, it will return that.
            opt_->setCostThreshold(opt_->infiniteCost());
	}
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            if (!starStrategy_)
                opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    sampler_ = si_->allocStateSampler();
}

void ompl::geometric::CLazyPRM::setRange(double distance)
{
    maxDistance_ = distance;
    if (!userSetConnectionStrategy_)
        setDefaultConnectionStrategy();
    if (isSetup())
        setup();
}

// void ompl::geometric::CLazyPRM::addExperienceWaypoint(std::vector<double> waypoint)
// {
//     base::State* workState = si_->allocState();
//     si_->getStateSpace()->copyFromReals(workState, waypoint);

//     addMilestone(workState);
// }

void ompl::geometric::CLazyPRM::setMaxNearestNeighbors(unsigned int k)
{
    if (starStrategy_)
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_ = KBoundedStrategy<Vertex>(k, maxDistance_, nn_);
    if (isSetup())
        setup();
}

void ompl::geometric::CLazyPRM::setDefaultConnectionStrategy()
{
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }

    if (starStrategy_)
        connectionStrategy_ = KStarStrategy<Vertex>([this] { return milestoneCount(); }, nn_, si_->getStateDimension());
    else
        connectionStrategy_ = KBoundedStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS_LAZY, maxDistance_, nn_);
}

void ompl::geometric::CLazyPRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::CLazyPRM::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::CLazyPRM::clearValidity()
{
    foreach (const Vertex v, boost::vertices(g_))
        vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
    foreach (const Edge e, boost::edges(g_))
        edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
}

void ompl::geometric::CLazyPRM::clear()
{
    Planner::clear();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();

    componentCount_ = 0;
    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::CLazyPRM::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

ompl::geometric::CLazyPRM::Vertex ompl::geometric::CLazyPRM::addMilestone(base::State *state)
{
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    
    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);
    foreach (Vertex n, neighbors)
        if (connectionFilter_(m, n))
        {
            if(opt_->motionCost(stateProperty_[m], stateProperty_[n]).value() == 0.0) // if there is a duplicate, remove the new one and return the old one.
            {
                boost::remove_vertex(m, g_);
                return n;
            }
        }


    vertexValidityProperty_[m] = VALIDITY_UNKNOWN;
    unsigned long int newComponent = componentCount_++;
    vertexComponentProperty_[m] = newComponent;
    componentSize_[newComponent] = 1;

    foreach (Vertex n, neighbors)
        if (connectionFilter_(m, n))
        {
            const base::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
            const Graph::edge_property_type properties(weight);
            const Edge &e = boost::add_edge(m, n, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
            uniteComponents(m, n);
        }

    nn_->add(m);

    return m;
}

ompl::base::PlannerStatus ompl::geometric::CLazyPRM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
    {
        startM_.push_back(addMilestone(si_->cloneState(st)));
    }

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // time::point addgoalstart = time::now(); // the add goal should be replaced later.

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }
    
    // time::point addgoalend = time::now();
    // std::cout << "-------------------------------------Time to add goal states: " << time::seconds(addgoalend - addgoalstart) << std::endl;

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    bestCost_ = opt_->infiniteCost();
    base::State *workState = si_->allocState();
    std::pair<std::size_t, std::size_t> startGoalPair;
    base::PathPtr bestSolution;
    bool fullyOptimized = false;
    bool someSolutionFound = false;
    unsigned int optimizingComponentSegments = 0;

    time::point start = time::now();

    // Grow roadmap in lazy fashion -- add edges without checking validity
    while (!ptc)
    {
        sampler_->sampleUniform(workState); // need to check the number of constraints on the sampler
        // if the sampled point is not valid, then continue.
        if(not si_->isValid(workState))
            continue;
    
        ++iterations_;
	
        Vertex addedVertex = addMilestone(si_->cloneState(workState));

        const long int solComponent = solutionComponent(&startGoalPair);
        // If the start & goal are connected and we either did not find any solution
        // so far or the one we found still needs optimizing and we just added an edge
        // to the connected component that is used for the solution, we attempt to
        // construct a new solution.
        if (solComponent != -1 &&
            (!someSolutionFound || (long int)vertexComponentProperty_[addedVertex] == solComponent))
        {
            // If we already have a solution, we are optimizing. We check that we added at least
            // a few segments to the connected component that includes the previously found
            // solution before attempting to construct a new solution.
            if (someSolutionFound)
            {
                if (++optimizingComponentSegments < magic::MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION)
                    continue;
                optimizingComponentSegments = 0;
            }
            Vertex startV = startM_[startGoalPair.first];
            Vertex goalV = goalM_[startGoalPair.second];
            base::PathPtr solution;
            do
            {
                solution = constructSolution(startV, goalV);
            } while (!solution && vertexComponentProperty_[startV] == vertexComponentProperty_[goalV] && !ptc);
            if (solution)
            {
                someSolutionFound = true;
                base::Cost c = solution->cost(opt_);
                if (opt_->isSatisfied(c))
                {
                    fullyOptimized = true;
                    bestSolution = solution;
                    bestCost_ = c;
                    break; // once the solution is found, then return it.
                }
                if (opt_->isCostBetterThan(c, bestCost_))
                {
                    bestSolution = solution;
                    bestCost_ = c;
                }
            }
        }

        // [jiaming hu] add goal state into goalM_ if there are new goal
        // const base::State *newGoal = pis_.nextGoal();
        // if (newGoal != nullptr)
        //     goalM_.push_back(addMilestone(si_->cloneState(newGoal)));
    }
    
    time::point end = time::now();
    std::cout << "----------------------------------------actual planning time: " << time::seconds(end - start) << "------------------------------" << std::endl;

    si_->freeState(workState);

    if (bestSolution)
    {
        base::PlannerSolution psol(bestSolution);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, fullyOptimized);
        pdef_->addSolutionPath(psol);
    }

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    return bestSolution ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::CLazyPRM::uniteComponents(Vertex a, Vertex b)
{
    unsigned long int componentA = vertexComponentProperty_[a];
    unsigned long int componentB = vertexComponentProperty_[b];
    if (componentA == componentB)
        return;
    if (componentSize_[componentA] > componentSize_[componentB])
    {
        std::swap(componentA, componentB);
        std::swap(a, b);
    }
    markComponent(a, componentB);
}

void ompl::geometric::CLazyPRM::markComponent(Vertex v, unsigned long int newComponent)
{
    std::queue<Vertex> q;
    q.push(v);
    while (!q.empty())
    {
        Vertex n = q.front();
        q.pop();
        unsigned long int &component = vertexComponentProperty_[n];
        if (component == newComponent)
            continue;
        if (componentSize_[component] == 1)
            componentSize_.erase(component);
        else
            componentSize_[component]--;
        component = newComponent;
        componentSize_[newComponent]++;
        boost::graph_traits<Graph>::adjacency_iterator nbh, last;
        for (boost::tie(nbh, last) = boost::adjacent_vertices(n, g_); nbh != last; ++nbh)
            q.push(*nbh);
    }
}

long int ompl::geometric::CLazyPRM::solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const
{
    for (std::size_t startIndex = 0; startIndex < startM_.size(); ++startIndex)
    {
        long int startComponent = vertexComponentProperty_[startM_[startIndex]];
        for (std::size_t goalIndex = 0; goalIndex < goalM_.size(); ++goalIndex)
        {
            if (startComponent == (long int)vertexComponentProperty_[goalM_[goalIndex]])
            {
                startGoalPair->first = startIndex;
                startGoalPair->second = goalIndex;
                return startComponent;
            }
        }
    }
    return -1;
}

ompl::base::PathPtr ompl::geometric::CLazyPRM::constructSolution(const Vertex &start, const Vertex &goal)
{
    // Need to update the index map here, becuse nodes may have been removed and
    // the numbering will not be 0 .. N-1 otherwise.
    unsigned long int index = 0;
    boost::graph_traits<Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index)
        indexProperty_[*vi] = index;

    boost::property_map<Graph, boost::vertex_predecessor_t>::type prev;
    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(g_, start,
                            [this, goal](Vertex v)
                            {
                                return costHeuristic(v, goal);
                            },
                            boost::predecessor_map(prev)
                                .distance_compare([this](base::Cost c1, base::Cost c2)
                                                  {
                                                      return opt_->isCostBetterThan(c1, c2);
                                                  })
                                .distance_combine([this](base::Cost c1, base::Cost c2)
                                                  {
                                                      return opt_->combineCosts(c1, c2);
                                                  })
                                .distance_inf(opt_->infiniteCost())
                                .distance_zero(opt_->identityCost())
                                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }
    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");

    // First, get the solution states without copying them, and check them for validity.
    // We do all the node validity checks for the vertices, as this may remove a larger
    // part of the graph (compared to removing an edge).
    std::vector<const base::State *> states(1, stateProperty_[goal]);
    std::set<Vertex> milestonesToRemove;
    for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos])
    {
        const base::State *st = stateProperty_[pos];
        unsigned int &vd = vertexValidityProperty_[pos];
        if ((vd & VALIDITY_TRUE) == 0)
            if (si_->isValid(st))
                vd |= VALIDITY_TRUE;
        if ((vd & VALIDITY_TRUE) == 0)
            milestonesToRemove.insert(pos);
        if (milestonesToRemove.empty())
            states.push_back(st);
    }


    // We remove *all* invalid vertices. This is not entirely as described in the original CLazyPRM
    // paper, as the paper suggest removing the first vertex only, and then recomputing the
    // shortest path. Howeve, the paper says the focus is on efficient vertex & edge removal,
    // rather than collision checking, so this modification is in the spirit of the paper.
    if (!milestonesToRemove.empty())
    {
        unsigned long int comp = vertexComponentProperty_[start];
        // Remember the current neighbors.
        std::set<Vertex> neighbors;
        for (auto it = milestonesToRemove.begin(); it != milestonesToRemove.end(); ++it)
        {
            boost::graph_traits<Graph>::adjacency_iterator nbh, last;
            for (boost::tie(nbh, last) = boost::adjacent_vertices(*it, g_); nbh != last; ++nbh)
                if (milestonesToRemove.find(*nbh) == milestonesToRemove.end())
                    neighbors.insert(*nbh);
            // Remove vertex from nearest neighbors data structure.
            nn_->remove(*it);
            // Free vertex state.
            si_->freeState(stateProperty_[*it]);
            // Remove all edges.
            boost::clear_vertex(*it, g_);
            // Remove the vertex.
            boost::remove_vertex(*it, g_);
        }
        // Update the connected component ID for neighbors.
        for (auto neighbor : neighbors)
        {
            if (comp == vertexComponentProperty_[neighbor])
            {
                unsigned long int newComponent = componentCount_++;
                componentSize_[newComponent] = 0;
                markComponent(neighbor, newComponent);
            }
        }
        return base::PathPtr();
    }

    // start is checked for validity already
    states.push_back(stateProperty_[start]);

    // Check the edges too, if the vertices were valid. Remove the first invalid edge only.
    std::vector<const base::State *>::const_iterator prevState = states.begin(), state = prevState + 1;
    Vertex prevVertex = goal, pos = prev[goal];
    do
    {
        Edge e = boost::lookup_edge(pos, prevVertex, g_).first;
        unsigned int &evd = edgeValidityProperty_[e];
        if ((evd & VALIDITY_TRUE) == 0)
        {
            if (si_->checkMotion(*state, *prevState)){
                evd |= VALIDITY_TRUE;
            }
        }
        if ((evd & VALIDITY_TRUE) == 0)
        {
            boost::remove_edge(e, g_);
            unsigned long int newComponent = componentCount_++;
            componentSize_[newComponent] = 0;
            markComponent(pos, newComponent);
            return base::PathPtr();
        }
        prevState = state;
        ++state;
        prevVertex = pos;
        pos = prev[pos];
    } while (prevVertex != pos);

    auto p(std::make_shared<PathGeometric>(si_));
    for (std::vector<const base::State *>::const_reverse_iterator st = states.rbegin(); st != states.rend(); ++st)
        p->append(*st);
    return p;
}

ompl::base::Cost ompl::geometric::CLazyPRM::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

void ompl::geometric::CLazyPRM::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // generate a list of states
    std::vector<const base::State *> savedStates(num_vertices(g_));

    unsigned long int index = 0;
    boost::graph_traits<Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index)
    {
        indexProperty_[*vi] = index;
        savedStates[index] = si_->cloneState(stateProperty_[*vi]);
    }

    // Adding only valid edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {

        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);

        // need to find the cost of the edge
        double weight = weightProperty_[e].value();

        // If the edge is invalid, we need to negate the weight
        if ((edgeValidityProperty_[e] & VALIDITY_TRUE) == 0)
        {
            weight *= -1;
        }

        data.addEdge(base::PlannerDataVertex(savedStates[indexProperty_[v1]]), base::PlannerDataVertex(savedStates[indexProperty_[v2]]), base::PlannerDataEdge(), base::Cost(weight));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(savedStates[indexProperty_[v2]]), base::PlannerDataVertex(savedStates[indexProperty_[v1]]), base::PlannerDataEdge(), base::Cost(weight));

        // Add tags for the newly added vertices
        data.tagState(savedStates[indexProperty_[v1]], (vertexValidityProperty_[v1] & VALIDITY_TRUE) == 0 ? 0 : 1);
        data.tagState(savedStates[indexProperty_[v2]], (vertexValidityProperty_[v2] & VALIDITY_TRUE) == 0 ? 0 : 1);
    }
}

void ompl::geometric::CLazyPRM::loadPlannerGraph(base::PlannerData &data)
{
    if (data.numVertices() > 0)
    {
        // mapping between vertex id from PlannerData and Vertex in Boost.Graph
        std::map<unsigned int, Vertex> vertices;
        // helper function to create vertices as needed and update the vertices mapping
        const auto &getOrCreateVertex = [&](unsigned int vertex_index) {
            if (!vertices.count(vertex_index))
            {
                const auto &data_vertex = data.getVertex(vertex_index);
                Vertex graph_vertex = boost::add_vertex(g_);
                stateProperty_[graph_vertex] = si_->cloneState(data_vertex.getState());
                vertexValidityProperty_[graph_vertex] = data_vertex.getTag() == 0 ? VALIDITY_UNKNOWN : VALIDITY_TRUE;
                unsigned long int newComponent = componentCount_++;
                vertexComponentProperty_[graph_vertex] = newComponent;
                vertices[vertex_index] = graph_vertex;
            }
            return vertices.at(vertex_index);
        };

        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b);});

        for (size_t vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index)
        {
            Vertex m = getOrCreateVertex(vertex_index);
            std::vector<unsigned int> neighbor_indices;
            data.getEdges(vertex_index, neighbor_indices);
            for (const unsigned int neighbor_index : neighbor_indices)
            {
                Vertex n = getOrCreateVertex(neighbor_index);
                base::Cost weight;
                data.getEdgeWeight(vertex_index, neighbor_index, &weight);
                const Graph::edge_property_type properties(base::Cost(abs(weight.value())));
                const Edge &edge = boost::add_edge(m, n, properties, g_).first;
                edgeValidityProperty_[edge] = weight.value() >= 0 ? VALIDITY_TRUE: VALIDITY_UNKNOWN;
                uniteComponents(m, n);
            }
            nn_->add(m);
        }
    }

    setDefaultConnectionStrategy();

}