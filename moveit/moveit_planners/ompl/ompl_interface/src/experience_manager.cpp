#include <moveit/ompl_interface/experience_manager.h>
namespace ompl_interface
{
ExperienceManager::ExperienceManager(moveit::core::RobotModelConstPtr robot_model)
{
    robot_model_ = robot_model;
    construct_atlas_of_roadmap_service_ =
        node_handle_.advertiseService("construct_atlas", &ExperienceManager::constructAtlasOfRoadmapService, this);
    clean_atlas_of_roadmap_services_ = 
        node_handle_.advertiseService("reset_atlas", &ExperienceManager::cleanAtlasOfRoadmapService, this);

}

ExperienceManager::~ExperienceManager()
{
    cleanAtlasDatabase();
}

bool ExperienceManager::constructAtlasOfRoadmapService(moveit_msgs::ConstructAtlas::Request& req,
                                                  moveit_msgs::ConstructAtlas::Response& res)
{
    // std::cout << "construct Atlas in manifold (" << req.foliation_id << ", " << req.co_parameter_id << ")" << std::endl;
    OMPL_INFORM("Construct Atlas in manifold ( %d , %d ) with %d configurations", req.foliation_id, req.co_parameter_id, req.list_of_configuration_with_info.size());
    // create the ompl constraints
    // req.constraints
    ob::ConstraintPtr ompl_constraint = 
	      createOMPLConstraints(robot_model_, req.group_name, req.constraints, req.default_state);

    uint robot_joint_number = robot_model_->getJointModelGroup(req.group_name)->getVariableCount();

    // ////////////////////////
    // const auto temp_space = std::make_shared<ob::RealVectorStateSpace>(robot_joint_number);

    // // set upper and lower bound
    // ob::RealVectorBounds bounds(robot_joint_number);
    // for(uint k = 0 ; k < robot_joint_number; k++){
    //     const robot_model::JointModel::Bounds& moveit_bounds = robot_model_->getJointModelGroup(req.group_name)->getActiveJointModels()[k]->getVariableBounds();
    //     bounds.setLow(k, moveit_bounds[0].min_position_);
    //     bounds.setHigh(k, moveit_bounds[0].max_position_);
    // }
    // temp_space->setBounds(bounds);

    // temp_space->setup();
    // std::shared_ptr<ob::JiamingAtlasStateSpace> atlas_temp = std::make_shared<ob::JiamingAtlasStateSpace>(temp_space, ompl_constraint);
    // atlas_temp->setEpsilon(0.4);
    // atlas_temp->setRho(0.4);

    // ///////////////////////

    for(uint i = 0; i < req.list_of_configuration_with_info.size(); i++)
    {
        // std::cout << "config " << i << ": ";
        // for(uint c = 0; c < req.list_of_configuration_with_info[i].joint_configuration.size() ; c++)
        //     std::cout << req.list_of_configuration_with_info[i].joint_configuration[c] << " ";
        // std::cout << " with distribution id " << req.list_of_configuration_with_info[i].distribution_id << std::endl;

        // check if the atlas_database_ contains the node which this configuration belongs to
        std::tuple<int, int, int> tuple_key{req.foliation_id, req.co_parameter_id, req.list_of_configuration_with_info[i].distribution_id};

        if(atlas_database_.find(tuple_key) == atlas_database_.end())
        {
            // atlas_database does not contain the current node.
            const auto space = std::make_shared<ob::RealVectorStateSpace>(robot_joint_number);

            // set upper and lower bound
            ob::RealVectorBounds bounds(robot_joint_number);
            for(uint k = 0 ; k < robot_joint_number; k++){
                const robot_model::JointModel::Bounds& moveit_bounds = robot_model_->getJointModelGroup(req.group_name)->getActiveJointModels()[k]->getVariableBounds();
                bounds.setLow(k, moveit_bounds[0].min_position_);
                bounds.setHigh(k, moveit_bounds[0].max_position_);
            }
            space->setBounds(bounds);

            space->setup();
            atlas_database_[tuple_key] = std::make_shared<ob::JiamingAtlasStateSpace>(space, ompl_constraint);
            atlas_database_[tuple_key]->setEpsilon(0.4);
            atlas_database_[tuple_key]->setRho(0.4);
        }

        ob::State *state = atlas_database_[tuple_key]->allocState();

        for(size_t j = 0; j < robot_joint_number; j++)
            state->as<ompl::base::WrapperStateSpace::StateType>()->getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = req.list_of_configuration_with_info[i].joint_configuration[j];

        // add the state into the Atlas
        atlas_database_.at(tuple_key)->getChart(state->as<ob::JiamingAtlasStateSpace::StateType>());

        // atlas_temp->getChart(state->as<ob::JiamingAtlasStateSpace::StateType>()); //TODO
    }
    
    // std::cout << "number of chart " << atlas_temp->getChartCount() << " with " << req.list_of_configuration_with_info.size() << " configurations" << std::endl;
    // // // try to sample in this AtlasStatespace for debuging
    // auto sampler = atlas_temp->allocDefaultStateSampler();
    // ob::State *temp_state = atlas_temp->allocState();
    // sampler->sampleUniform(temp_state);
    // atlas_temp->freeState(temp_state);

    OMPL_INFORM("Construct Atlas in manifold done.");

    return true;
}

bool ExperienceManager::cleanAtlasOfRoadmapService(moveit_msgs::ResetAtlas::Request& req, moveit_msgs::ResetAtlas::Response& res)
{
    cleanAtlasDatabase();
    res.success = true;
    return true;
}

void ExperienceManager::cleanAtlasDatabase(){

    OMPL_INFORM("Clean Atlas Dataset.");
    for(auto& pair: atlas_database_)
    {
        std::shared_ptr<ob::JiamingAtlasStateSpace> state_space = pair.second;
        state_space->clear();
    }
    atlas_database_.clear();
    if(experience_state_space_.get() != nullptr)
        experience_state_space_->clear();
    experience_state_space_.reset();
    OMPL_INFORM("Clean Atlas Dataset Done.");
}

std::vector<float> ExperienceManager::softmax(const std::vector<float>& input) {
    std::vector<float> result;
    float sum_exp = 0.0;

    // Calculate the sum of exponentials of input elements
    for (float value : input) {
        sum_exp += std::exp(value);
    }

    // Calculate softmax for each element
    for (float value : input) {
        float softmax_value = std::exp(value) / sum_exp;
        result.push_back(softmax_value);
    }

    return result;
}

std::shared_ptr<ob::JiamingAtlasStateSpace> ExperienceManager::extract_atlas(
    const std::vector<std::tuple<int, int, int, std::vector<std::tuple<int, float, float>>>>& task_node_sequence,
    const moveit_msgs::MotionPlanRequest& req,
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    float &atlas_distribution_ratio)
{
    // // create a experience state space which is combined by different Atlas from manifolds.
    ob::ConstraintPtr ompl_constraint = 
        createOMPLConstraints(robot_model_, req.group_name, req.path_constraints, planning_scene);

    uint robot_joint_number = robot_model_->getJointModelGroup(req.group_name)->getVariableCount();

    const auto space = std::make_shared<ob::RealVectorStateSpace>(robot_joint_number);

    // set upper and lower bound
    ob::RealVectorBounds bounds(robot_joint_number);
    for(uint k = 0 ; k < robot_joint_number; k++){
        const robot_model::JointModel::Bounds& moveit_bounds = robot_model_->getJointModelGroup(req.group_name)->getActiveJointModels()[k]->getVariableBounds();
        bounds.setLow(k, moveit_bounds[0].min_position_);
        bounds.setHigh(k, moveit_bounds[0].max_position_);
    }
    space->setBounds(bounds);

    space->setup();

    // reset experience state space
    if(experience_state_space_.get() != nullptr)
        experience_state_space_->clear();

    experience_state_space_.reset(new ob::JiamingAtlasStateSpace(space, ompl_constraint));
    experience_state_space_->setEpsilon(0.4);
    experience_state_space_->setRho(0.4);

    // extract different Atlas from different manifolds
    std::vector<std::tuple<int, int, int, float>> all_related_nodes;
    std::vector<float> beta_weight;
    std::vector<float> beta_values;
    
    // The element of task_node_sequence has the format as following
    // (foliation id, co-paramenter id, distribution id, [(related co-parameter id, related beta, related similarity)])
    for(auto node: task_node_sequence) // which is a list of task node equence with experience.
    {
        for(auto related_node: std::get<3>(node))
        {
            // if the beta time similarity score is too low, the ignore it.
            if(std::get<1>(related_node) * std::get<2>(related_node) < 0.1)
                continue;
            
            // std::cout << std::get<0>(related_node) << " " << std::get<1>(related_node) << " | ";
            all_related_nodes.push_back(
                std::tuple<int, int, int, float>{
                    (int)std::get<0>(node),
                    std::get<0>(related_node),
                    (int)std::get<2>(node),
                    std::get<1>(related_node) * std::get<2>(related_node)
                }
            );
            beta_values.push_back(std::get<1>(related_node));
            beta_weight.push_back(std::get<2>(related_node));
        }
    }

    // sort the related node based on the beta time similarity score.
    std::sort(all_related_nodes.begin(), all_related_nodes.end(), [](const std::tuple<int, int, int, float>& a, const std::tuple<int, int, int, float>& b) {
        return std::get<3>(a) > std::get<3>(b);
    });

    // std::cout << "related nodes" << std::endl;
    for(auto n: all_related_nodes)
    {
        std::shared_ptr<ob::JiamingAtlasStateSpace> ss = atlas_database_.at(std::tuple<int, int, int>{std::get<0>(n), std::get<1>(n), std::get<2>(n)});
        for(uint chart_index = 0; chart_index < ss->getChartCount(); chart_index++)
        {
            experience_state_space_->combineChart(ss->getChart(chart_index)->getOrigin(), (double)std::get<3>(n));
        }
    }

    atlas_distribution_ratio = 0;

    // the atlas-distribution sampling ratio is calculate the element multiplication between soft_max(similarity) and beta.
    if(beta_weight.size() > 0){
        std::vector<float> beta_weight_after_soft_max = softmax(beta_weight);
        for(unsigned int i = 0; i < beta_weight.size(); i++)
            atlas_distribution_ratio += (beta_weight_after_soft_max[i] * beta_weight[i]);
    }

    // combine the atlas from different nodes to the new statespace.
    return experience_state_space_;
}
}