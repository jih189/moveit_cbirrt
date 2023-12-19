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
    std::cout << "Construct atlas server receives " << req.list_of_configuration_with_info.size() << " configurations" << std::endl;
    std::cout << "for manifold (" << req.foliation_id << ", " << req.co_parameter_id << ")" << std::endl;
    // create the ompl constraints
    // req.constraints
    ob::ConstraintPtr ompl_constraint = 
	      createOMPLConstraints(robot_model_, req.group_name, req.constraints, req.default_state);

    uint robot_joint_number = robot_model_->getJointModelGroup(req.group_name)->getVariableCount();

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
        }

        ob::State *state = atlas_database_[tuple_key]->allocState();

        for(size_t j = 0; j < robot_joint_number; j++)
            state->as<ompl::base::WrapperStateSpace::StateType>()->getState()->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = req.list_of_configuration_with_info[i].joint_configuration[j];

        // add the state into the Atlas
        atlas_database_.at(tuple_key)->getChart(state->as<ob::JiamingAtlasStateSpace::StateType>());
        // try to sample in this AtlasStatespace for debuging
        // auto sampler = atlas_database_.at(tuple_key)->allocDefaultStateSampler();
        // ob::State *temp_state = atlas_database_[tuple_key]->allocState();
        // sampler->sampleUniform(temp_state);
        // atlas_database_.at(tuple_key)->freeState(temp_state);
        // break;
    }
    return true;
}

bool ExperienceManager::cleanAtlasOfRoadmapService(moveit_msgs::ResetAtlas::Request& req, moveit_msgs::ResetAtlas::Response& res)
{
    cleanAtlasDatabase();
    res.success = true;
    return true;
}

void ExperienceManager::cleanAtlasDatabase(){
    for(auto& pair: atlas_database_)
    {
        std::shared_ptr<ob::JiamingAtlasStateSpace> state_space = pair.second;
        state_space->clear();
    }
    atlas_database_.clear();
}

std::vector<std::shared_ptr<ob::JiamingAtlasStateSpace>> ExperienceManager::extract_atlas(const std::vector<std::tuple<int, int, int>>& task_node_ids)
{
    std::vector<std::shared_ptr<ob::JiamingAtlasStateSpace>> result;
    for(uint i = 0; i < task_node_ids.size(); i++)
    {
        if(atlas_database_.find(task_node_ids[i]) == atlas_database_.end()){
            result.push_back(nullptr);
        }
        else{
            result.push_back(atlas_database_[task_node_ids[i]]);
        }
    }
    return result;
}
}