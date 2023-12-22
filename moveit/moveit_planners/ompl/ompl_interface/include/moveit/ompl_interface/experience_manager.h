#include "ros/ros.h"
#include <moveit_msgs/ConstructAtlas.h>
#include <moveit_msgs/ResetAtlas.h>
#include <moveit/planning_interface/planning_interface.h>
#include "constrained_ompl_planners/JiamingAtlasChart.h"
// #include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include "constrained_ompl_planners/JiamingAtlasStateSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/WrapperStateSpace.h>
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/PDF.h"
#include <moveit/ompl_interface/detail/ompl_constraints.h>

namespace ompl_interface
{
namespace ob = ompl::base;


class ExperienceManager {
private:
    // atlas database is a map where we use foliation id, co-parameter id, and distribution id as the key to store Atlas.
    std::map<std::tuple<int, int, int>, std::shared_ptr<ob::JiamingAtlasStateSpace>> atlas_database_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::shared_ptr<ob::JiamingAtlasStateSpace> experience_state_space_;

    ros::NodeHandle node_handle_;
    ros::ServiceServer construct_atlas_of_roadmap_service_;
    ros::ServiceServer clean_atlas_of_roadmap_services_;

public:
    ExperienceManager(moveit::core::RobotModelConstPtr robot_model);
    ~ExperienceManager();
    bool constructAtlasOfRoadmapService(moveit_msgs::ConstructAtlas::Request& req, moveit_msgs::ConstructAtlas::Response& res);
    bool cleanAtlasOfRoadmapService(moveit_msgs::ResetAtlas::Request& req, moveit_msgs::ResetAtlas::Response& res);
    void cleanAtlasDatabase();
    std::tuple<std::shared_ptr<ob::JiamingAtlasStateSpace>, float> extract_atlas(const std::vector<std::tuple<int, int, int, std::vector<std::tuple<int, float>>>>& task_node_sequence, const moveit_msgs::MotionPlanRequest& req, const planning_scene::PlanningSceneConstPtr& planning_scene);
};

}