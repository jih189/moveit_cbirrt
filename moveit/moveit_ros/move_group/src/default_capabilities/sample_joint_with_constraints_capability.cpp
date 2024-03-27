#include "sample_joint_with_constraints_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupSampleJointWithConstraints::MoveGroupSampleJointWithConstraints() : MoveGroupCapability("SampleJointWithConstraints")
{
}

void MoveGroupSampleJointWithConstraints::initialize()
{
  sample_with_constraints_service_ =
      root_node_handle_.advertiseService(SAMPLE_JOINT_WITH_CONSTRAINTS_NAME, &MoveGroupSampleJointWithConstraints::sampleJointWithConstraintsService, this);
}

bool MoveGroupSampleJointWithConstraints::sampleJointWithConstraintsService(moveit_msgs::GetJointWithConstraints::Request& req,
                                                  moveit_msgs::GetJointWithConstraints::Response& res)
{
    int max_num_of_attemp = req.max_sampling_attempt <= 0 ? 10 : req.max_sampling_attempt;
    context_->planning_scene_monitor_->updateFrameTransforms();

    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);

    // initialize a constraint sampler
    constraint_samplers::IKConstraintSampler constraint_sampler(static_cast<const planning_scene::PlanningSceneConstPtr&>(ls), req.group_name);

    constraint_sampler.configure(req.constraints);
    
    moveit::core::RobotState sampled_state = ls->getCurrentState();
    const moveit::core::JointModelGroup* jmg = sampled_state.getJointModelGroup(req.group_name);
    if (jmg)
    {
        for(int i = 0; i < max_num_of_attemp; i++)
        {
            // if(constraint_sampler.sample(sampled_state, sampled_state, 10) && 
            //    !(static_cast<const planning_scene::PlanningSceneConstPtr&>(ls).get()->isStateColliding(sampled_state, jmg->getName())))
            // {
            //     moveit_msgs::RobotState m;
            //     moveit::core::robotStateToRobotStateMsg(sampled_state, m, true);
            //     res.solutions.push_back(m);
            // }
            if(constraint_sampler.sample(sampled_state, sampled_state, 10))
            {
                if(!(static_cast<const planning_scene::PlanningSceneConstPtr&>(ls).get()->isStateColliding(sampled_state, jmg->getName())))
                {
                    moveit_msgs::RobotState m;
                    moveit::core::robotStateToRobotStateMsg(sampled_state, m, true);
                    res.solutions.push_back(m);
                }
            }
            else
            {
                return true;
            }
        }
    }

  return true;
}


}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupSampleJointWithConstraints, move_group::MoveGroupCapability)
