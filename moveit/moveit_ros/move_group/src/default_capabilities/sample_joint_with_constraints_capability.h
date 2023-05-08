#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetJointWithConstraints.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>

namespace move_group
{
class MoveGroupSampleJointWithConstraints : public MoveGroupCapability
{
public:
  MoveGroupSampleJointWithConstraints();

  void initialize() override;

private:
    bool sampleJointWithConstraintsService(moveit_msgs::GetJointWithConstraints::Request& req, moveit_msgs::GetJointWithConstraints::Response& res);

//   void computeIK(moveit_msgs::PositionIKRequest& req, moveit_msgs::RobotState& solution,
//                  moveit_msgs::MoveItErrorCodes& error_code, moveit::core::RobotState& rs,
//                  const moveit::core::GroupStateValidityCallbackFn& constraint =
//                      moveit::core::GroupStateValidityCallbackFn()) const;

  ros::ServiceServer sample_with_constraints_service_;
};
}  // namespace move_group