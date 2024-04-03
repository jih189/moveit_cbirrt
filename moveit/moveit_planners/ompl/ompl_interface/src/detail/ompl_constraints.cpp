/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of KU Leuven nor the names of its
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

/* Author: Jeroen De Maeyer, Boston Cleek, Berend Pijnenburg */

#include <algorithm>
#include <iterator>

#include <moveit/ompl_interface/detail/ompl_constraints.h>
// #include <moveit/utils/logger.hpp>

#include <tf2_eigen/tf2_eigen.h>

constexpr char LOGNAME[] = "ompl_constraints";

namespace ompl_interface
{

Bounds::Bounds() : size_(0)
{
}

Bounds::Bounds(const std::vector<double>& lower, const std::vector<double>& upper)
  : lower_(lower), upper_(upper), size_(lower.size())
{
  // how to report this in release mode??
  assert(lower_.size() == upper_.size());
}

Eigen::VectorXd Bounds::penalty(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  assert(static_cast<long>(lower_.size()) == x.size());
  Eigen::VectorXd penalty(x.size());

  for (unsigned int i = 0; i < x.size(); ++i)
  {
    if (x[i] < lower_.at(i))
    {
      penalty[i] = x[i] - lower_.at(i);
    }
    else if (x[i] > upper_.at(i))
    {
      penalty[i] = x[i] - upper_.at(i);
    }
    else
    {
      penalty[i] = 0.0;
    }
  }
  return penalty;
}

std::size_t Bounds::size() const
{
  return size_;
}

std::ostream& operator<<(std::ostream& os, const ompl_interface::Bounds& bounds)
{
  os << "Bounds:\n";
  for (std::size_t i{ 0 }; i < bounds.size(); ++i)
  {
    os << "( " << bounds.lower_[i] << ", " << bounds.upper_[i] << " )\n";
  }
  return os;
}

/****************************
 * Base class for constraints
 * **************************/
BaseConstraint::BaseConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                               const unsigned int num_dofs, const unsigned int num_cons, const moveit::core::RobotState& default_robot_state)
  : ompl::base::Constraint(num_dofs, num_cons)
  , state_storage_(robot_model)
  , joint_model_group_(robot_model->getJointModelGroup(group))
  , default_robot_state_(robot_model)
{
  default_robot_state_.setToDefaultValues();
  default_robot_state_ = default_robot_state;
}

Eigen::Isometry3d BaseConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  (*robot_state) = default_robot_state_;
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  robot_state->updateLinkTransforms();
  return robot_state->getGlobalLinkTransform(link_name_) * in_hand_pose_;
}

Eigen::MatrixXd BaseConstraint::robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  (*robot_state) = default_robot_state_;
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  Eigen::MatrixXd jacobian;
  // return value (success) not used, could return a garbage jacobian.
  robot_state->getJacobian(joint_model_group_, joint_model_group_->getLinkModel(link_name_),
                           Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);
  Eigen::Vector3d hand_to_object_in_world = robot_state->getGlobalLinkTransform(link_name_).rotation() * in_hand_pose_.translation();

  Eigen::MatrixXd translation_J(6,6);
  translation_J << 1, 0, 0, 0, hand_to_object_in_world(2), -hand_to_object_in_world(1),
                   0, 1, 0, -hand_to_object_in_world(2), 0, hand_to_object_in_world(0),
                   0, 0, 1, hand_to_object_in_world(1), -hand_to_object_in_world(0), 0,
                   0, 0, 0, 1, 0, 0,
                   0, 0, 0, 0, 1, 0,
                   0, 0, 0, 0, 0, 1;
  return translation_J * jacobian;
}

/******************************************
 * Position constraints
 * ****************************************/
BoxConstraint::BoxConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                             const unsigned int num_dofs, const moveit_msgs::Constraints& con, const moveit::core::RobotState& default_robot_state)
  : BaseConstraint(robot_model, group, num_dofs, 0, default_robot_state)
{
  tf2::fromMsg(con.in_hand_pose, in_hand_pose_);
  const moveit_msgs::PositionConstraint pos_con = con.position_constraints.at(0);
  constrained_dims_ = getConstrainedDims(pos_con);
  setManifoldDimension(num_dofs - constrained_dims_.size());
  bounds_ = createBoundVector(pos_con, constrained_dims_);

  // extract target position and orientation
  geometry_msgs::Point position = pos_con.constraint_region.primitive_poses.at(0).position;

  target_position_ << position.x, position.y, position.z;

  tf2::fromMsg(pos_con.constraint_region.primitive_poses.at(0).orientation, target_orientation_);

  link_name_ = pos_con.link_name;
}

void BoxConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                             Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::Vector3d error =
      target_orientation_.matrix().transpose() * (forwardKinematics(joint_values).translation() - target_position_);

  int emplace_index = 0;
  for (auto& dim : constrained_dims_)
  {
    out[emplace_index] = error[dim];
    emplace_index++;
  }
  out = bounds_.penalty(out);
}

void BoxConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                             Eigen::Ref<Eigen::MatrixXd> out) const
{
  Eigen::MatrixXd jac = target_orientation_.matrix().transpose() * robotGeometricJacobian(joint_values).topRows(3);

  int emplace_index = 0;
  for (auto& dim : constrained_dims_)
  {
    out.row(emplace_index) = jac.row(dim);
    emplace_index++;
  }
}

std::vector<std::size_t> BoxConstraint::getConstrainedDims(const moveit_msgs::PositionConstraint& pos_con) const
{
  std::vector<std::size_t> constrained_dims;
  const auto dims = pos_con.constraint_region.primitives.at(0).dimensions;
  for (std::size_t i = 0; i < dims.size(); ++i)
  {
    // if (dims.at(i) > 0 && dims.at(i) != std::numeric_limits<double>::infinity())
    // {
    //   constrained_dims.push_back(i);
    // }
    if (dims.at(i) > 0 && dims.at(i) < 10.0)
    {
      constrained_dims.push_back(i);
    }
  }
  return constrained_dims;
}

Bounds BoxConstraint::createBoundVector(const moveit_msgs::PositionConstraint& pos_con,
                                        const std::vector<std::size_t>& constrained_dims) const
{
  std::vector<double> lower;
  std::vector<double> upper;

  const auto dims = pos_con.constraint_region.primitives.at(0).dimensions;

  for (auto& i : constrained_dims)
  {
    if (dims.at(i) < getTolerance())
    {
      // RCLCPP_ERROR_STREAM(
      //     getLogger(),
      //     "Dimension: " << i
      //                   << " of position constraint is smaller than the tolerance used to evaluate the constraints. "
      //                      "This will make all states invalid and planning will fail. Please use a value between: "
      //                   << getTolerance() << " and " << EQUALITY_CONSTRAINT_THRESHOLD);
      ROS_WARN_NAMED(LOGNAME, "Something is wrong. Please check.");
    }
    
    double value = 0;
    if (dims.at(i) > EQUALITY_CONSTRAINT_THRESHOLD)
    {
      value = dims.at(i) / 2;
    }
    lower.push_back(-value);
    upper.push_back(value);
  }

  return { lower, upper };
}

/******************************************
 * Orientation constraints
 * ****************************************/
OrientationConstraint::OrientationConstraint(const moveit::core::RobotModelConstPtr& robot_model,
                                             const std::string& group, const unsigned int num_dofs,
                                             const moveit_msgs::Constraints& con, const moveit::core::RobotState& default_robot_state)
  : BaseConstraint(robot_model, group, num_dofs, 0, default_robot_state)
{
  tf2::fromMsg(con.in_hand_pose, in_hand_pose_);
  const moveit_msgs::OrientationConstraint ori_con = con.orientation_constraints.at(0);
  constrained_dims_ = getConstrainedDims(ori_con);

  setManifoldDimension(num_dofs - constrained_dims_.size());

  bounds_ = createBoundVector(ori_con, constrained_dims_);

  tf2::fromMsg(ori_con.orientation, target_orientation_);

  link_name_ = ori_con.link_name;
}

void OrientationConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(joint_values).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa(orientation_difference);
  Eigen::VectorXd error = aa.axis() * aa.angle();

  int emplace_index = 0;
  for (auto& dim : constrained_dims_)
  {
    out[emplace_index] = error[dim];
    emplace_index++;
  }
  out = bounds_.penalty(out);
}

void OrientationConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     Eigen::Ref<Eigen::MatrixXd> out) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(joint_values).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa{ orientation_difference };
  Eigen::MatrixXd jac =
      -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * robotGeometricJacobian(joint_values).bottomRows(3);

  int emplace_index = 0;
  for (auto& dim : constrained_dims_)
  {
    out.row(emplace_index) = jac.row(dim);
    emplace_index++;
  }
}

std::vector<std::size_t>
OrientationConstraint::getConstrainedDims(const moveit_msgs::OrientationConstraint& ori_con) const
{
  std::vector<std::size_t> constrained_dims;
  // If a tolerance is < 0 or infinity dont constrain it

  // if (ori_con.absolute_x_axis_tolerance > 0 && 
  //     ori_con.absolute_x_axis_tolerance != std::numeric_limits<double>::infinity())
  if (ori_con.absolute_x_axis_tolerance > 0 && 
      ori_con.absolute_x_axis_tolerance < 6.28)
  {
    constrained_dims.push_back(0);
  }
  // if (ori_con.absolute_x_axis_tolerance > 0 &&
  //     ori_con.absolute_y_axis_tolerance != std::numeric_limits<double>::infinity())
  if (ori_con.absolute_x_axis_tolerance > 0 &&
      ori_con.absolute_y_axis_tolerance < 6.28)
  {
    constrained_dims.push_back(1);
  }
  // if (ori_con.absolute_x_axis_tolerance > 0 &&
  //     ori_con.absolute_z_axis_tolerance != std::numeric_limits<double>::infinity())
  if (ori_con.absolute_x_axis_tolerance > 0 &&
      ori_con.absolute_z_axis_tolerance < 6.28)
  {
    constrained_dims.push_back(2);
  }

  return constrained_dims;
}

Bounds OrientationConstraint::createBoundVector(const moveit_msgs::OrientationConstraint& ori_con,
                                                const std::vector<std::size_t>& constrained_dims) const
{
  std::vector<double> lower;
  std::vector<double> upper;

  for (auto& dim : constrained_dims)
  {
    if (dim == 0)
    {
      lower.push_back(-ori_con.absolute_x_axis_tolerance);
      upper.push_back(ori_con.absolute_x_axis_tolerance);
    }
    else if (dim == 1)
    {
      lower.push_back(-ori_con.absolute_y_axis_tolerance);
      upper.push_back(ori_con.absolute_y_axis_tolerance);
    }
    else if (dim == 2)
    {
      lower.push_back(-ori_con.absolute_z_axis_tolerance);
      upper.push_back(ori_con.absolute_z_axis_tolerance);
    }
  }
  return { lower, upper };
}

/******************************************
 * OMPL Constraints Factory
 * ****************************************/
ompl::base::ConstraintPtr createOMPLConstraints(const moveit::core::RobotModelConstPtr& robot_model,
                                                const std::string& group,
                                                const moveit_msgs::Constraints& constraints,
                                                const planning_scene::PlanningSceneConstPtr& planning_scene)
{
  // This factory method contains template code to support position and/or orientation constraints.
  // If the specified constraints are invalid, a nullptr is returned.
  std::vector<ompl::base::ConstraintPtr> ompl_constraints;
  const std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();

  // Parse Position Constraints
  if (!constraints.position_constraints.empty())
  {
    if (constraints.position_constraints.size() > 1)
    {
      // RCLCPP_WARN(getLogger(), "Only a single position constraint is supported. Using the first one.");
      ROS_WARN_NAMED(LOGNAME, "Only a single position constraint is supported. Using the first one.");
    }

    const auto& primitives = constraints.position_constraints.at(0).constraint_region.primitives;
    if (primitives.size() > 1)
    {
      // RCLCPP_WARN(getLogger(), "Only a single position primitive is supported. Using the first one.");
      ROS_WARN_NAMED(LOGNAME, "Only a single position primitive is supported. Using the first one.");
    }
    if (primitives.empty() || primitives.at(0).type != shape_msgs::SolidPrimitive::BOX)
    {
      // RCLCPP_ERROR(getLogger(), "Unable to plan with the requested position constraint. "
      //                           "Only BOX primitive shapes are supported as constraint region.");
      ROS_WARN_NAMED(LOGNAME, "Unable to plan with the requested position constraint. Only BOX primitive shapes are supported as constraint region.");
    }
    else
    {
      BaseConstraintPtr pos_con =
          std::make_shared<BoxConstraint>(robot_model, group, num_dofs, constraints, planning_scene->getCurrentState());
      ompl_constraints.emplace_back(pos_con);
    }
  }

  // Parse Orientation Constraints
  if (!constraints.orientation_constraints.empty())
  {
    if (constraints.orientation_constraints.size() > 1)
    {
      // RCLCPP_WARN(getLogger(), "Only a single orientation constraint is supported. Using the first one.");
      ROS_WARN_NAMED(LOGNAME, "Only a single orientation constraint is supported. Using the first one.");
    }

    auto ori_con = std::make_shared<OrientationConstraint>(robot_model, group, num_dofs,
                                                           constraints, planning_scene->getCurrentState());
    ompl_constraints.emplace_back(ori_con);
  }

  // Check if we have any constraints to plan with
  if (ompl_constraints.empty())
  {
    // RCLCPP_ERROR(getLogger(), "Failed to parse any supported path constraints from planning request.");
    ROS_WARN_NAMED(LOGNAME, "Failed to parse any supported path constraints from planning request.");
    return nullptr;
  }

  return std::make_shared<ompl::base::ConstraintIntersection>(num_dofs, ompl_constraints);
}

ompl::base::ConstraintPtr createOMPLConstraints(const moveit::core::RobotModelConstPtr& robot_model,
                                                const std::string& group,
                                                const moveit_msgs::Constraints& constraints,
                                                const moveit_msgs::RobotState& default_state_msg)
{
  moveit::core::RobotState default_state(robot_model);
  moveit::core::robotStateMsgToRobotState(default_state_msg, default_state);

  // This factory method contains template code to support position and/or orientation constraints.
  // If the specified constraints are invalid, a nullptr is returned.
  std::vector<ompl::base::ConstraintPtr> ompl_constraints;
  const std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();

  // Parse Position Constraints
  if (!constraints.position_constraints.empty())
  {
    if (constraints.position_constraints.size() > 1)
    {
      // RCLCPP_WARN(getLogger(), "Only a single position constraint is supported. Using the first one.");
      ROS_WARN_NAMED(LOGNAME, "Only a single position constraint is supported. Using the first one.");
    }

    const auto& primitives = constraints.position_constraints.at(0).constraint_region.primitives;
    if (primitives.size() > 1)
    {
      // RCLCPP_WARN(getLogger(), "Only a single position primitive is supported. Using the first one.");
      ROS_WARN_NAMED(LOGNAME, "Only a single position constraint is supported. Using the first one.");
    }
    if (primitives.empty() || primitives.at(0).type != shape_msgs::SolidPrimitive::BOX)
    {
      // RCLCPP_ERROR(getLogger(), "Unable to plan with the requested position constraint. "
      //                           "Only BOX primitive shapes are supported as constraint region.");
      ROS_WARN_NAMED(LOGNAME, "Unable to plan with the requested position constraint. Only BOX primitive shapes are supported as constraint region.");
    }
    else
    {
      BaseConstraintPtr pos_con =
          std::make_shared<BoxConstraint>(robot_model, group, num_dofs, constraints, default_state);
      ompl_constraints.emplace_back(pos_con);
    }
  }

  // Parse Orientation Constraints
  if (!constraints.orientation_constraints.empty())
  {
    if (constraints.orientation_constraints.size() > 1)
    {
      // RCLCPP_WARN(getLogger(), "Only a single orientation constraint is supported. Using the first one.");
      ROS_WARN_NAMED(LOGNAME, "Only a single orientation constraint is supported. Using the first one.");
    }

    auto ori_con = std::make_shared<OrientationConstraint>(robot_model, group, num_dofs,
                                                           constraints, default_state);
    ompl_constraints.emplace_back(ori_con);
  }

  // Check if we have any constraints to plan with
  if (ompl_constraints.empty())
  {
    // RCLCPP_ERROR(getLogger(), "Failed to parse any supported path constraints from planning request.");
    ROS_WARN_NAMED(LOGNAME, "Failed to parse any supported path constraints from planning request.");
    return nullptr;
  }

  return std::make_shared<ompl::base::ConstraintIntersection>(num_dofs, ompl_constraints);
}
}  // namespace ompl_interface