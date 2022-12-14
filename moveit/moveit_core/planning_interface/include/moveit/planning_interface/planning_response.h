/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/MotionPlanDetailedResponse.h>
#include <sensor_msgs/JointState.h>

namespace planning_interface
{
struct MotionEdge
{
  sensor_msgs::JointState verified_vertex_1_;
  sensor_msgs::JointState verified_vertex_2_;
  int verified_vertex_id_1_;
  int verified_vertex_id_2_;
};

struct MotionPlanResponse
{
  MotionPlanResponse() : planning_time_(0.0)
  {
  }

  void getMessage(moveit_msgs::MotionPlanResponse& msg) const;

  robot_trajectory::RobotTrajectoryPtr trajectory_;
  double planning_time_;
  moveit_msgs::MoveItErrorCodes error_code_;

  // jiaming add the verified motion as the experience used later
  std::vector<MotionEdge> motion_edges;
  //std::vector<sensor_msgs::JointState> verified_vertex_1_;
  //std::vector<sensor_msgs::JointState> verified_vertex_2_;
  //std::vector<int> verified_vertex_id_1_;
  //std::vector<int> verified_vertex_id_2_;
};

struct MotionPlanDetailedResponse
{
  void getMessage(moveit_msgs::MotionPlanDetailedResponse& msg) const;

  std::vector<robot_trajectory::RobotTrajectoryPtr> trajectory_;
  std::vector<std::string> description_;
  std::vector<double> processing_time_;
  moveit_msgs::MoveItErrorCodes error_code_;
};

}  // namespace planning_interface
