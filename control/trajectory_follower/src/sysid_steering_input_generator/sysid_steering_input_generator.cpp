// Copyright 2022 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_follower/sysid_steering_input_generator/sysid_steering_input_generator.hpp"

namespace autoware::motion::control::trajectory_follower
{

SysIDLateralController::SysIDLateralController(rclcpp::Node &node) : node_{&node}
{

  using std::placeholders::_1;
  loadParams();

}

SysIDLateralController::~SysIDLateralController() = default;

void SysIDLateralController::setInputData(InputData const &input_data)
{
  m_current_velocity_ptr_ = input_data.current_odometry_ptr;
  m_current_steering_ptr_ = input_data.current_steering_ptr;
  m_current_trajectory_ptr_ = input_data.current_trajectory_ptr;
}

boost::optional<LateralOutput> SysIDLateralController::run()
{

  if (!checkData())
  {
    return boost::none;
  }

  autoware_auto_control_msgs::msg::AckermannLateralCommand ctrl_cmd;
  ctrl_cmd.steering_tire_angle = 0.;
  ctrl_cmd.steering_tire_rotation_rate = 0.;

  const auto createLateralOutput = [this](const auto &cmd)
  {
    LateralOutput output;
    output.control_cmd = createCtrlCmdMsg(cmd);
    output.sync_data.is_steer_converged = true; // isSteerConverged(cmd);
    return boost::optional<LateralOutput>(output);
  };

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 5000 /*ms*/, "In SYSID run ....");

  auto stream = ns_utils::print_stream("min_speed ", min_speed_);

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000 /*ms*/, "\n  %s", stream.str().c_str());

  stream = ns_utils::print_stream("\n max_speed ", max_speed_);

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000 /*ms*/, "\n  %s", stream.str().c_str());

  return createLateralOutput(ctrl_cmd);

}
autoware_auto_control_msgs::msg::AckermannLateralCommand SysIDLateralController::createCtrlCmdMsg(
  autoware_auto_control_msgs::msg::AckermannLateralCommand ctrl_cmd)
{
  ctrl_cmd.stamp = node_->now();
  // m_steer_cmd_prev = ctrl_cmd.steering_tire_angle;

  return ctrl_cmd;
}
void SysIDLateralController::loadParams()
{
  dummy_param_ = node_->declare_parameter<double>("dummy_param", 0.);
  min_speed_ = node_->declare_parameter<double>("common_variables.min_speed", 0.);
  max_speed_ = node_->declare_parameter<double>("common_variables.max_speed", 0.);
}

bool SysIDLateralController::checkData() const
{

  if (!m_current_velocity_ptr_)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for the  current_velocity = %d",
                 m_current_velocity_ptr_ != nullptr);
    return false;
  }

  if (!m_current_steering_ptr_)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "Waiting for the current_steering = %d", m_current_steering_ptr_ != nullptr);
    return false;
  }

  if (!m_current_trajectory_ptr_)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), " Waiting for the current trajectory = %d", m_current_trajectory_ptr_ != nullptr);
    return false;
  }

  return true;
}

} // namespace autoware::motion::control::trajectory_follower
