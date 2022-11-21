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

  // Define input type
  int input_type_id = node_->declare_parameter<int>("default_input_class", 0);
  auto input_type = getInputType(input_type_id);
  loadParams(input_type);

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

  if (!checkData() || !updateCurrentPose())
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


  // DEBUG
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 5000 /*ms*/, "In SYSID run ....");

  auto stream = ns_utils::print_stream("min_speed ", common_input_lib_params_.minimum_speed);

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000 /*ms*/, "\n  %s", stream.str().c_str());

  stream = ns_utils::print_stream("\n max_speed ", common_input_lib_params_.maximum_speed);

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000 /*ms*/, "\n  %s", stream.str().c_str());

  stream = ns_utils::print_stream("\n signal magnitude ", common_input_lib_params_.maximum_amplitude);

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000 /*ms*/, "\n  %s", stream.str().c_str());

  stream = ns_utils::print_stream("\n time starts ", common_input_lib_params_.tstart);

  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000 /*ms*/, "\n  %s", stream.str().c_str());

  return createLateralOutput(ctrl_cmd);

}
autoware_auto_control_msgs::msg::AckermannLateralCommand SysIDLateralController::createCtrlCmdMsg(
  autoware_auto_control_msgs::msg::AckermannLateralCommand ctrl_cmd)
{
  // Generate input from input classes.
  auto const &vx = m_current_velocity_ptr_->twist.twist.linear.x;
  auto const &sysid_input_val = input_wrapper_.generateInput(vx);

  // Put in the command message.
  ctrl_cmd.steering_tire_angle = sysid_input_val;
  ctrl_cmd.stamp = node_->now();
  // m_steer_cmd_prev = ctrl_cmd.steering_tire_angle;

  return ctrl_cmd;
}

InputType SysIDLateralController::getInputType(int const &input_id)
{
  if (input_id == 0)
  { return InputType::IDENTITY; }

  if (input_id == 1)
  { return InputType::STEP; }

  if (input_id == 2)
  { return InputType::PRBS; }

  if (input_id == 3)
  { return InputType::FWNOISE; }

  if (input_id == 4)
  { return InputType::SUMSINs; }

  return InputType::IDENTITY;
}

void SysIDLateralController::loadParams(InputType const &input_type)
{

  // Load common parameters.
  common_input_lib_params_.maximum_amplitude =
    node_->declare_parameter<double>("common_variables.signal_magnitude", 0.);

  common_input_lib_params_.minimum_speed = node_->declare_parameter<double>("common_variables.min_speed", 0.);
  common_input_lib_params_.maximum_speed = node_->declare_parameter<double>("common_variables.max_speed", 0.);
  common_input_lib_params_.tstart = node_->declare_parameter<double>("common_variables.time_start_after", 0.);

  if (input_type == InputType::STEP)
  {
    sysid::sStepParameters step_params;
    step_params.start_time = common_input_lib_params_.tstart;
    step_params.max_amplitude = common_input_lib_params_.maximum_amplitude;

    step_params.step_period = node_->declare_parameter<double>("step_input_params.step_period", 1.);
    step_params.step_direction_flag = node_->declare_parameter<int8_t>("step_input_params.step_direction", 0);

    sysid::InpStepUpDown step_up_down(common_input_lib_params_.minimum_speed,
                                      common_input_lib_params_.maximum_speed,
                                      step_params);
    // Change the input wrapper class.
    input_wrapper_ = sysid::InputWrapper{step_up_down};
  }
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

bool SysIDLateralController::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform =
      m_tf_buffer_.lookupTransform(m_current_trajectory_ptr_->header.frame_id, "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000 /*ms*/, "%s", ex.what());
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000 /*ms*/,
                                   "%s", m_tf_buffer_.allFramesAsString().c_str());
    return false;
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  m_current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);

  return true;
}

} // namespace autoware::motion::control::trajectory_follower
