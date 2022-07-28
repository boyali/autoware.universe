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

#include "param_id_node.hpp"

namespace sys_id
{

ParameterIdentificationNode::ParameterIdentificationNode(const rclcpp::NodeOptions &node_options)
  : Node("parameter_identification_node", node_options)
{

  using std::placeholders::_1;

  // Create Publishers
  pub_rameter_ =
    create_publisher<float64_t>("~/output/steering_time_constant", 1);


  // Create subscriptions
  sub_control_cmds_ =
    create_subscription<ControlCommand>("~/input/control_cmd", rclcpp::QoS{1},
                                        std::bind(
                                          &sys_id::ParameterIdentificationNode::onControlCommands,
                                          this, std::placeholders::_1));

  sub_current_steering_ptr_ =
    create_subscription<SteeringReport>("~/input/steering_state", rclcpp::QoS{1},
                                        std::bind(
                                          &sys_id::ParameterIdentificationNode::onCurrentSteering,
                                          this, std::placeholders::_1));

}

void ParameterIdentificationNode::initTimer(float64_t period_s)
{
  const auto
    period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<float64_t>(period_s));

  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns,
                         std::bind(&ParameterIdentificationNode::onTimer, this));
}

void ParameterIdentificationNode::onTimer()
{

}

void ParameterIdentificationNode::onControlCommands(const ControlCommand::SharedPtr msg)
{
//  previous_control_cmd_ptr_ = current_control_cmd_ptr_;
//  current_control_cmd_ptr_ = std::make_shared<ControlCommand>(*msg);

  // Debug
  // ns_utils::print("ACT On control method ");
  // ns_utils::print("Read parameter control period :", params_node_.cdob_ctrl_period);
  // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "onControlCommands");
  // end of debug
}

void ParameterIdentificationNode::onCurrentSteering(const SteeringReport::SharedPtr msg)
{
//  prev_steering_ptr_ = current_steering_ptr_;
//  current_steering_ptr_ = std::make_shared<SteeringReport>(*msg);

  // Debug
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "[communication_delay] On Steering  ...");
  // ns_utils::print("ACT On steering method ");
  // end of debug
}
void ParameterIdentificationNode::loadParams()
{

}

} // namespace sys_id

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sys_id::ParameterIdentificationNode)