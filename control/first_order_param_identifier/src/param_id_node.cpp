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
  : Node("first_order_param_identifier", node_options)
{

  using std::placeholders::_1;

  // Create Publishers
  pub_parameter_ =
    create_publisher<float64_t>("~/output/steering_time_constant", 1);

  // Create subscriptions
  sub_control_cmds_ =
    create_subscription<ControlCommand>("~/input/control_cmd", rclcpp::QoS{1},
                                        std::bind(
                                          &sys_id::ParameterIdentificationNode::onControlCommands,
                                          this, std::placeholders::_1));

  sub_current_steering_ptr_ =
    create_subscription<SteeringReport>("~/input/steering_state", rclcpp::QoS{1},
                                        std::bind(&sys_id::ParameterIdentificationNode::onCurrentSteering,
                                                  this, std::placeholders::_1));

  loadParams();

  initTimer(param_node_.sys_dt);

}

void ParameterIdentificationNode::initTimer(float64_t period_s)
{
  auto timer_callback = std::bind(&ParameterIdentificationNode::onTimer, this);

  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback) >>(this->get_clock(), period_ns,
                                                                             std::move(timer_callback),
                                                                             this->get_node_base_interface()->get_context());

  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void ParameterIdentificationNode::onTimer()
{
  ns_utils::print("OnTimer ()");

  publishParameter();

  if (!isDataReady())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Not enough data to start the identification process");

    return;
  }


  // Debug
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "[communication_delay] On Timer  ...");

}

void ParameterIdentificationNode::onControlCommands(const ControlCommand::SharedPtr msg)
{

  current_control_cmd_ptr_ = std::make_shared<ControlCommand>(*msg);

  // Debug
  // ns_utils::print("ACT On control method ");
  // ns_utils::print("Read parameter control period :", params_node_.cdob_ctrl_period);
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "onControlCommands");
  // end of debug
}

void ParameterIdentificationNode::onCurrentSteering(const SteeringReport::SharedPtr msg)
{

  current_steering_ptr_ = std::make_shared<SteeringReport>(*msg);

  // Debug
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "[communication_delay] On Steering  ...");
  // ns_utils::print("ACT On steering method ");
  // end of debug
}
void ParameterIdentificationNode::loadParams()
{

  // Read the filter orders.
  param_node_.sys_dt = declare_parameter<float64_t>("sys_dt");  // reads sec.
  param_node_.use_switching_sigma = declare_parameter<bool8_t>("robust_options.use_switching_sigma");  // reads sec.

}

bool8_t ParameterIdentificationNode::isDataReady()
{
  if (!current_steering_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(),
                                   *get_clock(),
                                   (1000ms).count(),
                                   "[first_order_param_identifier] Waiting for the steering measurement ...");
    return false;
  }

  if (!current_control_cmd_ptr_)
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(),
                                   *get_clock(),
                                   (1000ms).count(),
                                   "[first_order_param_identifier] Waiting for the control command ...");
    return false;
  }

  return true;
}

void ParameterIdentificationNode::publishParameter()
{
  pub_parameter_->publish(10.);
}

} // namespace sys_id

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sys_id::ParameterIdentificationNode)