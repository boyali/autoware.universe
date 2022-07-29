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
  pub_parameter_ = create_publisher<sysIDmsg>("~/output/steering_time_constant", 1);

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

  param_id_core_ = std::make_unique<ParamIDCore>(params_node_);

  // Create a timer to publish the parameter
  initTimer(params_node_.sys_dt);

  // Debug


}

void ParameterIdentificationNode::initTimer(float64_t period_s)
{
  auto timer_callback = std::bind(&ParameterIdentificationNode::onTimer, this);

  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback) >>
    (this->get_clock(), period_ns,
     std::move(timer_callback),
     this->get_node_base_interface()->get_context());

  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void ParameterIdentificationNode::onTimer()
{

  if (!isDataReady())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Not enough data to start the identification process");

    return;
  }

  auto const &steering_measured = current_steering_ptr_->steering_tire_angle;
  auto const &steering_command = current_control_cmd_ptr_->lateral.steering_tire_angle;



  /**
   * Get steering time-derivative estimate.
   * */
  auto const &steering_filtered = param_id_core_->trackingDifferentiator(steering_tracking_differentiator_x_,
                                                                         static_cast<float64_t>(steering_command));

  /**
   * Manage steering queue
   * */

  steering_frquency_history_.pop_front();
  steering_frquency_history_.emplace_back(steering_filtered(1));

  auto const &l1_norm = std::accumulate(steering_frquency_history_.begin(), steering_frquency_history_.end(), 0.0,
                                        [](auto total, auto xitem)
                                        { return total + std::fabs(xitem); });

  ns_utils::print("L1 norm: ", l1_norm);

  if (l1_norm > 0.1)
  {
    param_id_core_->updateParameterEstimate(static_cast< float64_t>(steering_measured),
                                            static_cast<float64_t>(steering_command), current_param_estimate_ab_);
  }

  sysIDmsg current_param_estimate_msg;
  current_param_estimate_msg.a = current_param_estimate_ab_[0];
  current_param_estimate_msg.b = current_param_estimate_ab_[1];
  current_param_estimate_msg.xtracked = steering_filtered(0);
  current_param_estimate_msg.xtracked_dot = steering_filtered(1);

  publishParameter(current_param_estimate_msg);

  // Debug
  RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(), "[Sys Id] On Timer  ...");

  //  ns_utils::print("params sys_dt:", params_node_.sys_dt);
  //  ns_utils::print("Read params lower bound :", params_node_.a_lower_bound);
  // param_id_core_->printModels();

}

void ParameterIdentificationNode::onControlCommands(const ControlCommand::SharedPtr msg)
{
  if (current_control_cmd_ptr_)
  {
    prev_control_cmd_ptr_ = current_control_cmd_ptr_;
  }

  current_control_cmd_ptr_ = std::make_shared<ControlCommand>(*msg);

  // Debug
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "onControlCommands");
  // end of debug
}

void ParameterIdentificationNode::onCurrentSteering(const SteeringReport::SharedPtr msg)
{

  current_steering_ptr_ = std::make_shared<SteeringReport>(*msg);

  // Debug
  RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(),
                                 *get_clock(),
                                 (1000ms).count(),
                                 "[communication_delay] On Steering  ...");

  // end of debug
}
void ParameterIdentificationNode::loadParams()
{

  // Read the filter orders.
  params_node_.sys_dt = declare_parameter<float64_t>("sys_dt");  // reads sec.

  params_node_.use_switching_sigma = declare_parameter<bool8_t>("robust_options.use_switching_sigma");
  params_node_.use_deadzone = declare_parameter<bool8_t>("robust_options.use_deadzone");
  params_node_.use_dynamic_normalization = declare_parameter<bool8_t>("robust_options.use_dynamic_normalization");

  params_node_.sigma_0 = declare_parameter<float64_t>("robust_options.sigma_0");
  params_node_.deadzone_threshold = declare_parameter<float64_t>("robust_options.deadzone_threshold");
  params_node_.delta0_norm_ = declare_parameter<float64_t>("robust_options.delta0_norm_");

  params_node_.tracking_tau = declare_parameter<float64_t>("robust_options.tracking_tau");
  params_node_.am_stabilizing = declare_parameter<float64_t>("robust_options.am_stabilizing");

  params_node_.smoother_eps = declare_parameter<float64_t>("projection_options.smoother_eps");
  params_node_.forgetting_factor = declare_parameter<float64_t>("projection_options.forgetting_factor");


  // Since we identify the 1/tau, min max order changes.
  params_node_.a_lower_bound = 1. / declare_parameter<float64_t>("parameter_vars.a_upper_bound");
  params_node_.a_upper_bound = 1. / declare_parameter<float64_t>("parameter_vars.a_lower_bound");

  params_node_.b_lower_bound = 1. / declare_parameter<float64_t>("parameter_vars.b_upper_bound");
  params_node_.b_upper_bound = 1. / declare_parameter<float64_t>("parameter_vars.b_lower_bound");

  params_node_.param_normalized_upper_bound =
    declare_parameter<float64_t>("parameter_vars.param_normalized_upper_bound");

}

bool8_t ParameterIdentificationNode::isDataReady()
{
  if (!current_steering_ptr_ && !prev_control_cmd_ptr_)
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
                                   *get_clock(), (1000ms).count(),
                                   "[first_order_param_identifier] Waiting for the control command ...");
    return false;
  }

  return true;
}

void ParameterIdentificationNode::publishParameter(sysIDmsg const &msg) const
{

  pub_parameter_->publish(msg);
}

} // namespace sys_id

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sys_id::ParameterIdentificationNode)