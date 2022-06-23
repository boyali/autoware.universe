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

#include "communication_delay_compensator_node.hpp"

namespace observers
{
CommunicationDelayCompensatorNode::CommunicationDelayCompensatorNode(
  const rclcpp::NodeOptions & node_options)
: Node("communication_delay_compensator", node_options)
{
  using std::placeholders::_1;

  /* get parameter updates */
  readAndLoadParameters();

  params_node_.wheel_base = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo().wheel_base_m;
  initTimer(params_node_.cdob_ctrl_period);

  // Create Publishers
  pub_delay_compensator_ =
    create_publisher<DelayCompensatatorMsg>("~/output/communication_delay_compensation_refs", 1);

  pub_delay_compensator_debug_ = create_publisher<DelayCompensatorDebugMsg>(
    "~/output/communication_delay_compensation_debug", 1);

  // Create subscriptions
  sub_control_cmds_ = create_subscription<ControlCommand>(
    "~/input/control_cmd", rclcpp::QoS{1},
    std::bind(
      &observers::CommunicationDelayCompensatorNode::onControlCommands, this,
      std::placeholders::_1));

  sub_current_velocity_ptr_ = create_subscription<VelocityMsg>(
    "~/input/current_odometry", rclcpp::QoS{1},
    std::bind(
      &observers::CommunicationDelayCompensatorNode::onCurrentVelocity, this,
      std::placeholders::_1));

  sub_current_steering_ptr_ = create_subscription<SteeringReport>(
    "~/input/steering_state", rclcpp::QoS{1},
    std::bind(
      &observers::CommunicationDelayCompensatorNode::onCurrentSteering, this,
      std::placeholders::_1));

  sub_current_long_error_ptr_ = create_subscription<ControllerErrorReportMsg>(
    "~/input/long_errors", rclcpp::QoS{1},
    std::bind(
      &observers::CommunicationDelayCompensatorNode::onCurrentLongitudinalError, this,
      std::placeholders::_1));

  sub_current_lat_errors_ptr_ = create_subscription<ControllerErrorReportMsg>(
    "~/input/lat_errors", rclcpp::QoS{1},
    std::bind(
      &observers::CommunicationDelayCompensatorNode::onCurrentLateralErrors, this,
      std::placeholders::_1));

  //  sub_control_perf_errors_ptr_ = create_subscription<ErrorStampedControlPerfMsg>(
  //    "~/input/cp_errors", rclcpp::QoS{1},
  //    std::bind(
  //      &observers::CommunicationDelayCompensatorNode::onControlPerfErrors, this,
  //      std::placeholders::_1));

  // Dynamic Parameter Update.
  is_parameters_set_res_ = this->add_on_set_parameters_callback(
    std::bind(&CommunicationDelayCompensatorNode::onParameterUpdate, this, _1));

  // set the vehicle model.
  vehicle_model_ptr_ = std::make_shared<LinearKinematicErrorModel>(
    params_node_.wheel_base, params_node_.steering_tau, params_node_.cdob_ctrl_period);
}

void CommunicationDelayCompensatorNode::initTimer(float64_t period_s)
{
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<float64_t>(period_s));

  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&CommunicationDelayCompensatorNode::onTimer, this));
}

void CommunicationDelayCompensatorNode::onTimer()
{
  // Create compensator messages: For breaking cyclic dependency (controllers wait this package vice
  // versa.).
  DelayCompensatatorMsg compensation_msg{};
  current_delay_references_msg_ = std::make_shared<DelayCompensatatorMsg>(compensation_msg);

  DelayCompensatorDebugMsg compensation_debug_msg{};
  current_delay_debug_msg_ = std::make_shared<DelayCompensatorDebugMsg>(compensation_debug_msg);

  if (!isDataReady()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Not enough data to compute delay compensation");
    publishCompensationReferences();
    return;
  }

  if (!previous_control_cmd_ptr_) {
    ControlCommand zero_cmd{};
    previous_control_cmd_ptr_ = std::make_shared<ControlCommand>(zero_cmd);
    publishCompensationReferences();
  }

  // Update vehicle model.
  updateVehicleModel();
  // vehicle_model_ptr_->printDiscreteSystem();

  // Publish delay compensation reference.
  publishCompensationReferences();

  // Debug
  {
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Hello world");

    // RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Hello World!");
    // RCLCPP_ERROR(get_logger(), "Trajectory is invalid!, stop computing.");
    // RCLCPP_DEBUG(get_logger(), "MPC does not have a QP solver");
    //  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Timer");
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Control frequency  %4.2f ",
    //    params_node_.cdob_ctrl_period);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter ey order %4.2i ",
    //    params_node_.qfilter_lateral_error_order);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter eyaw order %4.2i ",
    //    params_node_.qfilter_heading_error_order);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter steering order %4.2i ",
    //    params_node_.qfilter_steering_order);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter velocity order %4.2i ",
    //    params_node_.qfilter_velocity_error_order);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter ey frq %4.2f ",
    //    params_node_.qfilter_lateral_error_freq);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter eyaw frq %4.2f ",
    //    params_node_.qfilter_heading_error_freq);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter steering frq %4.2f ",
    //    params_node_.qfilter_steering_freq);
    //
    //  RCLCPP_INFO_THROTTLE(
    //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter velocity frq %4.2f ",
    //    params_node_.qfilter_velocity_error_freq);

    //    if (delay_comp_steering_ptr_) {
    //      delay_comp_steering_ptr_->print();
    //    } else {
    //      ns_utils::print("Unique pointer is not set ");
    //    }
  }
}

void CommunicationDelayCompensatorNode::onControlCommands(const ControlCommand::SharedPtr msg)
{
  previous_control_cmd_ptr_ = current_control_cmd_ptr_;
  current_control_cmd_ptr_ = std::make_shared<ControlCommand>(*msg);

  // Debug
  // ns_utils::print("ACT On control method ");
  // ns_utils::print("Read parameter control period :", params_node_.cdob_ctrl_period);
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "onControlCommands");
  // end of debug
}

void CommunicationDelayCompensatorNode::onCurrentVelocity(const VelocityMsg::SharedPtr msg)
{
  if (current_velocity_ptr) {
    previous_velocity_ = current_velocity_ptr->twist.twist.linear.x;
  }

  current_velocity_ptr = std::make_shared<VelocityMsg>(*msg);

  // ns_utils::print("ACT On velocity method ");
  // ns_utils::print("Read parameter control period :", params_node_.cdob_ctrl_period);
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Velocity");
}

void CommunicationDelayCompensatorNode::onCurrentLongitudinalError(
  ControllerErrorReportMsg::SharedPtr const msg)
{
  prev_longitudinal_errors_ = current_longitudinal_errors_;
  current_longitudinal_errors_ = std::make_shared<ControllerErrorReportMsg>(*msg);

  if (prev_longitudinal_errors_) {
    // Compute current steering error.
    previous_target_velocity_ = prev_longitudinal_errors_->target_velocity_read;
  }

  // Debug
  // auto vel_error = static_cast<double>(current_longitudinal_errors_->velocity_error_read);
  // ns_utils::print("Longitudinal velocity error :", vel_error);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Longitudinal Error");
  // end of debug
}

void CommunicationDelayCompensatorNode::onCurrentLateralErrors(
  ControllerErrorReportMsg::SharedPtr const msg)
{
  prev_lateral_errors_ = current_lateral_errors_;
  current_lateral_errors_ = std::make_shared<ControllerErrorReportMsg>(*msg);

  // Compute current steering error.
  current_curvature_ = current_lateral_errors_->curvature_read;

  // Ackerman Ideal Steering
  current_ideal_steering_ = std::atan(current_curvature_ * params_node_.wheel_base);

  if (prev_lateral_errors_) {
    // Compute current steering error.
    prev_curvature_ = prev_lateral_errors_->curvature_read;

    // Ackerman Ideal Steering
    prev_ideal_steering_ = std::atan(prev_curvature_ * params_node_.wheel_base);
  }

  // Debug
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Lateral Errors");

  // 			if (current_lateral_errors_)
  // 			{
  // 				auto lat_error =
  // static_cast<double>(current_lateral_errors_->lateral_deviation_read);
  // auto heading_error = static_cast<double>(current_lateral_errors_->heading_angle_error_read);
  // 				ns_utils::print("Current lateral errors : ", lat_error,
  // heading_error);
  // 			}
  // end of debug.
}

// void CommunicationDelayCompensatorNode::onControlPerfErrors(
//   const ErrorStampedControlPerfMsg::SharedPtr msg)
//{
//   current_cont_perf_errors_ = std::make_shared<ErrorStampedControlPerfMsg>(*msg);
//
//   // Debug
//   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Control Perf. Errors");
// }

void CommunicationDelayCompensatorNode::publishCompensationReferences()
{
  current_delay_references_msg_->stamp = this->now();
  current_delay_debug_msg_->stamp = this->now();

  // new_msg.lateral_deviation_error_compensation_ref = 1.0;
  pub_delay_compensator_->publish(*current_delay_references_msg_);
  pub_delay_compensator_debug_->publish(*current_delay_debug_msg_);
}

void CommunicationDelayCompensatorNode::onCurrentSteering(const SteeringReport::SharedPtr msg)
{
  prev_steering_ptr_ = current_steering_ptr_;
  current_steering_ptr_ = std::make_shared<SteeringReport>(*msg);

  if (prev_steering_ptr_) {
    previous_steering_angle_ = prev_steering_ptr_->steering_tire_angle;
  }

  // Debug
  RCLCPP_WARN_SKIPFIRST_THROTTLE(
    get_logger(), *get_clock(), (1000ms).count(), "[communication_delay] On Steering  ...");
  // ns_utils::print("ACT On steering method ");
  // end of debug
}

bool8_t CommunicationDelayCompensatorNode::isDataReady()
{
  if (!current_velocity_ptr) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[communication_delay] Waiting for the velocity measurement ...");
    return false;
  }

  if (!current_steering_ptr_) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[communication_delay] Waiting for the steering measurement ...");
    return false;
  }

  if (!current_control_cmd_ptr_) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[communication_delay] Waiting for the control command ...");
    return false;
  }

  if (!current_lateral_errors_) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[communication_delay] Waiting for the current lateral error report ...");
    return false;
  }

  if (!current_longitudinal_errors_) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[communication_delay] Waiting for the current longitudinal error report ...");
    return false;
  }

  return true;
}

void CommunicationDelayCompensatorNode::readAndLoadParameters()
{
  try {
    // Read the filter orders.
    params_node_.cdob_ctrl_period = declare_parameter<float64_t>("cdob_ctrl_period");  // reads sec.
    params_node_.qfilter_lateral_error_order =
      declare_parameter<int>("qfilter_lateral_error_order");
    params_node_.qfilter_heading_error_order =
      declare_parameter<int>("qfilter_heading_error_order");
    params_node_.qfilter_steering_order = declare_parameter<int>("qfilter_steering_order");
    params_node_.qfilter_velocity_error_order =
      declare_parameter<int>("qfilter_velocity_error_order");
    params_node_.qfilter_acc_error_order = declare_parameter<int>("qfilter_acc_error_order");

    // Read the filter cut-oof frequencies.
    params_node_.qfilter_lateral_error_freq =
      declare_parameter<float64_t>("qfilter_lateral_error_freq");
    params_node_.qfilter_heading_error_freq =
      declare_parameter<float64_t>("qfilter_heading_error_freq");
    params_node_.qfilter_steering_freq = declare_parameter<float64_t>("qfilter_steering_freq");
    params_node_.qfilter_velocity_error_freq =
      declare_parameter<float64_t>("qfilter_velocity_error_freq");
    params_node_.qfilter_acc_error_freq = declare_parameter<float64_t>("qfilter_acc_error_freq");

    // First order state dynamics parameters.
    params_node_.steering_tau = declare_parameter<float64_t>("steering_time_constant_");
    params_node_.velocity_tau = declare_parameter<float64_t>("velocity_time_constant_");
    params_node_.acc_tau = declare_parameter<float64_t>("acc_time_constant_");

  }

  catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
  }
}

rcl_interfaces::msg::SetParametersResult CommunicationDelayCompensatorNode::onParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param(parameters, "cdob_ctrl_period", params_node_.cdob_ctrl_period);

    update_param(
      parameters, "qfilter_lateral_error_order", params_node_.qfilter_lateral_error_order);
    update_param(
      parameters, "qfilter_heading_error_order", params_node_.qfilter_heading_error_order);
    update_param(parameters, "qfilter_steering_order", params_node_.qfilter_steering_order);
    update_param(
      parameters, "qfilter_velocity_error_order", params_node_.qfilter_velocity_error_order);
    update_param(parameters, "qfilter_acc_error_order", params_node_.qfilter_acc_error_order);

    update_param(parameters, "qfilter_lateral_error_freq", params_node_.qfilter_lateral_error_freq);
    update_param(parameters, "qfilter_heading_error_freq", params_node_.qfilter_heading_error_freq);
    update_param(parameters, "qfilter_steering_freq", params_node_.qfilter_steering_freq);
    update_param(
      parameters, "qfilter_velocity_error_freq", params_node_.qfilter_velocity_error_freq);
    update_param(parameters, "qfilter_acc_error_freq", params_node_.qfilter_acc_error_freq);

    update_param(parameters, "steering_time_constant_", params_node_.steering_tau);
    update_param(parameters, "velocity_time_constant_", params_node_.velocity_tau);
    update_param(parameters, "acc_time_constant_", params_node_.acc_tau);

  }

  // transaction succeeds, now assign values
  catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  for (const auto & param : parameters) {
    RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
  }

  return result;
}

bool8_t CommunicationDelayCompensatorNode::isVehicleStopping()
{
  auto current_vel = current_velocity_ptr->twist.twist.linear.x;
  return std::fabs(current_vel) <= 0.5;
}
void CommunicationDelayCompensatorNode::updateVehicleModel()
{
  // auto vref = current_velocity_ptr->twist.twist.linear.x;
  // auto & u_prev = previous_control_cmd_ptr_->lateral.steering_tire_angle;

  // Update the matrices
  vehicle_model_ptr_->updateStateSpace(previous_target_velocity_, current_ideal_steering_);

  // Update the initial state.
  //  auto current_steering = current_steering_ptr_->steering_tire_angle;
  //  float64_t ey{current_lateral_errors_->lateral_deviation_read};
  //  float64_t eyaw{current_lateral_errors_->heading_angle_error_read};

  //  if (!vehicle_model_ptr_->areInitialStatesSet()) {
  //    vehicle_model_ptr_->updateInitialStates(ey, eyaw, current_steering);
  //  }

  if (prev_lateral_errors_ && prev_steering_ptr_ && prev_longitudinal_errors_) {
    float64_t ey{prev_lateral_errors_->lateral_deviation_read};
    float64_t eyaw{prev_lateral_errors_->heading_angle_error_read};

    vehicle_model_ptr_->updateInitialStates(ey, eyaw, previous_steering_angle_);
  }

  //  ns_utils::print(
  //    "vref, delta, ey, eyaw, ackerman", vref, current_steering, ey, eyaw,
  //    ideal_ackerman_steering);
}

}  // namespace observers
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(observers::CommunicationDelayCompensatorNode)