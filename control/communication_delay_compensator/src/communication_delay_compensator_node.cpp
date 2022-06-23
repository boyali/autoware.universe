
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

  setLateralCDOB();
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
  current_delay_ref_msg_ptr_ = std::make_shared<DelayCompensatatorMsg>(compensation_msg);

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
  vehicle_model_ptr_->printDiscreteSystem();

  // Compute lateral CDOB references.
  if (!isVehicleStopping()) {
    computeLateralCDOB();
  }

  // Publish delay compensation reference.
  publishCompensationReferences();

  // Debug
  {
    // cdob_lateral_ptr_->printQfilterTFs();
    // cdob_lateral_ptr_->printQfilterSSs();
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
  prev_long_errors_ptr_ = current_long_errors_ptr_;
  current_long_errors_ptr_ = std::make_shared<ControllerErrorReportMsg>(*msg);

  if (prev_long_errors_ptr_) {
    // Compute current steering error.
    previous_target_velocity_ = prev_long_errors_ptr_->target_velocity_read;
  }

  // Debug
  // auto vel_error = static_cast<double>(current_long_errors_ptr_->velocity_error_read);
  // ns_utils::print("Longitudinal velocity error :", vel_error);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Longitudinal Error");
  // end of debug
}

void CommunicationDelayCompensatorNode::onCurrentLateralErrors(
  ControllerErrorReportMsg::SharedPtr const msg)
{
  prev_lat_errors_ptr_ = current_lat_errors_ptr_;
  current_lat_errors_ptr_ = std::make_shared<ControllerErrorReportMsg>(*msg);

  // Compute current steering error.
  current_curvature_ = current_lat_errors_ptr_->curvature_read;

  // Ackerman Ideal Steering
  current_ideal_steering_ = std::atan(current_curvature_ * params_node_.wheel_base);

  if (prev_lat_errors_ptr_) {
    // Compute current steering error.
    prev_curvature_ = prev_lat_errors_ptr_->curvature_read;

    // Ackerman Ideal Steering
    prev_ideal_steering_ = std::atan(prev_curvature_ * params_node_.wheel_base);
  }

  // Debug
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Lateral Errors");

  // 			if (current_lat_errors_ptr_)
  // 			{
  // 				auto lat_error =
  // static_cast<double>(current_lat_errors_ptr_->lateral_deviation_read);
  // auto heading_error =
  // static_cast<double>(current_lat_errors_ptr_->heading_angle_error_read);
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
  current_delay_ref_msg_ptr_->stamp = this->now();
  current_delay_debug_msg_->stamp = this->now();

  // new_msg.lateral_deviation_error_compensation_ref = 1.0;
  pub_delay_compensator_->publish(*current_delay_ref_msg_ptr_);
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

  if (!current_lat_errors_ptr_) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(),
      "[communication_delay] Waiting for the current lateral error report ...");
    return false;
  }

  if (!current_long_errors_ptr_) {
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
  //  float64_t ey{current_lat_errors_ptr_->lateral_deviation_read};
  //  float64_t eyaw{current_lat_errors_ptr_->heading_angle_error_read};

  //  if (!vehicle_model_ptr_->areInitialStatesSet()) {
  //    vehicle_model_ptr_->updateInitialStates(ey, eyaw, current_steering);
  //  }

  if (prev_lat_errors_ptr_ && prev_steering_ptr_ && prev_long_errors_ptr_) {
    float64_t ey{prev_lat_errors_ptr_->lateral_deviation_read};
    float64_t eyaw{prev_lat_errors_ptr_->heading_angle_error_read};

    vehicle_model_ptr_->updateInitialStates(ey, eyaw, previous_steering_angle_);
  }

  // ns_utils::print(" Previous target speed :", previous_target_velocity_);
}
void CommunicationDelayCompensatorNode::setLateralCDOB()
{
  /**
   * Create qfilters for each states. These qfilters take separately the same input and forward
   * it to the vehicle model by filtering the input commands.
   */

  // --------------- Qfilter Construction for steering state ------------------------------
  // Create a qfilter from the given order for the steering system.
  auto const & order_steering = params_node_.qfilter_steering_order;
  auto const & wc_steering = params_node_.qfilter_steering_freq;  // cut-off frq Hz.

  // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
  auto qfilter_steering = get_nthOrderTF(wc_steering, order_steering);

  //  float64_t damping_val{1.};
  //  auto remaining_order = order_of_q - 2;
  //  auto q_tf = get_nthOrderTFwithDampedPoles(cut_off_frq_in_hz_q, damping_val, remaining_order);

  // --------------- Qfilter Construction for heading error state -------------------------
  auto const & order_heading_error = params_node_.qfilter_heading_error_order;
  auto const & wc_heading_error = params_node_.qfilter_heading_error_order;

  // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
  auto qfilter_heading_error = get_nthOrderTF(wc_heading_error, order_heading_error);

  //  float64_t damping_val{1.};
  //  auto remaining_order = order_of_q - 2;
  //  auto q_tf = get_nthOrderTFwithDampedPoles(cut_off_frq_in_hz_q, damping_val, remaining_order);

  // --------------- Qfilter Construction for lateral error state -------------------------
  auto const & order_lat_error = params_node_.qfilter_lateral_error_order;
  auto const & wc_lat_error = params_node_.qfilter_lateral_error_freq;

  // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
  auto qfilter_lat_error = get_nthOrderTF(wc_lat_error, order_lat_error);

  //  float64_t damping_val{1.};
  //  auto remaining_order = order_of_q - 2;
  //  auto q_tf = get_nthOrderTFwithDampedPoles(cut_off_frq_in_hz_q, damping_val, remaining_order);

  CommunicationDelayCompensatorForward delay_compensator_forward(
    vehicle_model_ptr_, qfilter_lat_error, qfilter_heading_error, qfilter_steering,
    params_node_.cdob_ctrl_period);

  cdob_lateral_ptr_ =
    std::make_unique<CommunicationDelayCompensatorForward>(delay_compensator_forward);
}
void CommunicationDelayCompensatorNode::computeLateralCDOB()
{
  // get the current outputs observed y=[ey, eyaw, steering] for qfilters.
  auto const & current_lat_error = current_lat_errors_ptr_->lateral_deviation_read;
  auto const & current_heading_error = current_lat_errors_ptr_->lateral_deviation_read;
  auto const & current_steering = current_steering_ptr_->steering_tire_angle;

  current_lat_measurements_ << current_lat_error, current_heading_error, current_steering;

  // get the previous inputs and parameter that are used and sent to the vehicle.
  /**
   * previous: ideal steering and target velocities to linearize the model, and previous
   * curvature as an input to the steering.
   * */

  auto const & prev_steering_control_cmd = previous_control_cmd_ptr_->lateral.steering_tire_angle;
  previous_inputs_to_cdob_ << prev_steering_control_cmd, prev_curvature_;

  cdob_lateral_ptr_->simulateOneStep(
    current_lat_measurements_, previous_inputs_to_cdob_, current_delay_ref_msg_ptr_,
    current_delay_debug_msg_);

  // Set messages
  current_delay_ref_msg_ptr_->lateral_deviation_read = current_lat_error;
  current_delay_ref_msg_ptr_->heading_angle_error_read = current_heading_error;
  current_delay_ref_msg_ptr_->steering_read = current_steering;

  // DEBUG
  {
    //    ns_utils::print(
    //      "Current readings : ", current_lat_error, current_heading_error, current_steering);
    //
    //    ns_eigen_utils::printEigenMat(Eigen::MatrixXd(current_lat_measurements_));
    //
    //    ns_utils::print("Previous inputs to CDOB : ");
    //    ns_eigen_utils::printEigenMat(previous_inputs_to_cdob_);

    // get the vehicle model parameters on which the controllers compute the control signals.
    // previous : curvature, previous_target velocity
  }
}

}  // namespace observers
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(observers::CommunicationDelayCompensatorNode)