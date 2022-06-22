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

#ifndef COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP

// Base Headers

#include "eigen3/Eigen/Core"

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

// Autoware Headers
#include "common/types.hpp"
#include "control_performance_analysis/msg/error_stamped.hpp"
#include "vehicle_models/vehicle_kinematic_error_model.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/controller_error_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/delay_compensation_debug.hpp"
#include "autoware_auto_vehicle_msgs/msg/delay_compensation_refs.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tier4_vehicle_msgs/msg/delay_compensators_stamped.hpp"

// LIBRARY HEADERS
// #include "autoware_control_toolbox.hpp"
// #include "utils_delay_observer/delay_compensation_utils.hpp"
// #include "qfilters.hpp"
#include "communication_delay_compensator_core.hpp"
#include "node_denifitions/node_definitions.hpp"

// ROS headers
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace observers
{
using namespace std::chrono_literals;
using ControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
// using DelayCompensatatorMsg = tier4_vehicle_msgs::msg::DelayCompensators;
using DelayCompensatatorMsg = autoware_auto_vehicle_msgs::msg::DelayCompensationRefs;
using DelayCompensatorDebugMsg = autoware_auto_vehicle_msgs::msg::DelayCompensationDebug;

/**
 * @brief longitudinal_controller reports vcurrent - vtarget.
 * lateral_controller reports current_yaw - target_yaw and current_lat_distance -
 * target_lat_distance
 *
 * */
using ControllerErrorReportMsg = autoware_auto_vehicle_msgs::msg::ControllerErrorReport;

using VelocityMsg = nav_msgs::msg::Odometry;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using vehicle_info_util::VehicleInfoUtil;
using ErrorStampedControlPerfMsg = control_performance_analysis::msg::ErrorStamped;

// Parameters to pass around.
struct Parameters
{
  float64_t wheel_base{};
  float64_t cdob_ctrl_period{0.05};

  // Qfilter orders .
  int qfilter_lateral_error_order{3};
  int qfilter_heading_error_order{2};
  int qfilter_steering_order{1};
  int qfilter_velocity_error_order{1};
  int qfilter_acc_error_order{1};

  // Qfilter cut-off frequencies Hz. (low-pass).
  float64_t qfilter_lateral_error_freq{5};
  float64_t qfilter_heading_error_freq{5};
  float64_t qfilter_steering_freq{5};
  float64_t qfilter_velocity_error_freq{5};
  float64_t qfilter_acc_error_freq{5};

  // First order vehicle state models.
  float64_t steering_tau{0.3};
  float64_t velocity_tau{0.3};
  float64_t acc_tau{0.3};
};

template <typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = static_cast<T>(it->template get_value<T>());
  }
}

// The node class.
class CommunicationDelayCompensatorNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit CommunicationDelayCompensatorNode(const rclcpp::NodeOptions & node_options);

  /**
   * @brief destructor
   */
  ~CommunicationDelayCompensatorNode() override = default;

private:
  // Data Members
  Parameters params_node_{};

  //!< @brief timer to update after a given interval
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscribers
  rclcpp::Subscription<ControlCommand>::SharedPtr sub_control_cmds_;

  //!< @brief subscription for current velocity
  rclcpp::Subscription<VelocityMsg>::SharedPtr sub_current_velocity_ptr_;

  //!< @brief subscription for current velocity
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_current_steering_ptr_;

  //!< @brief subscription for current velocity error.
  rclcpp::Subscription<ControllerErrorReportMsg>::SharedPtr sub_current_long_error_ptr_;

  //!< @brief subscription for current lateral and heading errors.
  rclcpp::Subscription<ControllerErrorReportMsg>::SharedPtr sub_current_lat_errors_ptr_;

  //!< @brief subscription for current lateral and heading errors.
  rclcpp::Subscription<ErrorStampedControlPerfMsg>::SharedPtr sub_control_perf_errors_ptr_;

  // Publishers
  rclcpp::Publisher<DelayCompensatatorMsg>::SharedPtr pub_delay_compensator_;
  rclcpp::Publisher<DelayCompensatorDebugMsg>::SharedPtr pub_delay_compensator_debug_;

  // Data Members for the delay-compensation.
  std::shared_ptr<LinearKinematicErrorModel> vehicle_model_ptr_;
  std::unique_ptr<CommunicationDelayCompensatorCore> delay_comp_lat_error_ptr_{};
  std::unique_ptr<CommunicationDelayCompensatorCore> delay_comp_yaw_error_ptr_{};
  std::unique_ptr<CommunicationDelayCompensatorCore> delay_comp_steering_ptr_{};
  std::unique_ptr<CommunicationDelayCompensatorCore> delay_comp_velocity_error_ptr_{};
  std::unique_ptr<CommunicationDelayCompensatorCore> delay_compensator_acc_error_ptr_{};

  // Pointers to the ROS topics.
  // Pointers for ros topic
  // Pointers to the model state variables inputs
  std::shared_ptr<nav_msgs::msg::Odometry> current_velocity_ptr{nullptr};
  std::shared_ptr<SteeringReport> current_steering_ptr_{nullptr};
  float64_t previous_velocity_{};

  // Pointer to the model inputs
  std::shared_ptr<ControlCommand> current_ctrl_ptr_{nullptr};
  std::shared_ptr<ControlCommand> previous_ctrl_ptr_{nullptr};

  // Pointers to the compensator outputs.
  std::shared_ptr<DelayCompensatatorMsg> current_delay_references_msg_{nullptr};
  std::shared_ptr<ControllerErrorReportMsg> current_lateral_errors_{nullptr};
  std::shared_ptr<ControllerErrorReportMsg> current_longitudinal_errors_{nullptr};
  std::shared_ptr<ErrorStampedControlPerfMsg> current_cont_perf_errors_{nullptr};

  // Steering related.
  float64_t current_curvature_{};
  float64_t current_steering_error_{};  // from Ackerman angle.

  // Debug messages
  std::shared_ptr<DelayCompensatorDebugMsg> current_delay_debug_msg_{nullptr};

  // Node Methods
  //!< initialize timer to work in real, simulation, and replay
  void initTimer(float64_t period_s);

  /**
   * @brief compute and publish the compensating reference signals for the controllers with a
   * constant control period
   */
  void onTimer();

  /**
   * @brief Subscription callbacks
   */
  void onControlCommands(const ControlCommand::SharedPtr msg);

  /**
   * @brief Subscription callbacks
   */
  void onCurrentVelocity(const VelocityMsg::SharedPtr msg);

  /**
   * @brief Subscription callbacks
   */
  void onCurrentSteering(const SteeringReport::SharedPtr msg);

  /**
   * @brief Subscription to computed velocity error
   */
  void onCurrentLongitudinalError(const ControllerErrorReportMsg::SharedPtr msg);

  /**
   * @brief Subscription to lateral reference errors ey, eyaw.
   */
  void onCurrentLateralErrors(const ControllerErrorReportMsg::SharedPtr msg);

  /**
   * @brief Subscription to control performance errors.
   */
  void onControlPerfErrors(const ErrorStampedControlPerfMsg::SharedPtr msg);

  /**
   * @brief Publish message.
   * */

  void publishCompensationReferences();

  /**
   * @brief Check if data flows.
   * */
  bool8_t isDataReady();

  /**
   * @brief Default parameters of the parameters.
   * */

  void readAndLoadParameters();

  /**
   * @brief Dynamic update of the parameters.
   * */
  OnSetParametersCallbackHandle::SharedPtr is_parameters_set_res_;

  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Set steering tracking q-filter and 1st order steering linear model.
   * Steering is a state in the kinematic error model and it does not have an error term in this
   * application.
   * */
  void setSteeringCDOBcompensator();      // creates steering cdob compensator.
  void computeSteeringCDOBcompensator();  // computes corrective ref for the heading error

  void setHeadingErrorCDOBcompensator();  // creates a heading angle error compensator.
  void computeHeadingCDOBcompensator();

  void setLateralErrorCDOBcompensator();
  void computeLateralCDOBcompensator();

  void setVelocityErrorCDOBcompensator();
  void computeVelocityCDOBcompensator();

  void setAccelerationErrorCDOBcompensator();
  void computeAccelerationCDOBcompensator();

  /**
   * @brief checks if vehicle is stopping.
   * */
  bool8_t isVehicleStopping();

  /**
   * @brief updates the vehicle model.
   * */
  void updateVehicleModel();

  /**
   * @brief placeholders for delay compensator outputs.
   * */

  /**
   * @brief Outputs of the delay compensator.
   * y0: u_filtered,Q(s)*u where u is the input sent to the system.
   * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
   * y2: du = y0 - y1 where du is the estimated disturbance input
   * y3: ydu = G(s)*du where ydu is the response of the system to du.
   * */
  std::vector<float64_t> cdob_lateral_error_y_outputs_{};
  std::vector<float64_t> cdob_heading_error_y_outputs_{};
  std::vector<float64_t> cdob_steering_y_outputs_{};
  std::vector<float64_t> cdob_velocity_error_y_outputs_{};
  std::vector<float64_t> cdob_acc_error_y_outputs_{};
};

}  // namespace observers
#endif  // COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP
