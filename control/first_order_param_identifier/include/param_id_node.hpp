
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

#ifndef PARAM_IDENTIFIER_FO_INCLUDE_PARAM_ID_NODE_HPP_
#define PARAM_IDENTIFIER_FO_INCLUDE_PARAM_ID_NODE_HPP_

// Standard libraries.
#include "memory"
#include <vector>
#include "eigen3/Eigen/Core"

// Autoware headers
#include "param_id_core.hpp"

#include "common/types.hpp"
#include "node_definitions.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// ROS headers
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float64.hpp"

namespace sys_id
{

using ControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;

class ParameterIdentificationNode : public rclcpp::Node
{

 public:

  /**
   * @brief constructor
   */

  explicit ParameterIdentificationNode(const rclcpp::NodeOptions &node_options);

  /**
   * @brief destructor
   */
  ~ParameterIdentificationNode() override = default;

 private:
  sNodeParameters params_node_{};

  // Core
  std::unique_ptr<ParamIDCore> param_id_core_{nullptr};

  // ROS publishers.
  //!< @brief timer to update after a given interval
  rclcpp::TimerBase::SharedPtr timer_;

  //!< @brief subscription for current velocity
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_current_steering_ptr_;

  // Subscribers
  rclcpp::Subscription<ControlCommand>::SharedPtr sub_control_cmds_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_parameter_;

  // Pointers to the ROS topics.
  std::shared_ptr<ControlCommand> current_control_cmd_ptr_{nullptr};
  std::shared_ptr<SteeringReport> current_steering_ptr_{nullptr};

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
  void onCurrentSteering(const SteeringReport::SharedPtr msg);

  void loadParams();

  void publishParameter();

  /**
   * @brief Check if data flows.
   * */
  bool8_t isDataReady();
};

} // namespace sys_id

#endif //PARAM_IDENTIFIER_FO_INCLUDE_PARAM_ID_NODE_HPP_
