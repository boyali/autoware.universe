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

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tier4_vehicle_msgs/msg/delay_compensators_stamped.hpp"

// LIBRARY HEADERS
#include "autoware_control_toolbox.hpp"
#include "visibility_control.hpp"

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace observers
{
		using namespace std::chrono_literals;
		using ControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
		using DelayCompensatatorMsg = tier4_vehicle_msgs::msg::DelayCompensators;
		using VelocityMsg = nav_msgs::msg::Odometry;

		class CommunicationDelayCompensatorNode : public rclcpp::Node
		{
		public:
				using float64_t = autoware::common::types::float64_t;

				/**
				 * @brief constructor
				 */
				explicit CommunicationDelayCompensatorNode(const rclcpp::NodeOptions& node_options);

				/**
				 * @brief destructor
				 */
				~CommunicationDelayCompensatorNode() override;

		private:
				// Data Members
				//!< @brief timer to update after a given interval
				rclcpp::TimerBase::SharedPtr timer_;

				// Subscribers
				rclcpp::Subscription<ControlCommand>::SharedPtr sub_control_cmds_;

				//!< @brief subscription for current velocity
				rclcpp::Subscription<VelocityMsg>::SharedPtr sub_current_velocity_ptr_;

				// Publishers
				rclcpp::Publisher<DelayCompensatatorMsg>::SharedPtr pub_delay_compensator_;

				// Pointers to the ROS topics.
				// pointers for ros topic
				std::shared_ptr<nav_msgs::msg::Odometry> current_velocity_ptr{ nullptr };
				std::shared_ptr<ControlCommand> current_ctrl_ptr_{ nullptr };

				// Node Methods
				//!< initialize timer to work in real, simulation, and replay
				void initTimer(float64_t period_s);

				/**
				 * @brief compute and publish the compensating reference signals for the controllers with a
				 * constant control period
				 */
				void onTimer();

				/**
				 * @brief  subscription callbacks
				 */
				void onControlCommands(const ControlCommand::SharedPtr msg);

				/**
				 * @brief  subscription callbacks
				 */
				void onCurrentVelocity(const VelocityMsg::SharedPtr msg);
		};

}  // namespace observers
#endif  // COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP
