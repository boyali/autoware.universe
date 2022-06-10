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
#include <vehicle_info_util/vehicle_info_util.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "tier4_vehicle_msgs/msg/delay_compensators_stamped.hpp"
#include  "autoware_auto_vehicle_msgs/msg/delay_compensation_refs.hpp"
#include "autoware_auto_vehicle_msgs/msg/controller_error_report.hpp"

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
		// using DelayCompensatatorMsg = tier4_vehicle_msgs::msg::DelayCompensators;
		using DelayCompensatatorMsg = autoware_auto_vehicle_msgs::msg::DelayCompensationRefs;

		/**
		 * @brief longitudinal_controller reports vcurrent - vtarget.
		 * lateral_controller reports current_yaw - target_yaw and current_lat_distance - target_lat_distance
		 *
		 * */
		using ControllerErrorReportMsg = autoware_auto_vehicle_msgs::msg::ControllerErrorReport;

		using VelocityMsg = nav_msgs::msg::Odometry;
		using vehicle_info_util::VehicleInfoUtil;
		using autoware_auto_vehicle_msgs::msg::SteeringReport;

		using float64_t = autoware::common::types::float64_t;
		using float32_t = autoware::common::types::float32_t;

		struct Parameters
		{
				float64_t wheel_base{};
				float32_t cdob_ctrl_period{};
		};

		class CommunicationDelayCompensatorNode : public rclcpp::Node
		{
		public:

				/**
				 * @brief constructor
				 */
				explicit CommunicationDelayCompensatorNode(const rclcpp::NodeOptions& node_options);

				/**
				 * @brief destructor
				 */
				virtual ~CommunicationDelayCompensatorNode() = default;

		private:
				// Data Members
				Parameters params_{};

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

				// Publishers
				rclcpp::Publisher<DelayCompensatatorMsg>::SharedPtr pub_delay_compensator_;

				// Pointers to the ROS topics.
				// pointers for ros topic
				std::shared_ptr<nav_msgs::msg::Odometry> current_velocity_ptr{ nullptr };
				std::shared_ptr<ControlCommand> current_ctrl_ptr_{ nullptr };
				std::shared_ptr<SteeringReport> current_steering_ptr_{ nullptr };
				std::shared_ptr<DelayCompensatatorMsg> current_delay_references_{ nullptr };
				std::shared_ptr<ControllerErrorReportMsg> current_lateral_errors_{ nullptr };
				std::shared_ptr<ControllerErrorReportMsg> current_longitudinal_errors_{ nullptr };

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

				/**
				 * @brief  subscription callbacks
				 */
				void onCurrentSteering(const SteeringReport::SharedPtr msg);

				/**
				 * @brief  subscription to computed velocity error
				 */
				void onCurrentLongitudinalError(const ControllerErrorReportMsg::SharedPtr msg);

				/**
				 * @brief  subscription to lateral reference errors ey, eyaw.
				 */
				void onCurrentLateralErrors(const ControllerErrorReportMsg::SharedPtr msg);

				/**
				 * @brief publish message.
				 * */
				void publishCompensationReferences();
		};

}  // namespace observers
#endif  // COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP
