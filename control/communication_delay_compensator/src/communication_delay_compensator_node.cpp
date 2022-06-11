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
		CommunicationDelayCompensatorNode::CommunicationDelayCompensatorNode(const rclcpp::NodeOptions& node_options)
			: Node("communication_delay_compensator", node_options)
		{
			using std::placeholders::_1;

			// Read vehicle model parameters
			// Implement Reading Global and Local Variables.
			// 			const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
			// 			params_.wheel_base = vehicle_info.wheel_base_m;

			/* Set up ros system timer and read the parameters */
			params_.cdob_ctrl_period = declare_parameter<float64_t>("cdob_ctrl_period");  // reads sec
			initTimer(params_.cdob_ctrl_period);

			// m_wheel_base = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo().wheel_base_m;

			// Create Publishers
			pub_delay_compensator_ = create_publisher<DelayCompensatatorMsg>(
				"~/output/communication_delay_compensation_refs", 1);

			// Create subscriptions
			sub_control_cmds_ = create_subscription<ControlCommand>("~/input/control_cmd", rclcpp::QoS{ 1 },
			                                                        std::bind(&observers::CommunicationDelayCompensatorNode::onControlCommands,
			                                                                  this, std::placeholders::_1));

			sub_current_velocity_ptr_ = create_subscription<VelocityMsg>("~/input/current_odometry", rclcpp::QoS{ 1 },
			                                                             std::bind(&observers::CommunicationDelayCompensatorNode::onCurrentVelocity,
			                                                                       this, std::placeholders::_1));

			sub_current_steering_ptr_ = create_subscription<SteeringReport>("~/input/steering_state", rclcpp::QoS{ 1 },
			                                                                std::bind(&observers::CommunicationDelayCompensatorNode::onCurrentSteering,
			                                                                          this, std::placeholders::_1));

			sub_current_long_error_ptr_ =
				create_subscription<ControllerErrorReportMsg>("~/input/long_errors", rclcpp::QoS{ 1 },
				                                              std::bind
					                                              (&observers::CommunicationDelayCompensatorNode::onCurrentLongitudinalError,
					                                               this, std::placeholders::_1));

			sub_current_lat_errors_ptr_ =
				create_subscription<ControllerErrorReportMsg>("~/input/lat_errors", rclcpp::QoS{ 1 },
				                                              std::bind
					                                              (&observers::CommunicationDelayCompensatorNode::onCurrentLateralErrors,
					                                               this, std::placeholders::_1));

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

			if (!isDataReady())
			{
				RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Not enough data to compute delay compensation");
				return;

			}

			// Publish delay compensation reference.
			publishCompensationReferences();

			// Debug
			// RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "In the delay compensator node ");

			RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Hello world");

			// RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Hello World!");
			// RCLCPP_ERROR(get_logger(), "Trajectory is invalid!, stop computing.");
			// RCLCPP_DEBUG(get_logger(), "MPC does not have a QP solver");
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Timer");
			// ns_utils::print("ACT On timer method ");
			// end of debug
		}

		void CommunicationDelayCompensatorNode::onControlCommands(const ControlCommand::SharedPtr msg)
		{
			current_ctrl_ptr_ = std::make_shared<ControlCommand>(*msg);


			// Debug
			// ns_utils::print("ACT On control method ");
			// ns_utils::print("Read parameter control period :", params_.cdob_ctrl_period);
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "onControlCommands");
			// end of debug
		}

		void CommunicationDelayCompensatorNode::onCurrentVelocity(const VelocityMsg::SharedPtr msg)
		{

			current_velocity_ptr = std::make_shared<VelocityMsg>(*msg);

			// ns_utils::print("ACT On velocity method ");
			// ns_utils::print("Read parameter control period :", params_.cdob_ctrl_period);
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Velocity");
		}

		void CommunicationDelayCompensatorNode::onCurrentLongitudinalError(ControllerErrorReportMsg::SharedPtr const msg)
		{
			current_longitudinal_errors_ = std::make_shared<ControllerErrorReportMsg>(*msg);


			// Debug
			// auto vel_error = static_cast<double>(current_longitudinal_errors_->velocity_error_read);
			// ns_utils::print("Longitudinal velocity error :", vel_error);

			RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Longitudinal Error");
			// end of debug

		}

		void CommunicationDelayCompensatorNode::onCurrentLateralErrors(ControllerErrorReportMsg::SharedPtr const msg)
		{
			current_lateral_errors_ = std::make_shared<ControllerErrorReportMsg>(*msg);

			// Debug
			// auto lat_error = static_cast<double>(current_lateral_errors_->lateral_deviation_read);
			// auto heading_error = static_cast<double>(current_lateral_errors_->heading_angle_error_read);

			// ns_utils::print("Current lateral errors : ", lat_error, heading_error);
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Lateral Errors");
			// end of debug.

		}

		void CommunicationDelayCompensatorNode::publishCompensationReferences()
		{
			DelayCompensatatorMsg new_msg{};
			new_msg.lateral_deviation_error_compensation_ref = 1.0;
			current_delay_references_ = std::make_shared<DelayCompensatatorMsg>(new_msg);
			pub_delay_compensator_->publish(*current_delay_references_);

		}
		void CommunicationDelayCompensatorNode::onCurrentSteering(const SteeringReport::SharedPtr msg)
		{

			current_steering_ptr_ = std::make_shared<SteeringReport>(*msg);

			// Debug
			RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
			                               "[communication_delay] On Steering  ...");
			// ns_utils::print("ACT On steering method ");
			// end of debug
		}

		bool8_t CommunicationDelayCompensatorNode::isDataReady()
		{
			if (!current_velocity_ptr)
			{
				RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
				                               "[mpc_nonlinear] Waiting for the velocity measurement ...");
				return false;
			}

			if (!current_steering_ptr_)
			{
				RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
				                               "[mpc_nonlinear] Waiting for the steering measurement ...");
				return false;
			}

			if (!current_ctrl_ptr_)
			{
				RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), (1000ms).count(),
				                               "[mpc_nonlinear] Waiting for the control command ...");
				return false;
			}

			return true;
		}

}  // namespace observers
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(observers::CommunicationDelayCompensatorNode)