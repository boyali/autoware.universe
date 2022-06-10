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

			/* set up ros system */
			float64_t duration{ 50 };  // ms
			initTimer(duration);

			// Create Publishers
			pub_delay_compensator_ = create_publisher<DelayCompensatatorMsg>(
				"~/output/communication_delay_compensation_refs", 1);

			// Create subscriptions
			sub_control_cmds_ = create_subscription<ControlCommand>("~/input/control_cmd", rclcpp::QoS{ 1 },
			                                                        std::bind(&observers::CommunicationDelayCompensatorNode::onControlCommands,
			                                                                  this,
			                                                                  std::placeholders::_1));

			sub_current_velocity_ptr_ = create_subscription<VelocityMsg>("~/input/current_odometry", rclcpp::QoS{ 1 },
			                                                             std::bind
				                                                             (&observers::CommunicationDelayCompensatorNode::onCurrentVelocity,
				                                                              this,
				                                                              std::placeholders::_1));

			sub_current_steering_ptr_ = create_subscription<SteeringReport>("~/input/steering_state", rclcpp::QoS{ 1 },
			                                                                std::bind
				                                                                (&observers::CommunicationDelayCompensatorNode::onCurrentSteering,
				                                                                 this,
				                                                                 std::placeholders::_1));
		}

		void CommunicationDelayCompensatorNode::initTimer(float64_t period_s)
		{
			const auto period_ns = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::duration<float64_t>(period_s));

			timer_ = rclcpp::create_timer(
				this, get_clock(), period_ns, std::bind(&CommunicationDelayCompensatorNode::onTimer, this));
		}

		void CommunicationDelayCompensatorNode::onTimer()
		{
			// RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "In the delay compensator node ");

			// RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "Hello world");

			// RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Hello World!");

			// RCLCPP_ERROR(get_logger(), "Trajectory is invalid!, stop computing.");

			// RCLCPP_DEBUG(get_logger(), "MPC does not have a QP solver");

			publishCompensationReferences();

			ns_utils::print("ACT On timer method ");

			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500 /*ms*/, "onTimerCommands");
		}

		void CommunicationDelayCompensatorNode::onControlCommands(const ControlCommand::SharedPtr msg)
		{
			current_ctrl_ptr_ = std::make_shared<ControlCommand>(*msg);
			ns_utils::print("ACT On control method ");
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500 /*ms*/, "onControlCommands");
		}

		void CommunicationDelayCompensatorNode::onCurrentVelocity(const VelocityMsg::SharedPtr msg)
		{
			current_velocity_ptr = std::make_shared<VelocityMsg>(*msg);

			ns_utils::print("ACT On velocity method ");
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500 /*ms*/, "onControlCommands");

		}

		void CommunicationDelayCompensatorNode::publishCompensationReferences()
		{
			current_delay_references_->heading_angle_error_compensation_ref = 1.0;
			current_delay_references_->lateral_deviation_error_compensation_ref = 2.0;

			pub_delay_compensator_->publish(*current_delay_references_);

		}
		void CommunicationDelayCompensatorNode::onCurrentSteering(const SteeringReport::SharedPtr msg)
		{
			ns_utils::print("ACT On steering method ");
			current_steering_ptr_ = std::make_shared<SteeringReport>(*msg);
		}

}  // namespace observers
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(observers::CommunicationDelayCompensatorNode)