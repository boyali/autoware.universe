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

namespace autoware::motion::control::observer
{
	CommunicationDelayCompensatorNode::CommunicationDelayCompensatorNode(
			const rclcpp::NodeOptions& node_options)
			: Node("lateral_controller", node_options)
	{
		using std::placeholders::_1;

		/* set up ros system */
		float64_t duration{ 50 };  // ms
		initTimer(duration);
	}

	void CommunicationDelayCompensatorNode::initTimer(float64_t period_s)
	{
		const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::duration<float64_t>(period_s));

		m_timer_ = rclcpp::create_timer(
				this, get_clock(), period_ns, std::bind(&CommunicationDelayCompensatorNode::onTimer, this));
	}

	void CommunicationDelayCompensatorNode::onTimer()
	{
		// RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "In the delay compensator node ");

		// RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "Hello world");

		// RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Hello World!");

		// RCLCPP_ERROR(get_logger(), "Trajectory is invalid!, stop computing.");

		// RCLCPP_DEBUG(get_logger(), "MPC does not have a QP solver");

		RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 1000 /*ms*/,
				"waiting for vehicle measured steering message ...");
	}

} // namespace autoware::motion::control::observer

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::observer::CommunicationDelayCompensatorNode)