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

// LIBRARY HEADERS
#include "autoware_control_toolbox.hpp"
#include "visibility_control.hpp"

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace observers
{
	using namespace std::chrono_literals;

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
		~CommunicationDelayCompensatorNode() override = default;

	private:
		// Data Members
		//!< @brief timer to update after a given interval
		rclcpp::TimerBase::SharedPtr timer_;

		// Node Methods
		//!< initialize timer to work in real, simulation, and replay
		void initTimer(float64_t period_s);

		/**
		 * @brief compute and publish the compensating reference signals for the controllers with a
		 * constant control period
		 */
		void onTimer();
		};

} // namespace observers
#endif  // COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP
