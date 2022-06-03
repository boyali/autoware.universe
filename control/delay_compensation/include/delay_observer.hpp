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

#ifndef DELAY_COMPENSATION__DELAY_OBSERVER_H
#define DELAY_COMPENSATION__DELAY_OBSERVER_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include "visibility_control.hpp"
#include "qfilters.hpp"
#include "autoware_control_toolbox.hpp"


// class __attribute__((__visibility__("default"))) DelayCompensator
template<typename eigenT>
class CDOB_PUBLIC DelayCompensator
{
public:
	using pairs = std::pair<std::string_view, std::string_view>;

	// Constructors.
	DelayCompensator() = default;

	DelayCompensator(s_filter_data const& Qfilter_data, s_model_G_data& Gdata);

	void print() const;

	// simulate  one-step and get the outputs.
	std::array<double, 4> simulateOneStep();


private:

	// Associated Qfilter parameters
	int q_order_{ 1 }; // @brief order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	double q_cut_off_frequency_{}; // @brief cut-off frequency in Hz.
	double q_time_constant_tau_{};

	// Qfilter transfer function.
	ns_control_toolbox::tf Qfilter_tf_{}; // @brief

	/**
	 * @brief  Associated model parameters as multiplication factors for num and den of G and Q/G.
	 * */
	pairs num_den_constant_names_G_{ "1.0", "1.0" };
	pairs num_den_constant_names_QGinv_{ "1.0", "1.0" };


	// Model transfer function.
	ns_control_toolbox::tf G_{};

	// Q(s)/G(s)
	ns_control_toolbox::tf QGinv_{};

	// Internal states.
	eigenT x0_filter_{ eigenT::Zero() };

	/**
	 * @brief Outputs of the delay compensator.
	 * y0: u_filtered,Q(s)*u where u is the input sent to the system.
	 * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
	 * y2: du = y0 - y1 where du is the estimated disturbance input
	 * y3: ydu = G(s)*du where ydu is the response of the system to du.
	 * */

	std::array<double, 4> y_outputs{};


};


#endif // DELAY_COMPENSATION__DELAY_OBSERVER_H
