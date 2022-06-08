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

#ifndef DELAY_COMPENSATOR__DELAY_OBSERVER_HPP
#define DELAY_COMPENSATOR__DELAY_OBSERVER_HPP

#include <cmath>
#include <iostream>
#include <utility>
#include <unordered_map>
#include <eigen3/Eigen/Core>
#include "visibility_control.hpp"
#include "qfilters.hpp"
#include "autoware_control_toolbox.hpp"

// class __attribute__((__visibility__("default"))) DelayCompensator
// For internal states use state_T, for ABCD matrices use mat_eig_T.


class DelayCompensator
	{
public:

	using pairs_t = s_model_g_data::pairs_t;
	using pairs_func_maps_t = s_model_g_data::pairs_func_maps_t;


	// Constructors.
	DelayCompensator() = default;

	DelayCompensator(s_filter_data const& qfilter_data,
			s_model_g_data& model_data, double const& dt);

	void print() const;

	// simulate  one-step and get the outputs.
	/**
	 * @brief Simulates one-step and produces the outputs.
	 * @param previous_input: previous input sent to the vehicle
	 * @param measured_model_state: the state that tracks a reference (i.e ey, eyaw, eV).
	 * @param num_den_args_of_G: the model states that define the transfer function, i.e (V^2/(cos(delta)*s + 1).
	 * @param outputs from the delay compensator.
	 * */
	void simulateOneStep(double const& previous_input, /** previous input*/
			double const& measured_model_state,
			std::pair<double, double> const& num_den_args_of_G, /** model parameters*/
			std::array<double, 4>& y_outputs);

private:
	double dt_{ 0.1 };

	// Associated Qfilter parameters
	int    q_order_{ 1 }; // @brief order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	double q_cut_off_frequency_{}; // @brief cut-off frequency in Hz.
	double q_time_constant_tau_{};

	// Qfilter transfer function.
	ns_control_toolbox::tf    Qfilter_tf_{}; // @brief Transfer function of the qfilter.
	ns_control_toolbox::tf2ss Qfilter_ss_{}; //@brief State space model of the qfilter


	/**
	 * @brief  Associated model parameters as multiplication factors for num and den of G and Q/G.
	 * i.e G(s) = f(V) num/ (g(delta) den). V and delta are the arguments stored in the pairs_t.
	 * */
	pairs_t num_den_constant_names_G_{ "1", "1" };
	pairs_t num_den_constant_names_QGinv_{ "1", "1" };

	// and their functions.
	pairs_func_maps_t pair_func_map_{};

	// Model transfer function.
	ns_control_toolbox::tf    Gtf_{};
	ns_control_toolbox::tf2ss Gss_;

	// Q(s)/G(s)
	ns_control_toolbox::tf    QGinv_tf_{};
	ns_control_toolbox::tf2ss QGinv_ss_{};

	// Internal states.
	Eigen::MatrixXd x0_qfilter_{}; // u--> Q(s) -->ufiltered
	Eigen::MatrixXd x0_gsystem_{}; // u--> G(s)-->y (i,e ey, eyaw ..)
	Eigen::MatrixXd x0_inv_system_{}; // y--> Q(s)/G(s) --> u-du

	// Placeholders for Ad, Bd, Cd, Dd.
	Eigen::MatrixXd Ad_{};
	Eigen::MatrixXd Bd_{};
	Eigen::MatrixXd Cd_{};
	Eigen::MatrixXd Dd_{};

	/**
	 * @brief Outputs of the delay compensator.
	 * y0: u_filtered,Q(s)*u where u is the input sent to the system.
	 * y2: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
	 * y2: du = y0 - y1 where du is the estimated disturbance input
	 * y3: ydu = G(s)*du where ydu is the response of the system to du.
	 * */
	std::array<double, 4> y_outputs_{};

	// Member functions.
	void getSSsystem(ns_control_toolbox::tf2ss const& ss);

	};


#endif // DELAY_COMPENSATOR__DELAY_OBSERVER_HPP
