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

	DelayCompensator(s_filter_data const& Qfilter_data,
	                 s_model_G_data& Gdata, double const& dt);

	void print() const;

	// simulate  one-step and get the outputs.
	std::array<double, 4> simulateOneStep();


private:
	double dt_{ 0.1 };

	// Associated Qfilter parameters
	int q_order_{ 1 }; // @brief order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	double q_cut_off_frequency_{}; // @brief cut-off frequency in Hz.
	double q_time_constant_tau_{};

	// Qfilter transfer function.
	ns_control_toolbox::tf Qfilter_tf_{}; // @brief Transfer function of the qfilter.
	ns_control_toolbox::tf2ss Qfilter_ss_{}; //@brief State space model of the qfilter


	/**
	 * @brief  Associated model parameters as multiplication factors for num and den of G and Q/G.
	 * */
	pairs num_den_constant_names_G_{ "1", "1" };
	pairs num_den_constant_names_QGinv_{ "1", "1" };


	// Model transfer function.
	ns_control_toolbox::tf Gtf_{};
	ns_control_toolbox::tf2ss Gss_;

	// Q(s)/G(s)
	ns_control_toolbox::tf QGinv_tf_{};
	ns_control_toolbox::tf2ss QGinv_ss_{};

	// Internal states.
	eigenT x0_Qfilter_{ eigenT::Zero() };
	eigenT x0_Gsystem_{ eigenT::Zero() };
	eigenT x0_QGinvsystem_{ eigenT::Zero() };


	/**
	 * @brief Outputs of the delay compensator.
	 * y0: u_filtered,Q(s)*u where u is the input sent to the system.
	 * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
	 * y2: du = y0 - y1 where du is the estimated disturbance input
	 * y3: ydu = G(s)*du where ydu is the response of the system to du.
	 * */

	std::array<double, 4> y_outputs{};


};


template<typename eigenT>
DelayCompensator<eigenT>::DelayCompensator(s_filter_data const& Qfilter_data,
                                           s_model_G_data& Gdata, double const& dt) :
		dt_{ dt },
		q_cut_off_frequency_{ Qfilter_data.cut_off_frq },
		q_time_constant_tau_{ Qfilter_data.time_constant },
		Qfilter_tf_(Qfilter_data.TF),
		num_den_constant_names_G_{ Gdata.num_den_coeff_names },
		Gtf_(Gdata.TF)
{

	// Compute the state-space model of Qfilter.
	Qfilter_ss_ = ns_control_toolbox::tf2ss(Qfilter_data.TF, dt);

	// Compute the state-space model of the system model G(s).
	Gss_ = ns_control_toolbox::tf2ss(Gtf_, dt);

	// Compute Q/G
	auto tempG = std::move(Gdata.TF);
	tempG.inv();

	QGinv_tf_ = Qfilter_tf_ * tempG;

	// Compute the state-space model of QGinv(s)
	QGinv_ss_ = ns_control_toolbox::tf2ss(QGinv_tf_, dt);

	// Invert the num den constant names.
	num_den_constant_names_QGinv_ = pairs(num_den_constant_names_G_.second, num_den_constant_names_G_.first);


}

template<typename eigenT>
void DelayCompensator<eigenT>::print() const
{
	ns_utils::print("Delay Compensator Summary :  \n");
	ns_utils::print("Qfilter Model : \n");
	Qfilter_tf_.print();

	ns_utils::print("Forward model G(s) : \n\n");
	Gtf_.print();

	ns_utils::print("G(s) num and den constant names :", num_den_constant_names_G_.first,
	                num_den_constant_names_G_.second, "\n");

	ns_utils::print("Forward model Q/G(s) :  \n");
	QGinv_tf_.print();

	ns_utils::print("Q/G(s) num and den constant names :", num_den_constant_names_QGinv_.first,
	                num_den_constant_names_QGinv_.second);

	ns_utils::print("\n -------------- DISCRETE STATE-SPACE MODELS ----------\n");
	ns_utils::print("Qfilter State-Space: ");
	Qfilter_ss_.print_discrete_system();

	ns_utils::print("\n -------------- DISCRETE STATE-SPACE MODELS ----------\n");
	ns_utils::print("System G(s) State-Space: ");
	Gss_.print_discrete_system();

	ns_utils::print("\n -------------- DISCRETE STATE-SPACE MODELS ----------\n");
	ns_utils::print("System Q(s)/G(s) State-Space: ");
	QGinv_ss_.print_discrete_system();


}

/**
 * @brief Given an input "u" that sent to the system and system response "y", computes four outputs by means of the
 * internal models for the filter Qf, for the system model G(s) for the Q-filtered inverse model.
 * [in] u : control input for this system.
 * [in] ym: measured response i.e ey, eyaw, edelta, eV.
 * [out] y0: u_filtered,Q(s)*u where u is the input sent to the system.
 * [out] y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
 * [out] y2: du = y0 - y1 where du is the estimated disturbance input
 * [out] y3: ydu = G(s)*du where ydu is the response of the system to du.
 * */
template<typename eigenT>
std::array<double, 4> DelayCompensator<eigenT>::simulateOneStep()
{
	//

	return std::array<double, 4>();
}

#endif // DELAY_COMPENSATION__DELAY_OBSERVER_H
