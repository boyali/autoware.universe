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
// For internal states use state_T, for ABCD matrices use mat_eig_T.
template<int Norder>
class DelayCompensatorBaseTypes
{
public:
	using state_vec_type = Eigen::Matrix<double, Norder, 1>;
	using mat_Atype = Eigen::Matrix<double, Norder, Norder>;
	using mat_Btype = Eigen::Matrix<double, Norder, 1>;
	using mat_Ctype = Eigen::Matrix<double, 1, Norder>;
	using mat_Dtype = Eigen::Matrix<double, 1, 1>;
	using pairs_t = s_model_g_data::pairs_t;
	using pairs_func_maps_t = s_model_g_data::pairs_func_maps_t;
};


template<int Norder>
class CDOB_PUBLIC DelayCompensator : public DelayCompensatorBaseTypes<Norder>
{
public:

	using state_vec_type = Eigen::Matrix<double, Norder, 1>;
	using mat_Atype = Eigen::Matrix<double, Norder, Norder>;
	using mat_Btype = Eigen::Matrix<double, Norder, 1>;
	using mat_Ctype = Eigen::Matrix<double, 1, Norder>;
	using mat_Dtype = Eigen::Matrix<double, 1, 1>;
	using pairs_t = s_model_g_data::pairs_t;
	using pairs_func_maps_t = s_model_g_data::pairs_func_maps_t;


	// Constructors.
	DelayCompensator() = default;

	DelayCompensator(s_filter_data const& qfilter_data,
	                 s_model_g_data& model_data, double const& dt);

	void print() const;

	// simulate  one-step and get the outputs.
	std::array<double, 4> simulateOneStep(double const& input,
	                                      std::pair<double, double> const& num_den_args_of_G,
	                                      std::array<double, 4>& y_outputs);

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
	 * i.e G(s) = f(V) num/ (g(delta) den). V and delta are the arguments stored in the pairs_t.
	 * */
	pairs_t num_den_constant_names_G_{ "1", "1" };
	pairs_t num_den_constant_names_QGinv_{ "1", "1" };

	// and their functions.
	pairs_func_maps_t pair_func_map_{};

	// Model transfer function.
	ns_control_toolbox::tf Gtf_{};
	ns_control_toolbox::tf2ss Gss_;

	// Q(s)/G(s)
	ns_control_toolbox::tf QGinv_tf_{};
	ns_control_toolbox::tf2ss QGinv_ss_{};

	// Internal states.
	state_vec_type x0_qfilter_{ state_vec_type::Zero() };
	state_vec_type x0_gsystem_{ state_vec_type::Zero() };
	state_vec_type x0_qg_inv_system_{ state_vec_type::Zero() };

	// Placeholders for Ad, Bd, Cd, Dd.
	Eigen::MatrixXd Ad_{};
	Eigen::MatrixXd Bd_{};
	Eigen::MatrixXd Cd_{};
	Eigen::MatrixXd Dd_{};

	/**
	 * @brief Outputs of the delay compensator.
	 * y0: u_filtered,Q(s)*u where u is the input sent to the system.
	 * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
	 * y2: du = y0 - y1 where du is the estimated disturbance input
	 * y3: ydu = G(s)*du where ydu is the response of the system to du.
	 * */

	std::array<double, 4> y_outputs_{};

	// Member functions.
	void getSSsystem(ns_control_toolbox::tf2ss const& ss);

};

template<int Norder>
DelayCompensator<Norder>::DelayCompensator(s_filter_data const& qfilter_data,
                                           s_model_g_data& model_data, double const& dt) :
		dt_{ dt }, q_order_{ qfilter_data.order },
		q_cut_off_frequency_{ qfilter_data.cut_off_frq },
		q_time_constant_tau_{ qfilter_data.time_constant },
		Qfilter_tf_(qfilter_data.TF),
		num_den_constant_names_G_{ model_data.num_den_coeff_names },
		pair_func_map_{ model_data.funcs },
		Gtf_(model_data.TF)
{

	// Compute the state-space model of Qfilter.
	Qfilter_ss_ = ns_control_toolbox::tf2ss(qfilter_data.TF, dt);

	// Compute the state-space model of the system model G(s).
	Gss_ = ns_control_toolbox::tf2ss(Gtf_, dt);

	// Compute Q/G
	auto tempG = std::move(model_data.TF);
	tempG.inv();

	QGinv_tf_ = Qfilter_tf_ * tempG;

	// Compute the state-space model of QGinv(s)
	QGinv_ss_ = ns_control_toolbox::tf2ss(QGinv_tf_, dt);

	// Invert the num den constant names.
	num_den_constant_names_QGinv_ = pairs_t(num_den_constant_names_G_.second, num_den_constant_names_G_.first);

}

template<int Norder>
void DelayCompensator<Norder>::print() const
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
 * [in] u : control input for this system (delta:steering in lateral control).
 * [in] ym: measured response i.e ey, eyaw, edelta, eV.
 * [out] y0: u_filtered,Q(s)*u where u is the input sent to the system.
 * [out] y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
 * [out] y2: du = y0 - y1 where du is the estimated disturbance input
 * [out] y3: ydu = G(s)*du where ydu is the response of the system to du.
 * */
template<int Norder>
std::array<double, 4> DelayCompensator<Norder>::simulateOneStep(double const& input,
                                                                std::pair<double, double> const& num_den_args_of_G,
                                                                std::array<double, 4>& y_outputs)
{
	// Get the output of num_constant for the G(s) = nconstant(x) * num / (denconstant(y) * den)
	if (num_den_constant_names_G_.first != "1")
	{
		auto&& numkey = num_den_constant_names_G_.first; // Corresponds to v in ey model.
		auto&& num_const_g = pair_func_map_[numkey](num_den_args_of_G.first);

		Gtf_.update_num_coef(num_const_g);
		QGinv_tf_.update_den_coef(num_const_g); // since they are inverted.

		ns_utils::print("input  : ", input, numkey, " : ", num_const_g);
	}

	if (num_den_constant_names_G_.second != "1")
	{
		auto&& denkey = num_den_constant_names_G_.second; // Corresponds to delta in ey model.
		auto&& den_const_g = pair_func_map_[denkey](num_den_args_of_G.second);

		Gtf_.update_den_coef(den_const_g);
		QGinv_tf_.update_num_coef(den_const_g); // since they are inverted.

		ns_utils::print("input  : ", input, denkey, " : ", den_const_g);
	}

	// If any of num den constant changes, update the state-space models.
	if (num_den_constant_names_G_.first != "1" || num_den_constant_names_G_.second != "1")
	{
		// Update Gss accordingly from the TF version.
		Gss_ = ns_control_toolbox::tf2ss(Gtf_, dt_);
		QGinv_ss_ = ns_control_toolbox::tf2ss(QGinv_tf_, dt_);
	}

	// Get Ad_, Bd_, Cd_,Dd_ from QGinv_ss_.
	// getSSsystem(QGinv_ss_);

	// Print matrices.
//	ns_eigen_utils::printEigenMat(Ad_, "Ad:");
//	ns_eigen_utils::printEigenMat(Bd_, "Bd:");
//	ns_eigen_utils::printEigenMat(Cd_, "Cd:");
//	ns_eigen_utils::printEigenMat(Dd_, "Dd:");


	// ns_utils::print("input  : ", input, numkey, " : ", num_const_g, ", ", denkey, " : ", den_const_g);
	y_outputs = std::array<double, 4>{ 1, 2, 3, 4 };

	return std::array<double, 4>{};

}

template<int Norder>
void DelayCompensator<Norder>::getSSsystem(ns_control_toolbox::tf2ss const& ss)
{

	Ad_ = ss.Ad();
	Bd_ = ss.B();
	Cd_ = ss.Cd();
	Dd_ = ss.Dd();

}

#endif // DELAY_COMPENSATION__DELAY_OBSERVER_H
