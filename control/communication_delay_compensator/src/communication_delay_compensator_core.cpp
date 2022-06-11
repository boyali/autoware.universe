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

#include "communication_delay_compensator_core.hpp"

CommunicationDelayCompensatorCore::CommunicationDelayCompensatorCore(s_filter_data const& qfilter_data,
                                                                     s_model_g_data& model_data, double const& dt) :
	dt_{ dt }, q_order_{ qfilter_data.order },
	q_cut_off_frequency_{ qfilter_data.cut_off_frq },
	q_time_constant_tau_{ qfilter_data.time_constant },
	Qfilter_tf_(qfilter_data.TF),
	num_den_constant_names_G_{ model_data.num_den_coeff_names },
	pair_func_map_{ model_data.funcs },
	Gtf_(model_data.TF)
{

	auto qfilter_order = Qfilter_tf_.order();
	auto system_model_order = Gtf_.order();
	auto inverse_system_order = std::max(qfilter_order, system_model_order);

	x0_qfilter_ = Eigen::MatrixXd(qfilter_order, 1);
	x0_gsystem_ = Eigen::MatrixXd(system_model_order, 1);
	x0_inv_system_ = Eigen::MatrixXd(inverse_system_order, 1);

	x0_qfilter_.setZero();
	x0_gsystem_.setZero();
	x0_inv_system_.setZero();


	// Compute the state-space model of Qfilter.
	Qfilter_ss_ = ns_control_toolbox::tf2ss(qfilter_data.TF, dt);
	// Qfilter_ss_.print();

	// Compute the state-space model of the system model G(s).
	// Gtf_.print();
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

void CommunicationDelayCompensatorCore::print() const
{
	ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
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
 * @brief Given an previous_input "u" that sent to the system and system response "y", computes four outputs by means of the
 * internal models for the filter Qf, for the system model G(s) for the Q-filtered inverse model.
 * [in] u : control previous_input for this system (delta:steering in lateral control).
 * [in] ym: measured response i.e ey, eyaw, edelta, eV.
 * [out] y0: u_filtered,Q(s)*u where u is the previous_input sent to the system.
 * [out] y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
 * [out] y2: du = y0 - y1 where du is the estimated disturbance previous_input
 * [out] y3: ydu = G(s)*du where ydu is the response of the system to du.
 * */

void CommunicationDelayCompensatorCore::simulateOneStep(double const& previous_input,
                                                        double const& measured_model_state,
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

		// ns_utils::print("previous_input  : ", previous_input, numkey, " : ", num_const_g);
	}

	if (num_den_constant_names_G_.second != "1")
	{
		auto&& denkey = num_den_constant_names_G_.second; // Corresponds to delta in ey model.
		auto&& den_const_g = pair_func_map_[denkey](num_den_args_of_G.second);

		Gtf_.update_den_coef(den_const_g);
		QGinv_tf_.update_num_coef(den_const_g); // since they are inverted.

		// ns_utils::print("previous_input  : ", previous_input, denkey, " : ", den_const_g);
	}

	// If any of num den constant changes, update the state-space models.
	if (num_den_constant_names_G_.first != "1" || num_den_constant_names_G_.second != "1")
	{
		// Update Gss accordingly from the TF version.
		Gss_.updateStateSpace(Gtf_);
		QGinv_ss_.updateStateSpace(QGinv_tf_);
	}

	// Simulate Qs, Gs, Qs/Gs/
	// set previous_input u of Q(s) to get the filtered output. u--> Q(s) -->uf
	// steering, gas sent to the vehicle.
	auto y0 = Qfilter_ss_.simulateOneStep(x0_qfilter_, previous_input); // Output is filtered previous_input uf.

	//	ns_utils::print("xuG before system: ");
	//	ns_eigen_utils::printEigenMat(x0_gsystem_);

	//  simulate y --> Q(s)/ G(s) --> u-du (original previous_input - disturbance previous_input).
	auto clamped_tracking_state = ns_utils::clamp(measured_model_state, -1.1, 1.1);
	// x0_inv_system_.setZero();
	auto y1 = QGinv_ss_.simulateOneStep(x0_inv_system_, clamped_tracking_state); // output is u-du.

	//	ns_utils::print("xuG after system: ");
	//	ns_eigen_utils::printEigenMat(x0_gsystem_);

	//	ns_utils::print("Current G Model");
	//	Gtf_.print();
	//	Gss_.print_discrete_system();

	// Get difference of uf-(u-du) ~=du
	auto&& du = y0 - y1;

	// Send du to the G(s) as the previous_input du --> G(s) --> dyu to obtain compensation signal.
	// x0_gsystem_.setZero();
	auto y3 = Gss_.simulateOneStep(x0_gsystem_, du); // output is y (i.e ey, eyaw, ...).


	ns_utils::print("Current velocity and steering :", num_den_args_of_G.first, previous_input);
	ns_utils::print("Current V^2 and cos(delta)^2  : ", pair_func_map_["v"](num_den_args_of_G.first),
	                pair_func_map_["delta"](num_den_args_of_G.second), "\n");

//	ns_utils::print("Current Q/G Model");
//	QGinv_tf_.print();
//	QGinv_ss_.print();
//	QGinv_ss_.print_discrete_system();


	// Get outputs.
	/**
	 * @brief Outputs of the delay compensator.
	 * y0: u_filtered,Q(s)*u where u is the previous_input sent to the system.
	 * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
	 * y2: du = y0 - y2 where du is the estimated disturbance previous_input
	 * y3: ydu = G(s)*du where ydu is the response of the system to du.
	 * */

	y_outputs[0] = y0; // ufiltered
	y_outputs[1] = y1; // u-du
	y_outputs[2] = du; // du
	y_outputs[3] = y3; // for time being y of u-->G(s)-->y


	// Get Ad_, Bd_, Cd_,Dd_ from QGinv_ss_.
	// getSSsystem(QGinv_ss_);

	// Print matrices.
	//	ns_eigen_utils::printEigenMat(Ad_, "Ad:");
	//	ns_eigen_utils::printEigenMat(Bd_, "Bd:");
	//	ns_eigen_utils::printEigenMat(Cd_, "Cd:");
	//	ns_eigen_utils::printEigenMat(Dd_, "Dd:");


}

void CommunicationDelayCompensatorCore::getSSsystem(ns_control_toolbox::tf2ss const& ss)
{

	Ad_ = ss.Ad();
	Bd_ = ss.B();
	Cd_ = ss.Cd();
	Dd_ = ss.Dd();

}