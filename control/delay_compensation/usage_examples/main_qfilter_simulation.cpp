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

#include "delay_compensator_core.hpp"
#include "qfilters.hpp"

int main()
{
	// To save and analyze the outputs.
	ns_utils::print(fs::current_path());
	ns_utils::print(fs::path("..") / "logs");
	fs::path output_path{ "../logs" };

	// Create a dummy output signal for ey, epsi and delta.
	double       tfinal{ 10. };     // signal length in time
	unsigned int frequency{ 40 };   // Hz
	double       dt{ 1. / frequency };

	Eigen::Index num_of_time_points = static_cast<Eigen::Index>(tfinal / dt ) + 1;
	auto         time_vector        = Eigen::VectorXd::LinSpaced(num_of_time_points, 0., tfinal);

	// a sinus signal
	double fhz_eysin = 0.1;
	double w_eysin   = 2 * M_PI * fhz_eysin; // [rad/sec]
	auto   ey_sin    = Eigen::VectorXd(time_vector.unaryExpr([&](auto const& t)
	{
		return sin(w_eysin * t);
	}));

	// Create an inverse vehicle model for these signal channels with Q-filters.
	// First create Q-filter for ey.
	double cut_off_frequency_ey = 20.; // [Hz]
	double time_constant_tau_ey = 0.1; // [sec] (time constant of the filter)

	const int order_ey    = 3;    // order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	const int order_e_yaw = 2;    // order of the filter for yaw error.



	// Base class construction.
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff{ cut_off_frequency_ey };
	QFilterBase                          qfilter_cutoff(sf_cutoff, order_ey, dt); // 3rd order low-pass filter


	StrongTypeDef<double, s_time_const_tag> stconstant{ time_constant_tau_ey };
	QFilterBase                             qfilter_tconst(stconstant, order_ey, dt); // 3rd order low-pass filter


	// Specialized qfilters for ey and eyaw.
	Qfilter<order_ey>    qfilter_ey{ sf_cutoff, order_ey, dt };
	Qfilter<order_e_yaw> qfilter_epsi{ sf_cutoff, order_e_yaw, dt };

#ifndef NDEBUG

	ns_eigen_utils::printEigenMat(time_vector.transpose());
	ns_eigen_utils::printEigenMat(ey_sin.transpose());

	// Print tf and transfer functions of the qfilters.
	ns_utils::print("Q-filter from Cutoff \n");
	qfilter_cutoff.print_tf();
	qfilter_cutoff.print_ss();

	ns_utils::print("Q-filter from TimeConstant \n");
	qfilter_tconst.print_tf();
	qfilter_tconst.print_ss();

	// Print Qfilter initial states from ey and epsi filters.
	ns_utils::print("Initial state of Qfilter ey : \n");
	qfilter_ey.print_x0();

	ns_utils::print("Setting Initial state of Qfilter epsi : \n");
	state_vector_qfilter<order_e_yaw> x0{ state_vector_qfilter<order_e_yaw>::Ones() };
	qfilter_epsi.set_initial_state_x0(x0);
	qfilter_epsi.print_x0();

	// Reseting an initial state.
	ns_utils::print("Re-setting Initial state of Qfilter epsi : \n");
	qfilter_epsi.reset_initial_state_x0();
	qfilter_epsi.print_x0();


	// Qfilter base class access;
	ns_utils::print("TF of ey : \n");
	qfilter_ey.print_tf();
	qfilter_ey.print_ss();

	ns_utils::print("TF of epsi : \n");
	qfilter_epsi.print_tf();
	qfilter_epsi.print_ss();


	// Test the dynamics of the filters.
	double u{ 1. };
	// dt = 1;

	auto yey   = qfilter_ey.y_hx(u);
	auto yepsi = qfilter_epsi.y_hx(u);

	ns_utils::print("Filter outputs ey, epsi : ", yey, "~", yepsi, "\n");

	// Simulate for the long input.
	auto ulong = ey_sin;
	qfilter_ey.reset_initial_state_x0();
	qfilter_epsi.reset_initial_state_x0();

	// Create a matrix to store and save the sim results.
	size_t          Nfinal{ 100 };
	Eigen::MatrixXd q_simresults_fromABCD(Nfinal, 3);
	Eigen::MatrixXd q_simresults_fromACT(Nfinal, 3);

	q_simresults_fromABCD.setZero();
	q_simresults_fromACT.setZero();

	Eigen::MatrixXd ey_xu(order_ey, 1); // [A, B;C D] matrix state
	Eigen::MatrixXd epsi_xu(order_e_yaw, 1); //

	ey_xu.setZero();
	epsi_xu.setZero();

	/**
	 * For autoware_control_toolbox sim, we need to give xu as a state of [A, B;C D] system.
	 * xu --> [A, B;C D] -- xy
	 **/

	for (auto k = 0; k < 50; ++k)
	{
		double& uk       = ulong(k);
		double&& yk_ey   = qfilter_ey.y_hx(uk);
		double&& yk_epsi = qfilter_epsi.y_hx(uk);

		q_simresults_fromABCD.row(k) << uk, yk_ey, yk_epsi;


		auto yey0   = qfilter_ey.simulateOneStep(ey_xu, uk);
		auto yeyaw0 = qfilter_epsi.simulateOneStep(epsi_xu, uk);

		q_simresults_fromACT.row(k) << uk, yey0, yeyaw0;


		ns_utils::print(" u, yey, yepsi : ", ulong(k), ",", yk_ey, ",", yk_epsi, "\n");
	}

	// Create and simulate an inverse vehicle model.
	/***
	 *   steering --> Vehicle Model --> ey, epsi, delta ---> Q * inverse vehicle model
	 *   ....  ey, epsi, delta ---> Q * inverse vehicle model --> ey, epsi, delta to subtract from refs.
	 * */

	writeToFile(output_path, q_simresults_fromABCD, "q_simresults_fromABCD");
	writeToFile(output_path, q_simresults_fromACT, "q_simresults_fromACT");

	auto pade0 = ns_control_toolbox::pade(0, 1);
	pade0.print();
	std::cout << "In the DEBUG mode ... " << std::endl;
#else
	std::cout << "In the RELEASE mode " << std::endl;
#endif


	return 0;
}