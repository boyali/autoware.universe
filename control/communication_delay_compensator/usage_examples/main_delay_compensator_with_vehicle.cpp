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
#include "qfilters.hpp"
#include "vehicle_models/vehicle_kinematic_error_model.hpp"
#include "utils_delay_observer/delay_compensation_utils.hpp"

namespace act = ns_control_toolbox;

int main()
{
	// Simulation parameters
	// Create a dummy output signal for ey, epsi and delta.
	fs::path output_path{ "../logs" };
	double tfinal{ 10. };     // signal length in time
	unsigned int frequency{ 40 };   // Hz
	double dt{ 1. / frequency };


	// Create an inverse vehicle model for these signal channels with Q-filters.
	// First create Q-filter for ey.
	double cut_off_frequency_ey = 8.; // [Hz]
	double cut_off_frequency_eyaw = 8.; // [Hz]
	double cut_off_frequency_delta = 8.;
	double cut_off_frequency_speed = 8.; // for longitudinal control.

	int const order_ey = 3;    // order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	int const order_e_yaw = 2;    // order of the filter for yaw error.
	int const order_delta = 1;    // order of stereing model.

	// Base class construction.
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_ey{ cut_off_frequency_ey };
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_eyaw{ cut_off_frequency_eyaw };
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_delta{ cut_off_frequency_delta };
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_speed{ cut_off_frequency_speed };


	// Specialized Qfilters for ey and eyaw.
	Qfilter<order_ey> qfilter_ey{ sf_cutoff_ey, order_ey, dt };
	Qfilter<order_e_yaw> qfilter_epsi{ sf_cutoff_eyaw, order_e_yaw, dt };
	Qfilter<order_delta> qfilter_delta{ sf_cutoff_delta, order_delta, dt };
	Qfilter<order_delta> qfilter_speed{ sf_cutoff_speed, order_delta, dt };


	/***
	 *   @brief 		Create and simulate an inverse vehicle model.
	 *   steering --> Vehicle Model --> ey, epsi, delta ---> Q * inverse vehicle model
	 *   ....  ey, epsi, delta ---> Q * inverse vehicle model --> ey, epsi, delta to subtract from refs.
	 * */

	s_filter_data qfilter_ey_data(qfilter_ey);
	s_filter_data qfilter_eyaw_data(qfilter_epsi);
	s_filter_data qfilter_delta_data(qfilter_delta);
	s_filter_data qfilter_speed_data(qfilter_speed);

	// Explicitly construct ey, eyaw, delta and V models.
	// ey model.
	/**
	 *  ey(s) = V^2 / (cos(delta)^2 * Ls^2 *(tau*s + 1))
	 * */

	double wheelbase{ 2.9 };
	double tau_vel{ 0.1 };
	double tau_steer{ 0.3 };
	double dead_time_steer{ 0.2 };
	double dead_time_vel{ 0.0 };

	// Create a linear vehicle model for the delay compensator.
	// Parameter names are the arguments of num den variable functions.
	act::tf_factor m_den1{{ wheelbase, 0, 0 }}; // L*s^2
	act::tf_factor m_den2{{ tau_steer, 1 }}; // (tau*s + 1)
	auto den_tf_factor = m_den1 * m_den2;

	//act::tf_t Gey({ 1. }, den_tf_factor(), 5., 2.);
	act::tf Gey({ 1. }, den_tf_factor(), 1., 1.); // num, den, num constant, den constant

	// We store the factored num and denominators:  a(var1) * num / b(var1)*den where num-den are constants.
	std::pair<std::string_view, std::string_view> param_names{ "v", "delta" };

	// Using unordered map to store functions.
	std::unordered_map<std::string_view, func_type<double>> f_variable_num_den_funcs{};

	// auto maplen = f_variable_num_den_funcs.size();
	f_variable_num_den_funcs["v"] = [](auto const& x) -> double
	{ return std::fabs(x) < 1. ? 1. : x * x; }; // to prevent zero division.

	f_variable_num_den_funcs["delta"] = [](auto const& x) -> double
	{ return std::cos(x) * std::cos(x); };

	// Store in a struct and pass to the delay compensator.
	s_model_g_data model_data(param_names, f_variable_num_den_funcs, Gey);

	// Create time-delay compensator for ey system.

	DelayCompensatorCore_PrototypeExample delay_compensator_ey(qfilter_ey_data, model_data, dt);
	delay_compensator_ey.print();

	// Create a nonlinear delayed vehicle model.
	// Generate a nonlinear vehicle vector.
	NonlinearVehicleKinematicModel nonlinear_model(wheelbase,
	                                               tau_vel, tau_steer,
	                                               dead_time_vel, dead_time_steer, dt);


	// Simulate the delay compensator.
	// Generate test signal
	auto time_vec = ns_control_toolbox::make_time_signal(dt, tfinal);

	// Control signals
	double signal_frequency{ 1. / (2. * M_PI) };
	auto vel_sqr_vec_input = ns_control_toolbox::make_square_signal(time_vec, signal_frequency);
	auto vel_trg_vec_input = ns_control_toolbox::make_triangle_signal(time_vec, 5);
	auto steer_sin_vec_input = ns_control_toolbox::make_sinus_signal(time_vec, signal_frequency);

	// Simulate the vehicle model.
	auto tsim_f = time_vec.rows();

	Eigen::MatrixXd sim_results_dc(tsim_f, 4);
	sim_results_dc.setZero();

	Eigen::MatrixXd sim_results_vh(tsim_f, 4);
	sim_results_vh.setZero();

	// State placeholder array
	std::array<double, 4> xnonlin_v{}; // ey, epsi, delta, V
	std::array<double, 4> y_ey{}; // output of delay compensator for ey.

	const double timer_dc_sim = ns_utils::tic();

	for (auto k = 1; k < tsim_f; ++k)
	{
		// Previous commands sent to the vehicle - To get a vehicle state
		auto uk_str_prev = steer_sin_vec_input(k - 1) * 0.1;
		auto uk_vel_prev = (vel_trg_vec_input(k - 1) + 1.) * 10.; // max(vel_.) is 1.

		// Current commands
		// auto uk_str_cur = steer_sin_vec_input(k) * 0.1;
		// auto uk_vel_cur = (vel_trg_vec_input(k) + 1.) * 10.; // max(vel_.) is 1.

		// Simulate Nonlinear Vehicle
		xnonlin_v = nonlinear_model.simulateOneStep(uk_vel_prev, uk_str_prev);

		auto measured_model_state = xnonlin_v[0]; // ey here
		auto steer_current = xnonlin_v[2];
		auto v_current = xnonlin_v[3];


		// Replace num den constant by the states [current vel, current steering states]
		std::pair<double, double> num_den_pairs_G{ v_current, steer_current };
		delay_compensator_ey.simulateOneStep(uk_str_prev,
		                                     measured_model_state,
		                                     num_den_pairs_G,
		                                     y_ey);

		sim_results_dc.row(k) = Eigen::Matrix<double, 1, 4>::Map(y_ey.data());
		sim_results_vh.row(k) = Eigen::Matrix<double, 1, 4>::Map(xnonlin_v.data());
	}
	ns_utils::print("Time for sim takes : ", ns_utils::toc(timer_dc_sim), " ms");

//	ns_utils::print("Simulation results for delay observer of ey");
	ns_eigen_utils::printEigenMat(sim_results_dc);
	writeToFile(output_path, sim_results_dc, "sim_results_dist_compensator_ey");
	writeToFile(output_path, sim_results_vh, "sim_results_nonlin_vehicle");
	writeToFile(output_path, vel_sqr_vec_input, "vel_trg_vec_input");
	writeToFile(output_path, steer_sin_vec_input, "steer_sin_vec_input");
	writeToFile(output_path, time_vec, "time_vec");

	std::cout << "In the DEBUG mode ... " << std::endl;

	return 0;
}
