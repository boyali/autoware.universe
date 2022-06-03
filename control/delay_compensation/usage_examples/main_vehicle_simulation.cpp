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

#include "delay_observer.hpp"
#include "qfilters.hpp"
#include "vehicle_models/vehicle_kinematic_error_model.hpp"
#include "utils_delay_observer/delay_compensation_utils.hpp"

int main()
{
	// Create a dummy output signal for ey, epsi and delta.
	double tfinal{ 10. };     // signal length in time
	unsigned int frequency{ 40 };   // Hz
	double dt{ 1. / frequency };


	// Create an inverse vehicle model for these signal channels with Q-filters.
	// First create Q-filter for ey.
	double cut_off_frequency_ey = 20.; // [Hz]
	double cut_off_frequency_eyaw = 15.; // [Hz]
	double cut_off_frequency_delta = 10.;
	double cut_off_frequency_speed = 10.; // for longitudinal control.


	int const order_ey = 3;    // order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	int const order_e_yaw = 2;    // order of the filter for yaw error.
	int const order_delta = 1;    // order of stereing model.


	// Base class construction.
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_ey{ cut_off_frequency_ey };
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_eyaw{ cut_off_frequency_eyaw };
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_delta{ cut_off_frequency_delta };
	StrongTypeDef<double, s_cut_off_tag> sf_cutoff_speed{ cut_off_frequency_speed };


	// Specialized Qfilters for ey and eyaw.
	Qfilter<state_vector_qfilter<order_ey>> qfilter_ey{ sf_cutoff_ey, order_ey, dt };
	Qfilter<state_vector_qfilter<order_e_yaw>> qfilter_epsi{ sf_cutoff_eyaw, order_e_yaw, dt };
	Qfilter<state_vector_qfilter<order_delta>> qfilter_delta{ sf_cutoff_delta, order_delta, dt };
	Qfilter<state_vector_qfilter<order_delta>> qfilter_speed{ sf_cutoff_speed, order_delta, dt };

	// Create a nonlinear vehicle model.
	double wheelbase{ 2.9 };
	double tau_vel{ 0.1 };
	double tau_steer{ 0.24 };
	double dead_time_steer{ 0.0 };
	double dead_time_vel{ 1.0 };

	// Generate test signal
	auto time_vec = ns_control_toolbox::make_time_signal(dt, tfinal);
//	ns_eigen_utils::printEigenMat(time_vec);

	// Control signals
	double control_frq{ 0.2 };
	auto vel_sqr_vec_input = ns_control_toolbox::make_square_signal(time_vec, control_frq);
	auto vel_trg_vec_input = ns_control_toolbox::make_triangle_signal(time_vec, 5);
	auto steer_sin_vec_input = ns_control_toolbox::make_sinus_signal(time_vec, 2 * control_frq);


	// Generate vehicle vector.
	NonlinearVehicleKinematicModel nonlinear_model(wheelbase,
	                                               tau_vel, tau_steer,
	                                               dead_time_vel, dead_time_steer, dt);

//		auto file_path_to_text = getOutputPath();
//		ns_utils::print(file_path_to_text.c_str());
	ns_utils::print(fs::current_path());
	ns_utils::print(fs::path("..") / "logs");

	fs::path output_path{ "../logs" };
	writeToFile(output_path, vel_trg_vec_input, "vel_trg_vec_input");
	writeToFile(output_path, vel_sqr_vec_input, "vel_sqr_vec_input");
	writeToFile(output_path, steer_sin_vec_input, "steer_sin_vec_input");
	writeToFile(output_path, time_vec, "time_vec");

	// Simulate the vehicle model.
	auto tsim_f = time_vec.rows();
	Eigen::MatrixXd sim_results(tsim_f, 4);
	sim_results.setZero();

	// State placeholder array
	std::array<double, 4> x{};

	for (auto k = 0; k < tsim_f; ++k)
	{
		double desired_vel = vel_trg_vec_input(k) * 10.; // max(vel_.) is 1.
		double desired_steer = steer_sin_vec_input(k) * 0.1;
		x = nonlinear_model.simulateOneStep(desired_vel, desired_steer);

		sim_results.row(k) = Eigen::Matrix<double, 1, 4>::Map(x.data());
	}

	ns_eigen_utils::printEigenMat(sim_results);
	writeToFile(output_path, sim_results, "sim_results_with_delay");

#ifndef NDEBUG


	// Create and simulate an inverse vehicle model.
	/***
	 *   steering --> Vehicle Model --> ey, epsi, delta ---> Q * inverse vehicle model
	 *   ....  ey, epsi, delta ---> Q * inverse vehicle model --> ey, epsi, delta to subtract from refs.
	 * */

	s_filter_data qfilter_ey_data(qfilter_ey);
	s_filter_data qfilter_eyaw_data(qfilter_epsi);
	s_filter_data qfilter_delta_data(qfilter_delta);
	s_filter_data qfilter_speed_data(qfilter_speed);

	// Explicitly construct ey, eyaw, delta and V models.


	std::cout << "In the DEBUG mode ... " << std::endl;
#else
	std::cout << "In the RELEASE mode " << std::endl;
#endif


	return 0;
}