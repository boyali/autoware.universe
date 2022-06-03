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


namespace act = ns_control_toolbox;

int main()
{
	// Create a dummy output signal for ey, epsi and delta.
	// double tfinal{ 10. };     // signal length in time
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


#ifndef NDEBUG

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

	double tau_steer{ 0.3 };
	double wheelbase{ 2.9 }; // L in vehicle model.
	std::pair<std::string_view, std::string_view> param_names{ "V^2,", "Cos(delta)^2" };

	act::tf_factor m_den1{{ wheelbase, 0, 0 }}; // L*s^2
	act::tf_factor m_den2{{ tau_steer, 1 }}; // (tau*s + 1)
	auto den_tf_factor = m_den1 * m_den2;

	//act::tf Gey({ 1. }, den_tf_factor(), 5., 2.);
	act::tf Gey({ 1. }, den_tf_factor(), 1., 1.);

	// Store in a struct and pass to the delay compensator.
	s_model_G_data model_data(param_names, Gey);

	// Create time-delay compensator for ey system.
	DelayCompensator delay_compensator_ey(qfilter_ey_data, model_data);
	delay_compensator_ey.print();

	// Simulate the delay compensator.
	// Generate test signal
	double tfinal{ 10. };
	auto time_vec = ns_control_toolbox::make_time_signal(dt, tfinal);

	// Control signals
	double control_frq{ 0.2 };
	auto vel_sqr_vec_input = ns_control_toolbox::make_square_signal(time_vec, control_frq);
	auto vel_trg_vec_input = ns_control_toolbox::make_triangle_signal(time_vec, 5);
	auto steer_sin_vec_input = ns_control_toolbox::make_sinus_signal(time_vec, 2 * control_frq);

	// Simulate the vehicle model.
	auto tsim_f = time_vec.rows();
	Eigen::MatrixXd sim_results(tsim_f, 2);
	sim_results.setZero();

	// State placeholder array
	double y_ey{};

	for (auto k = 0; k < tsim_f; ++k)
	{
		double desired_vel = vel_trg_vec_input(k) * 10.; // max(vel_.) is 1.
		double desired_steer = steer_sin_vec_input(k) * 0.1;
		// x = nonlinear_model.simulateOneStep(desired_vel, desired_steer);

		sim_results.row(k) = Eigen::Matrix<double, 1, 4>::Map(x.data());
	}


	std::cout << "In the DEBUG mode ... " << std::endl;
#else
	std::cout << "In the RELEASE mode " << std::endl;
#endif


	return 0;
}
