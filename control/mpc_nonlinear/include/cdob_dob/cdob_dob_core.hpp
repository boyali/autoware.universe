/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_DOB_CORE_HPP_
#define MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_DOB_CORE_HPP_

#include <eigen3/Eigen/Core>
#include <Eigen/StdVector>

#include <cmath>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <memory>

#include "autoware_control_toolbox.hpp"
#include "cdob_dob/cdob_dob_definitions.hpp"
#include "cdob_dob/cdob_models.hpp"
#include "utils/nmpc_utils_eigen.hpp"

namespace ns_cdob
{

struct sLyapMatrixVecs
{
	sLyapMatrixVecs()
	{
		vXs.reserve(cx_NUMBER_OF_LYAP_MATS);
		vYs.reserve(cx_NUMBER_OF_LYAP_MATS);
	}

	std::vector<state_matrix_observer_t> vXs;
	std::vector<measurement_matrix_observer_t> vYs;
};

using tf_t = ns_control_toolbox::tf;
using ss_t = ns_control_toolbox::tf2ss;

class LateralCommunicationDelayCompensator_CDOB
{
 public:
	using obs_model_ptr_t = std::shared_ptr<linear_state_observer_model_t>;
	using model_ptr_t = std::shared_ptr<linear_vehicle_model_t>;
	using state_qfilter = state_vector_qfilter<1>;  // @brief state vector for the filtered input

	LateralCommunicationDelayCompensator_CDOB() = default;

	LateralCommunicationDelayCompensator_CDOB(obs_model_ptr_t observer_vehicle_model,
																						model_ptr_t vehicle_model,
																						tf_t const &qfilter_lateral,
																						sLyapMatrixVecs const &lyap_matsXY,
																						double const &dt);

	void printQfilterTFs() const;

	void printQfilterSSs() const;

	void printLyapMatrices() const;

	void simulateOneStep(state_vector_vehicle_t const &current_measurements,
											 double const &prev_steering_control_cmd,
											 double const &current_steering_cmd,
											 std::array<double, 8> &results);

	void setInitialStates();

 private:
	obs_model_ptr_t observer_vehicle_model_ptr_{nullptr}; // state observer model
	model_ptr_t vehicle_model_ptr_{nullptr}; // vehicle model

	// transfer functions
	tf_t tf_qfilter_lat_;

	// state-space models.
	ss_t ss_qfilter_lat_;

	// ------------ QFILTER VARIABLES ----------------------------------
	// state vectors for filtering inputs.
	Eigen::MatrixXd xu0_;  // @brief state vector for the filtered input
	Eigen::MatrixXd xd0_;  // @brief state vector for the filtered disturbance
	double df_d0_{}; // q-filtered disturbance response

	// ------------ OBSERVER VARIABLES ----------------------------------
	/**
	 * @brief state observer estimated state [ey, eyaw, steering, disturbance
	 * */
	// @brief state estimate at step [k-1]
	state_vector_observer_t xhat0_prev_{state_vector_observer_t::Zero()};

	//@brief estimated vehicle states, disturbance row is zero (cannot observed)
	state_vector_vehicle_t current_yobs_{state_vector_vehicle_t::Zero()};

	// @brief temporary variables
	state_vector_observer_t xbar_temp_{state_vector_observer_t::Zero()};
	state_vector_vehicle_t ybar_temp_{state_vector_vehicle_t::Zero()};

	// -------------- VEHICLE MODEL VARIABLES ----------------------------
	state_vector_vehicle_t xv_d0_{state_vector_vehicle_t::Zero()}; // states for disturbance input simulations
	state_vector_vehicle_t yv_d0_{state_vector_vehicle_t::Zero()}; // response for disturbance input simulations

	// Lyapunov matrices to compute
	std::vector<state_matrix_observer_t> vXs_;
	std::vector<measurement_matrix_observer_t> vYs_;

	// placeholders
	measurement_matrix_observer_t Lobs_{measurement_matrix_observer_t::Zero()}; //@brief state observer gain matrix.
	state_vector_observer_t
		theta_params_{state_vector_observer_t::Zero()};  //@brieff nonlinear terms in A of SS models of vehicle.

	// smaller size data class members.
	double dt_{};
	double current_qfiltered_control_cmd_{};
	double prev_qfiltered_control_cmd_{};
	int qfilter_order_{1};
	bool is_observer_model_initial_states_set_{false};

	/**
	 * @brief computes the observer gain matrix given the operating conditions.
	 * */
	void computeObserverGains(const state_vector_vehicle_t &current_measurements);

	/**
	 * @brief filters the control input and store it as previous_filtered_cmd.
	 * */
	void qfilterControlCommand(double const &current_control_cmd);

	/**
	 * @brief estimates the vehicle states by the state observer.
	 * */
	void estimateVehicleStates(const state_vector_vehicle_t &current_measurements,
														 double const &prev_steering_control_cmd,
														 double const &current_steering_cmd);

};

/**
 * @brief Communication Delay Compensator Core without inverse models. It is an ordinary linear
 * observer model that estimates a slowly varying input disturbance. The time-delay in the input
 * channel is formulated as a disturbance in the form of:
 *
 *     u (s) - [d(s) = u(s)-u(s)e^{-sT}] = u(s)e^{-sT}
 *
 * */
class LateralDisturbanceCompensator_DOB
{
 public:
	using obs_model_ptr_t = std::shared_ptr<linear_state_observer_model_t>;
	using state_qfilter = state_vector_qfilter<1>;  // @brief state vector for the filtered input

	LateralDisturbanceCompensator_DOB() = default;

	LateralDisturbanceCompensator_DOB(obs_model_ptr_t observer_vehicle_model,
																		tf_t const &qfilter_lateral,
																		sLyapMatrixVecs const &lyap_matsXY,
																		double const &dt);

	void printQfilterTFs() const;

	void printQfilterSSs() const;

	void printLyapMatrices() const;

	void simulateOneStep(state_vector_vehicle_t const &current_measurements,
											 double const &prev_steering_control_cmd,
											 double const &current_steering_cmd,
											 double &curvature_compensation_input);

	void setInitialStates();

 private:
	obs_model_ptr_t observer_vehicle_model_ptr_{nullptr}; // state observer model

	// transfer functions
	tf_t tf_qfilter_lat_;

	// state-space models.
	ss_t ss_qfilter_lat_;

	// ------------ QFILTER VARIABLES ----------------------------------
	// state vectors for filtering inputs.
	Eigen::MatrixXd xu0_;  // @brief state vector for the filtered input
	Eigen::MatrixXd xd0_;  // @brief state vector for the filtered input
	double dist_input_{};

	Eigen::MatrixXd x_yvd0_;  // @brief state vector for the filtered disturbance
	Eigen::MatrixXd x_yvd1_;  // @brief state vector for the filtered disturbance
	Eigen::MatrixXd x_yvd2_;  // @brief state vector for the filtered disturbance


	// ------------ OBSERVER VARIABLES ----------------------------------
	/**
	 * @brief state observer estimated state [ey, eyaw, steering, disturbance
	 * */
	// @brief state estimate at step [k-1]
	state_vector_observer_t xhat0_prev_{state_vector_observer_t::Zero()};

	//@brief estimated vehicle states, disturbance row is zero (cannot observed)
	/**
	 * @brief this observed outputs: current_yobs_ does not contain disturbance information. The last row of this
	 * vector will always be zero. That is why we apply q-filter to get some output.
	 * */
	state_vector_vehicle_t current_yobs_{state_vector_vehicle_t::Zero()};

	// -------------- VEHICLE MODEL VARIABLES ----------------------------
	state_vector_vehicle_t x_yv_d0_{state_vector_vehicle_t::Zero()}; // time-delay obs outputs:y
	state_vector_vehicle_t yv_td0_{state_vector_vehicle_t::Zero()}; // time-delay obs outputs:y
	state_vector_vehicle_t yv_f0_{state_vector_vehicle_t::Zero()}; // filtered outputs.
	double current_ideal_steering_{};

	// simulations
	// @brief temporary variables
	state_vector_observer_t xbar_temp_{state_vector_observer_t::Zero()}; // states of the state observer
	state_vector_vehicle_t ybar_temp_{state_vector_vehicle_t::Zero()}; // outputs of the state observer

	// Lyapunov matrices to compute
	std::vector<state_matrix_observer_t> vXs_;
	std::vector<measurement_matrix_observer_t> vYs_;

	// placeholders
	measurement_matrix_observer_t Lobs_{measurement_matrix_observer_t::Zero()};    //@brief state observer gain matrix.
	state_vector_observer_t
		theta_params_{state_vector_observer_t::Zero()};  //@brief nonlinear terms in A of SS models of vehicle.

	// smaller size data class members.
	double dt_{};
	double current_qfiltered_control_cmd_{};
	double prev_qfiltered_control_cmd_{};
	int qfilter_order_{1};
	bool is_vehicle_initial_states_set_{false};

	/**
	 * @brief computes the observer gain matrix given the operating conditions.
	 * */
	void computeObserverGains(const state_vector_vehicle_t &current_measurements);

	/**
	 * @brief filters the control input and store it as previous_filtered_cmd.
	 * */
	void qfilterControlCommand(double const &current_control_cmd);

	/**
	 * @brief estimates the vehicle states by the state observer.
	 * */
	void estimateVehicleStates(const state_vector_vehicle_t &current_measurements,
														 double const &prev_steering_control_cmd,
														 double const &current_steering_cmd);

};

/**
 * @brief returns a transfer function in the form 1./(tau*s + 1)
 * @param w_cut_off_hz : cut-off frequency in Hertz.
 * */
tf_t get_nthOrderTF(double const &w_cut_off_hz, int const &n);

/**
 * @brief returns a transfer function in the form 1./(tau*s + 1)^n_remaining * (damped tf)
 * @param w_cut_off_hz : cut-off frequency in Hertz.
 * @param remaining_order: the order after subtracting two which is the order of damped roots.
 * @param damping_val: damping value of the roots.
 * */
tf_t get_nthOrderTFwithDampedPoles(double const &w_cut_off_hz,
																	 int const &remaining_order,
																	 double const &damping_val);

} // namespace ns_cdob
#endif //MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_DOB_CORE_HPP_
