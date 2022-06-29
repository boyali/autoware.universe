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

#ifndef COMMUNICATION_DELAY_COMPENSATOR__DELAY_OBSERVER_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__DELAY_OBSERVER_HPP

// Standard libraries.
#include <eigen3/Eigen/Core>

#include <cmath>
#include <iostream>
#include <unordered_map>
#include <utility>

// Autoware libs
#include "node_denifitions/node_definitions.hpp"
#include "vehicle_models/vehicle_kinematic_error_model.hpp"

#include "autoware_auto_vehicle_msgs/msg/delay_compensation_debug.hpp"
#include "autoware_auto_vehicle_msgs/msg/delay_compensation_refs.hpp"

namespace observers
{
using DelayCompensatatorMsg = autoware_auto_vehicle_msgs::msg::DelayCompensationRefs;
using DelayCompensatorDebugMsg = autoware_auto_vehicle_msgs::msg::DelayCompensationDebug;

struct sLyapMatrixVecs
{
  sLyapMatrixVecs()
  {
    vXs.reserve(cx_number_of_lyap_mats);
    vYs.reserve(cx_number_of_lyap_mats);
  }

  std::vector<state_matrix_observer_t> vXs;
  std::vector<input_matrix_observer_t> vYs;
};

/**
 * @brief Communication Delay Compensator Core without inverse models. It is an ordinary linear
 * observer model that estimates a slowly varying input disturbance. The time-delay in the input
 * channel is formulated as a disturbance in the form of:
 *
 *     u (s) - [d(s) = u(s)-u(s)e^{-sT}] = u(s)e^{-sT}
 *
 * */
class LateralCommunicationDelayCompensator
{
public:
  using model_ptr_t = std::shared_ptr<LinearKinematicErrorModel>;
  using state_qfilter = state_vector_qfilter<1>;  // @brief state vector for the filtered input

  LateralCommunicationDelayCompensator() = default;
  LateralCommunicationDelayCompensator(
    model_ptr_t vehicle_model, tf_t const & qfilter_lateral, sLyapMatrixVecs const & lyap_matsXY,
    float64_t const & dt);

  void printQfilterTFs() const;
  void printQfilterSSs() const;
  void printLyapMatrices() const;

  void simulateOneStep(
    state_vector_vehicle_t const & current_measurements, float64_t const & current_steering_cmd,
    std::shared_ptr<DelayCompensatatorMsg> & msg_compensation_results,
    std::shared_ptr<DelayCompensatorDebugMsg> & msg_debug_results);

  void setInitialStates();

private:
  model_ptr_t vehicle_model_ptr_{};

  // transfer functions
  tf_t tf_qfilter_lat_;

  // state-space models.
  ss_t ss_qfilter_lat_;

  // state vectors for filtering inputs.
  Eigen::MatrixXd xu0_;  // @brief state vector for the filtered input
  Eigen::MatrixXd xd0_;  // @brief state vector for the filtered disturbance

  /**
   * @brief state observer estimated state [ey, eyaw, steering, disturbance
   * */
  state_vector_observer_t xhat0_prev_;  // @brief state estimate at step [k-1]
  state_vector_observer_t xhat0_next_;  // @brief state estimate at step [k]

  // Lyapunov matrices to compute
  std::vector<state_matrix_observer_t> vXs_;
  std::vector<input_matrix_observer_t> vYs_;

  // placeholders
  input_matrix_observer_t Lobs_;          //@brief state observer gain matrix.
  state_vector_observer_t theta_params_;  //@brieff nonlinear terms in A of SS models of vehicle.

  // smaller size data class members.
  float64_t dt_{};
  float64_t previous_qfiltered_control_cmd_{};
  float64_t current_qfiltered_control_cmd_{};
  int qfilter_order_{1};
  bool8_t is_vehicle_initial_states_set_{false};

  /**
   * @brief computes the observer gain matrix given the operating conditions.
   * */
  void computeObserverGains();

  /**
   * @brief filters the control input and store it as previous_filtered_cmd.
   * */
  void qfilterControlCommand(float64_t const & current_control_cmd);

  /**
   * @brief estimates the vehicle states by the state observer.
   * */
  void estimateVehicleStates();
};

/**
 * @brief returns a transfer function in the form 1./(tau*s + 1)
 * @param w_cut_off_hz : cut-off frequency in Hertz.
 * */
tf_t get_nthOrderTF(float64_t const & w_cut_off_hz, int const & n);

/**
 * @brief returns a transfer function in the form 1./(tau*s + 1)^n_remaining * (damped tf)
 * @param w_cut_off_hz : cut-off frequency in Hertz.
 * @param remaining_order: the order after subtracting two which is the order of damped roots.
 * @param damping_val: damping value of the roots.
 * */
tf_t get_nthOrderTFwithDampedPoles(
  float64_t const & w_cut_off_hz, int const & remaining_order, float64_t const & damping_val);

}  // namespace observers

#endif  // COMMUNICATION_DELAY_COMPENSATOR__DELAY_OBSERVER_HPP