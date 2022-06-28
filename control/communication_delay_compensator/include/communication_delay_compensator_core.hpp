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

/**
 * @brief Communication Delay Compensator Core without inverse models. .
 * */

class CommunicationDelayCompensator
{
public:
  using model_ptr_t = std::shared_ptr<LinearKinematicErrorModel>;

  CommunicationDelayCompensator() = default;
  //  CommunicationDelayCompensator(
  //    model_ptr_t vehicle_model, tf_t const & qfilter_ey, tf_t const & qfilter_eyaw,
  //    tf_t const & qfilter_steering, float64_t const & dt);
  //
  //  void printQfilterTFs() const;
  //  void printQfilterSSs() const;
  //
  //  void simulateOneStep(
  //    state_vector_vehicle_t const & current_measurements, float64_t const & steering_cmd,
  //    std::shared_ptr<DelayCompensatatorMsg> & msg_compensation_results,
  //    std::shared_ptr<DelayCompensatorDebugMsg> & msg_debug_results);
  //
  //  void setInitialStates();

private:
  bool8_t is_vehicle_initial_states_set_{false};
  //  model_ptr_t vehicle_model_ptr_{};
  //
  //  // transfer functions
  //  tf_t tf_qfilter_ey_;
  //  tf_t tf_qfilter_eyaw_;
  //  tf_t tf_qfilter_steering_;
  //
  //  // state-space models.
  //  ss_t ss_qfilter_ey_;
  //  ss_t ss_qfilter_eyaw_;
  //  ss_t ss_qfilter_steering_;
  //
  //  // States and outputs for each qfilter vehicle model simulations u --> G(s)Q(s) --> y
  //  state_vector_vehicle_t x0_qey_{state_vector_vehicle_t::Zero()};
  //  state_vector_vehicle_t x0_qeyaw_{state_vector_vehicle_t::Zero()};
  //  state_vector_vehicle_t x0_qsteering_{state_vector_vehicle_t::Zero()};
  //
  //  // state vectors for filtering inputs.
  //  Eigen::MatrixXd xu0_ey_;
  //  Eigen::MatrixXd xu0_eyaw_;
  //  Eigen::MatrixXd xu0_steering_;
  //
  //  // state vectors for filtering output measurements.
  //  Eigen::MatrixXd xy0_ey_;
  //  Eigen::MatrixXd xy0_eyaw_;
  //  Eigen::MatrixXd xy0_steering_;
  //
  //  // placeholders
  //  state_vector_vehicle_t output_temp_{state_vector_vehicle_t ::Zero()};
  //
  //  float64_t dt_{};
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