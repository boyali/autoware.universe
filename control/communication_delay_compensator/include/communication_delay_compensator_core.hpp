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

class CommunicationDelayCompensatorForward
{
public:
  using model_ptr_t = std::shared_ptr<LinearKinematicErrorModel>;

  CommunicationDelayCompensatorForward() = default;
  CommunicationDelayCompensatorForward(
    model_ptr_t vehicle_model, tf_t const & qfilter_ey, tf_t const & qfilter_eyaw,
    tf_t const & qfilter_steering, float64_t const & dt);

  void printQfilterTFs() const;
  void printQfilterSSs() const;

  void simulateOneStep(
    state_vector_vehicle_t const & current_measurements, float64_t const & steering_cmd,
    std::shared_ptr<DelayCompensatatorMsg> & msg_compensation_results,
    std::shared_ptr<DelayCompensatorDebugMsg> & msg_debug_results);

  void setInitialStates();

private:
  bool8_t is_vehicle_initial_states_set_{false};
  model_ptr_t vehicle_model_ptr_{};

  // transfer functions
  tf_t tf_qfilter_ey_;
  tf_t tf_qfilter_eyaw_;
  tf_t tf_qfilter_steering_;

  // state-space models.
  ss_t ss_qfilter_ey_;
  ss_t ss_qfilter_eyaw_;
  ss_t ss_qfilter_steering_;

  // States and outputs for each qfilter vehicle model simulations u --> G(s)Q(s) --> y
  state_vector_vehicle_t x0_qey_{state_vector_vehicle_t::Zero()};
  state_vector_vehicle_t x0_qeyaw_{state_vector_vehicle_t::Zero()};
  state_vector_vehicle_t x0_qsteering_{state_vector_vehicle_t::Zero()};

  // state vectors for filtering inputs.
  Eigen::MatrixXd xu0_ey_;
  Eigen::MatrixXd xu0_eyaw_;
  Eigen::MatrixXd xu0_steering_;

  // state vectors for filtering output measurements.
  Eigen::MatrixXd xy0_ey_;
  Eigen::MatrixXd xy0_eyaw_;
  Eigen::MatrixXd xy0_steering_;

  // placeholders
  state_vector_vehicle_t output_temp_{state_vector_vehicle_t ::Zero()};

  float64_t dt_{};
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

/// -------------- AS A PROTOTYPE NOT USED in the NODE ---------------------------------

// class __attribute__((__visibility__("default"))) DelayCompensatorCore_PrototypeExample
// For internal states use state_T, for ABCD matrices use mat_eig_T.
class DelayCompensatorCore_PrototypeExample
{
public:
  using pairs_t = s_model_g_data::pairs_t;
  using pairs_func_maps_t = s_model_g_data::pairs_func_maps_t;

  // Constructors. s_ for struct.
  DelayCompensatorCore_PrototypeExample() = default;

  DelayCompensatorCore_PrototypeExample(
    s_filter_data const & qfilter_data, s_model_g_data & model_data, double const & dt);

  void print() const;

  // Simulate  one-step and get the outputs.
  /**
   * @brief Simulates one-step and produces the outputs.
   * @param previous_input: previous input sent to the vehicle
   * @param measured_model_state: the state that tracks a reference (i.e ey, eyaw, eV).
   * @param num_den_args_of_g: the model states that define the transfer function, i.e
   * (V^2/(cos(delta)*s + 1).
   * @param outputs from the delay compensator.
   * */
  void simulateOneStep(
    double const & previous_input, /** previous input*/
    double const & measured_model_state,
    std::pair<double, double> const & num_den_args_of_g, /** model parameters*/
    std::array<double, 5> & y_outputs);

private:
  double dt_{0.1};

  // Associated Qfilter parameters
  int q_order_{1};  // @brief order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
  double q_cut_off_frequency_{};  // @brief cut-off frequency in Hz.
  double q_time_constant_tau_{};

  // Qfilter transfer function.
  ns_control_toolbox::tf Qfilter_tf_{};     // @brief Transfer function of the qfilter.
  ns_control_toolbox::tf2ss Qfilter_ss_{};  //@brief State space model of the qfilter

  /**
   * @brief  Associated model parameters as multiplication factors for num and den of G and Q/G.
   * i.e G(s) = f(V) num/ (g(delta) den). V and delta are the arguments stored in the pairs_t.
   * */
  pairs_t num_den_constant_names_g_{"1", "1"};
  pairs_t num_den_constant_names_QGinv_{"1", "1"};

  // and their functions.
  pairs_func_maps_t pair_func_map_{};

  // Model transfer function.
  ns_control_toolbox::tf Gtf_{};
  ns_control_toolbox::tf2ss Gss_;

  // Q(s)/G(s)
  ns_control_toolbox::tf QGinv_tf_{};
  ns_control_toolbox::tf2ss QGinv_ss_{};

  // Internal states.
  Eigen::MatrixXd x0_qfilter_{};     // u--> Q(s) -->ufiltered
  Eigen::MatrixXd x0_gsystem_{};     // u--> G(s)-->y (i,e ey, eyaw ..)
  Eigen::MatrixXd x0_inv_system_{};  // y--> Q(s)/G(s) --> u-du

  // Placeholders for Ad, Bd, Cd, Dd.
  Eigen::MatrixXd Ad_{};
  Eigen::MatrixXd Bd_{};
  Eigen::MatrixXd Cd_{};
  Eigen::MatrixXd Dd_{};

  /**
   * @brief Outputs of the delay compensator.
   * y0: u_filtered,Q(s)*u where u is the input sent to the system.
   * y2: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
   * y2: du = y0 - y1 where du is the estimated disturbance input
   * y3: ydu = G(s)*du where ydu is the response of the system to du.
   * */
  std::array<double, 4> y_outputs_{};

  // Member functions.
  void getSSsystem(ns_control_toolbox::tf2ss const & ss);
};

#endif  // COMMUNICATION_DELAY_COMPENSATOR__DELAY_OBSERVER_HPP