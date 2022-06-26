
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

#include <utility>

// ------------------- Communication Delay Compensator using Forward Dynamics. ---------
observers::CommunicationDelayCompensatorForward::CommunicationDelayCompensatorForward(
  observers::CommunicationDelayCompensatorForward::model_ptr_t vehicle_model,
  const observers::tf_t & qfilter_ey, const observers::tf_t & qfilter_eyaw,
  const observers::tf_t & qfilter_steering, float64_t const & dt)
: vehicle_model_ptr_(std::move(vehicle_model)),
  tf_qfilter_ey_{qfilter_ey},
  tf_qfilter_eyaw_{qfilter_eyaw},
  tf_qfilter_steering_{qfilter_steering},
  dt_{dt}
{
  // Compute the state-space model of QGinv(s)
  ss_qfilter_ey_ = ss_t(qfilter_ey, dt_);  // Do not forget to enter the time step dt.
  ss_qfilter_eyaw_ = ss_t(qfilter_eyaw, dt_);
  ss_qfilter_steering_ = ss_t(qfilter_steering, dt_);

  auto order_ey = tf_qfilter_ey_.order();
  auto order_eyaw = tf_qfilter_eyaw_.order();
  auto order_steering = tf_qfilter_ey_.order();

  xu0_ey_ = Eigen::MatrixXd(order_ey, 1);
  xu0_eyaw_ = Eigen::MatrixXd(order_eyaw, 1);
  xu0_steering_ = Eigen::MatrixXd(order_steering, 1);

  xy0_ey_ = Eigen::MatrixXd(order_ey, 1);
  xy0_eyaw_ = Eigen::MatrixXd(order_eyaw, 1);
  xy0_steering_ = Eigen::MatrixXd(order_steering, 1);

  // Set all to zero
  xu0_ey_.setZero();
  xu0_eyaw_.setZero();
  xu0_steering_.setZero();

  xy0_ey_.setZero();
  xy0_eyaw_.setZero();
  xy0_steering_.setZero();
}

void observers::CommunicationDelayCompensatorForward::printQfilterTFs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("Transfer function of qfilter of lateral error : \n");
  tf_qfilter_ey_.print();

  ns_utils::print("Transfer function of qfilter of heading error : \n");
  tf_qfilter_eyaw_.print();

  ns_utils::print("Transfer function of qfilter of steering state : \n");
  tf_qfilter_steering_.print();
}

void observers::CommunicationDelayCompensatorForward::printQfilterSSs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("State-Space Matrices of qfilter of lateral error : \n");
  ss_qfilter_ey_.print();

  ns_utils::print("State-Space Matrices of qfilter of heading error : \n");
  ss_qfilter_eyaw_.print();

  ns_utils::print("State-Space Matrices of qfilter of steering state : \n");
  ss_qfilter_steering_.print();
}
void observers::CommunicationDelayCompensatorForward::simulateOneStep(
  const state_vector_vehicle_t & current_measurements, float64_t const & steering_cmd,
  std::shared_ptr<DelayCompensatatorMsg> & msg_compensation_results,
  std::shared_ptr<DelayCompensatorDebugMsg> & msg_debug_results)
{
  setInitialStates();  // sets once.

  // Q(s)*u(s) filtered inputs.
  auto const & uf_ey = ss_qfilter_ey_.simulateOneStep(xu0_ey_, steering_cmd);
  auto const & uf_eyaw = ss_qfilter_eyaw_.simulateOneStep(xu0_eyaw_, steering_cmd);
  auto const & uf_steering = ss_qfilter_eyaw_.simulateOneStep(xu0_eyaw_, steering_cmd);

  // Q filter outputs.
  auto const & measured_ey = current_measurements(0, 0);
  auto const & measured_eyaw = current_measurements(1, 0);
  auto const & measured_steering = current_measurements(2, 0);

  // Q(s)y(s)
  auto yf_ey = ss_qfilter_ey_.simulateOneStep(xy0_ey_, measured_ey);
  auto yf_eyaw = ss_qfilter_eyaw_.simulateOneStep(xy0_eyaw_, measured_eyaw);
  auto yf_steering = ss_qfilter_eyaw_.simulateOneStep(xy0_eyaw_, measured_steering);

  // Simulate the vehicle outputs. G(s)Q(s)u
  vehicle_model_ptr_->simulateOneStep(output_temp_, x0_qey_, uf_ey);
  auto const vec_ey_output = output_temp_(0, 0);

  vehicle_model_ptr_->simulateOneStep(output_temp_, x0_qeyaw_, uf_eyaw);
  auto const vec_eyaw_output = output_temp_(1, 0);

  vehicle_model_ptr_->simulateOneStep(output_temp_, x0_qsteering_, uf_steering);
  auto const vec_steering_output = output_temp_(2, 0);

  // Set the message values.
  msg_debug_results->lat_uf = uf_ey;
  msg_debug_results->lat_yu = vec_ey_output + measured_ey - yf_ey;
  msg_compensation_results->lateral_deviation_error_compensation_ref = msg_debug_results->lat_yu;

  msg_debug_results->heading_uf = uf_eyaw;
  msg_debug_results->heading_yu = vec_eyaw_output + measured_eyaw - yf_eyaw;
  msg_compensation_results->heading_angle_error_compensation_ref = msg_debug_results->heading_yu;

  msg_debug_results->steering_uf = uf_steering;
  msg_debug_results->steering_yu = vec_steering_output + measured_steering - yf_steering;
  msg_compensation_results->steering_error_compensation_ref = msg_debug_results->steering_yu;

  // Send to the vehicle model.
  // Q filter measurements.
  // Sum the signals to compute the new references for the controllers.

  ns_utils::print("Filtered inputs :", uf_ey, uf_eyaw, uf_steering);
  ns_utils::print("Filtered outputs :", yf_ey, yf_eyaw, yf_steering);

  ns_utils::print("And output temp: ");
  ns_eigen_utils::printEigenMat(output_temp_);

  ns_utils::print("And x0_qey temp: ");
  ns_eigen_utils::printEigenMat(x0_qey_);

  ns_eigen_utils::printEigenMat(current_measurements);
}
void observers::CommunicationDelayCompensatorForward::setInitialStates()
{
  //  if (!is_vehicle_initial_states_set_) {
  if (vehicle_model_ptr_->areInitialStatesSet()) {
    x0_qey_ = vehicle_model_ptr_->getInitialStates();
    x0_qeyaw_ = x0_qey_;
    x0_qsteering_ = x0_qey_;
    is_vehicle_initial_states_set_ = true;
    //    }
  }
}

observers::tf_t observers::get_nthOrderTF(
  const autoware::common::types::float64_t & w_cut_off_hz, const int & n)
{
  auto && wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

  // float64_t time_constant_of_qfilter{};
  auto tau = 1.0 / wc_rad_sec;

  // --------------- Qfilter Construction --------------------------------------
  // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
  // Calculate the transfer function.
  ns_control_toolbox::tf_factor denominator{std::vector<double>{tau, 1.}};  // (tau*s+1)

  // Take power of the denominator.
  denominator.power(static_cast<unsigned int>(n));

  // Create the transfer function from a numerator an denominator.
  auto const && q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

  return q_tf;
}

observers::tf_t observers::get_nthOrderTFwithDampedPoles(
  const autoware::common::types::float64_t & w_cut_off_hz, const int & remaining_order,
  const autoware::common::types::float64_t & damping_val)
{
  auto && wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

  // float64_t time_constant_of_qfilter{};
  auto tau = 1.0 / wc_rad_sec;

  // A second order damped transfer function.
  auto q_tf_damped = tf_t({1.}, {tau * tau, 2 * damping_val * tau, 1.});

  if (remaining_order > 0) {
    auto q_remaining_tf = get_nthOrderTF(w_cut_off_hz, remaining_order);
    q_tf_damped = q_tf_damped * q_remaining_tf;
  }

  return q_tf_damped;
}