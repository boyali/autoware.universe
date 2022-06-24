
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

/**
 * @brief Communication Delay Compensator Core.
 * */

/// -------------- AS A PROTOTYPE NOT USED in the NODE ---------------------------------
DelayCompensatorCore_PrototypeExample::DelayCompensatorCore_PrototypeExample(
  s_filter_data const & qfilter_data, s_model_g_data & model_data, double const & dt)
: dt_{dt},
  q_order_{qfilter_data.order},
  q_cut_off_frequency_{qfilter_data.cut_off_frq},
  q_time_constant_tau_{qfilter_data.time_constant},
  Qfilter_tf_(qfilter_data.TF),
  num_den_constant_names_g_{model_data.num_den_coeff_names},
  pair_func_map_{model_data.funcs},
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
  num_den_constant_names_QGinv_ =
    pairs_t(num_den_constant_names_g_.second, num_den_constant_names_g_.first);
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

void DelayCompensatorCore_PrototypeExample::print() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("Qfilter Model : \n");
  Qfilter_tf_.print();

  ns_utils::print("Forward model G(s) : \n\n");
  Gtf_.print();

  ns_utils::print(
    "G(s) num and den constant names :", num_den_constant_names_g_.first,
    num_den_constant_names_g_.second, "\n");

  ns_utils::print("Forward model Q/G(s) :  \n");
  QGinv_tf_.print();

  ns_utils::print(
    "Q/G(s) num and den constant names :", num_den_constant_names_QGinv_.first,
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
 * @brief Given an previous_input "u" that sent to the system and system response "y", computes four
 * outputs by means of the internal models for the filter Qf, for the system model G(s) for the
 * Q-filtered inverse model. [in] u : control previous_input for this system (delta:steering in
 * lateral control). [in] ym: measured response i.e ey, eyaw, edelta, eV. [out] y0:
 * u_filtered,Q(s)*u where u is the previous_input sent to the system. [out] y1: u-d_u =
 * (Q(s)/G(s))*y_system where y_system is the measured system response. [out] y2: du = y0 - y1 where
 * du is the estimated disturbance previous_input [out] y3: ydu = G(s)*du where ydu is the response
 * of the system to du.
 * */

void DelayCompensatorCore_PrototypeExample::simulateOneStep(
  double const & previous_input, double const & measured_model_state,
  std::pair<double, double> const & num_den_args_of_g, std::array<double, 5> & y_outputs)
{
  // Get the output of num_constant for the G(s) = nconstant(x) * num / (denconstant(y) * den)
  if (num_den_constant_names_g_.first != "1") {
    auto && numkey = num_den_constant_names_g_.first;  // Corresponds to v in ey model.

    // Find the function of numerator variable and apply its function on it.
    auto && num_const_g = pair_func_map_[numkey](num_den_args_of_g.first);

    // Update numerator of G(s) and denominator of QGinv
    Gtf_.update_num_coef(num_const_g);       // update numerator
    QGinv_tf_.update_den_coef(num_const_g);  // update denominator - since they are inverted.

    // ns_utils::print("previous_input  : ", previous_input, numkey, " : ", num_const_g);
  }

  // if there exists a denominator variable
  if (num_den_constant_names_g_.second != "1") {
    // Get the name of denominator variable
    auto && denkey = num_den_constant_names_g_.second;  // Corresponds to delta in ey model.

    // Apply its function
    auto && den_const_g = pair_func_map_[denkey](num_den_args_of_g.second);

    Gtf_.update_den_coef(den_const_g);       // Update denominator of vehicle
    QGinv_tf_.update_num_coef(den_const_g);  // Update the numerator - since they are inverted.

    // ns_utils::print("previous_input  : ", previous_input, denkey, " : ", den_const_g);
  }

  // If any of num den constant changes, update the state-space models.
  if (num_den_constant_names_g_.first != "1" || num_den_constant_names_g_.second != "1") {
    // Update Gss accordingly from the TF version.
    Gss_.updateStateSpace(Gtf_);
    QGinv_ss_.updateStateSpace(QGinv_tf_);
  }

  // Debug
  //	ns_utils::print("Current Qfilter Model");
  //	Qfilter_tf_.print();
  //
  //	ns_utils::print("Current G Model");
  //	Gtf_.print();
  //
  //	ns_utils::print("Current Q/G Model");
  //	QGinv_tf_.print();

  // Simulate Qs, Gs, Qs/Gs/
  // set previous_input u of Q(s) to get the filtered output. u--> Q(s) -->uf
  // steering, gas sent to the vehicle.

  /**
   * @brief Filtered control command that has been not delayed yet.
   * */
  auto uf = Qfilter_ss_.simulateOneStep(x0_qfilter_, previous_input);  // -->ufiltered

  //  simulate y --> Q(s)/ G(s) --> u-du (original previous_input - disturbance previous_input).

  /**
   * @brief u(-Td) = u - ud : is an estimate of what enters to the system as control. It is the
   * delayed control signal estimate where u represent the non-delayed control signal generated by
   * the controller.
   * */
  x0_inv_system_.setZero();
  auto const && u_minus_ud = QGinv_ss_.simulateOneStep(x0_inv_system_, measured_model_state);

  // Get difference of uf-(u-du) ~=du
  auto const && du = uf - u_minus_ud;

  // Send du to the G(s) as the previous_input du --> G(s) --> dyu to obtain compensation signal.

  // The response of the vehicle state model to the disturbance input.
  // x0_gsystem_.setZero();
  auto const && ydu = Gss_.simulateOneStep(x0_gsystem_, du);  // output is y (i.e ey, eyaw, ...).

  // ns_utils::print("Current velocity and steering :", num_den_args_of_g.first, previous_input);
  // ns_utils::print("Current V^2 and cos(delta)^2  : ",
  // pair_func_map_["v"](num_den_args_of_g.first),
  // pair_func_map_["delta"](num_den_args_of_g.second),
  //"\n");

  // Get outputs.
  /**
   * @brief Outputs of the delay compensator.
   * uf: u_filtered,Q(s)*u where u is the previous_input sent to the system.
   * u_minus_ud: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
   * y2: du = uf - y2 where du is the estimated disturbance previous_input
   * ydu: ydu = G(s)*du where ydu is the response of the system to du.
   * */

  y_outputs[0] = uf;                          // ufiltered
  y_outputs[1] = u_minus_ud;                  // u-du
  y_outputs[2] = du;                          // du
  y_outputs[3] = ydu;                         // for time being y of du-->G(s)--> ey_du
  y_outputs[4] = measured_model_state + ydu;  // for time being y of du-->G(s)--> ey_du

  // Get Ad_, Bd_, Cd_,Dd_ from QGinv_ss_.
  // getSSsystem(QGinv_ss_);

  // Print matrices.
  //	ns_eigen_utils::printEigenMat(Ad_, "Ad:");
  //	ns_eigen_utils::printEigenMat(Bd_, "Bd:");
  //	ns_eigen_utils::printEigenMat(Cd_, "Cd:");
  //	ns_eigen_utils::printEigenMat(Dd_, "Dd:");
}

void DelayCompensatorCore_PrototypeExample::getSSsystem(ns_control_toolbox::tf2ss const & ss)
{
  Ad_ = ss.Ad();
  Bd_ = ss.B();
  Cd_ = ss.Cd();
  Dd_ = ss.Dd();
}

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
  const state_vector_vehicle_t & current_measurements,
  const input_vector_vehicle_t & steering_and_ideal_steering,
  std::shared_ptr<DelayCompensatatorMsg> & msg_compensation_results,
  std::shared_ptr<DelayCompensatorDebugMsg> & msg_debug_results)
{
  setInitialStates();  // sets once.

  // Q filter inputs.
  auto const & steering_input = steering_and_ideal_steering(0, 0);
  auto const & uf_ey = ss_qfilter_ey_.simulateOneStep(xu0_ey_, steering_input);
  auto const & uf_eyaw = ss_qfilter_eyaw_.simulateOneStep(xu0_eyaw_, steering_input);
  auto const & uf_steering = ss_qfilter_eyaw_.simulateOneStep(xu0_eyaw_, steering_input);

  // Q filter outputs.
  auto const & measured_ey = current_measurements(0, 0);
  auto const & measured_eyaw = current_measurements(1, 0);
  auto const & measured_steering = current_measurements(2, 0);

  // (1 - Q)
  auto yf_ey = ss_qfilter_ey_.simulateOneStep(xy0_ey_, measured_ey);
  auto yf_eyaw = ss_qfilter_eyaw_.simulateOneStep(xy0_eyaw_, measured_eyaw);
  auto yf_steering = ss_qfilter_eyaw_.simulateOneStep(xy0_eyaw_, measured_steering);

  // Simulate the vehicle outputs.
  auto const & prev_ideal_steering = steering_and_ideal_steering(1, 0);

  input_temp_ = input_vector_vehicle_t{uf_ey, prev_ideal_steering};
  vehicle_model_ptr_->simulateOneStep(output_temp_, x0_qey_, input_temp_);
  auto const vec_ey_output = output_temp_(0, 0);

  input_temp_ = input_vector_vehicle_t{uf_eyaw, prev_ideal_steering};
  vehicle_model_ptr_->simulateOneStep(output_temp_, x0_qeyaw_, input_temp_);
  auto const vec_eyaw_output = output_temp_(1, 0);

  input_temp_ = input_vector_vehicle_t{uf_steering, prev_ideal_steering};
  vehicle_model_ptr_->simulateOneStep(output_temp_, x0_qsteering_, input_temp_);
  auto const vec_steering_output = output_temp_(2, 0);

  // Set the message values.
  msg_debug_results->lat_uf = uf_ey;
  msg_debug_results->lat_yu = vec_ey_output + measured_ey - yf_ey;
  msg_compensation_results->lateral_deviation_error_compensation_ref = msg_debug_results->lat_yu;

  msg_debug_results->heading_uf = uf_eyaw;
  msg_debug_results->heading_yu = vec_eyaw_output + measured_eyaw - yf_eyaw;
  msg_compensation_results->heading_angle_error_compensation_ref = msg_debug_results->heading_yu;

  msg_debug_results->steering_uf = uf_steering;
  msg_debug_results->steering_yu = vec_steering_output + measured_eyaw - yf_steering;
  msg_compensation_results->steering_error_compensation_ref = vec_steering_output + yf_steering;

  // Send to the vehicle model.
  // Q filter measurements.
  // Sum the signals to compute the new references for the controllers.

  ns_utils::print("Filtered inputs :", uf_ey, uf_eyaw, uf_steering);
  ns_utils::print("Filtered outputs :", yf_ey, yf_eyaw, yf_steering);
  ns_utils::print("And input temp: ");
  ns_eigen_utils::printEigenMat(input_temp_);

  ns_utils::print("And output temp: ");
  ns_eigen_utils::printEigenMat(output_temp_);

  ns_utils::print("And x0_qey temp: ");
  ns_eigen_utils::printEigenMat(x0_qey_);

  ns_eigen_utils::printEigenMat(current_measurements);
  ns_eigen_utils::printEigenMat(steering_and_ideal_steering);
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
