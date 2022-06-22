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

/**
 * @brief Communication Delay Compensator Core.
 * */

observers::CommunicationDelayCompensatorCore::CommunicationDelayCompensatorCore(
  tf_t & qfilter, tf_t & g_system_model, double const & dt)
: q_filter_tf_{qfilter}, g_tf_(g_system_model), dt_{dt}
{
  // The vector that keeps the output of the compensator.
  // y_outputs_.reserve(num_of_outputs);
  y_outputs_ = std::vector<float64_t>(num_of_outputs, 0.);

  // Internal states of the transfer functions in the form of x[k+1] = Ax[k] + Bu[k]
  x0_qfilter_.setZero();
  x0_gsystem_.setZero();
  x0_inv_system_.setZero();

  // Compute the state spaces which can simulate the system given a u, x0.
  q_filter_ss_ = ss_t(q_filter_tf_, dt_);
  g_ss_ = ss_t(g_tf_, dt_);

  // Compute Q(ss)/G(ss)
  auto g_inv = std::move(g_system_model);
  g_inv.inv();  // invert tempG

  // Q(s)/G*s)
  q_g_inv_tf_ = q_filter_tf_ * g_inv;

  // Compute the state-space model of QGinv(s)
  q_g_inv_ss_ = ss_t(q_g_inv_tf_, dt_);  // Do not forget to enter the time step dt.

  // Set default function mapping to identity if there is not varying parameters in the num and den.
  pair_func_map_["1"] = [](auto const & x) -> double { return x; };  // to prevent zero division.

  // Initialize the Eigen vectors.
  auto qfilter_order = std::max(q_filter_tf_.num().size(), q_filter_tf_.den().size());
  auto system_model_order = std::max(g_tf_.num().size(), g_tf_.den().size());
  auto inverse_system_order = std::max(q_g_inv_tf_.num().size(), q_g_inv_tf_.den().size());

  x0_qfilter_ = Eigen::MatrixXd(qfilter_order, 1);
  x0_gsystem_ = Eigen::MatrixXd(system_model_order, 1);
  x0_inv_system_ = Eigen::MatrixXd(inverse_system_order, 1);
}

/**
 * @brief Summarizes the communication delay compensator.
 * */
void observers::CommunicationDelayCompensatorCore::print() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("Qfilter Model : \n");
  q_filter_tf_.print();

  ns_utils::print("Forward model G(s) : \n\n");
  g_tf_.print();

  ns_utils::print(
    "G(s) num and den constant names :", num_den_constant_names_g_.first,
    num_den_constant_names_g_.second, "\n");

  ns_utils::print("Forward model Q/G(s) :  \n");
  q_g_inv_tf_.print();

  ns_utils::print(
    "Q/G(s) num and den constant names :", num_den_constant_names_g_inv_.first,
    num_den_constant_names_g_inv_.second);

  // 	ns_utils::print("\n -------------- DISCRETE STATE-SPACE MODELS ----------\n");
  // 	ns_utils::print("Qfilter State-Space: ");
  // 	q_filter_ss_.print_discrete_system();
  //
  // 	ns_utils::print("\n -------------- DISCRETE STATE-SPACE MODELS ----------\n");
  // 	ns_utils::print("System G(s) State-Space: ");
  // 	g_ss_.print_discrete_system();
  //
  // 	ns_utils::print("\n -------------- DISCRETE STATE-SPACE MODELS ----------\n");
  // 	ns_utils::print("System Q(s)/G(s) State-Space: ");
  // 	q_g_inv_ss_.print_discrete_system();
}

/**
 * @brief if there are dynamic parameters multiplying a^2* num/ (cos(b)* den) we can define using
 * it. a and b must be a function. These parameters belong to the G(s) system model.
 * */
void observers::CommunicationDelayCompensatorCore::setDynamicParams_num_den(
  observers::pairs_string_view_t & param_names, observers::pairs_func_maps_t & param_funcs)
{
  // Set G(s) parameters.
  num_den_constant_names_g_ = param_names;
  pair_func_map_ = param_funcs;

  // Invert the names for 1/G(s).
  num_den_constant_names_g_inv_ =
    pairs_string_view_t(num_den_constant_names_g_.second, num_den_constant_names_g_.first);
}

void observers::CommunicationDelayCompensatorCore::simulateOneStep(
  float64_t const & previous_input, float64_t const & measured_model_state,
  std::pair<float64_t, float64_t> const & num_den_args_of_g)
{
  // Get the output of num_constant for the G(s) = nconstant(x) * num / (denconstant(y) * den)
  if (num_den_constant_names_g_.first != "1") {
    auto && numkey = num_den_constant_names_g_.first;  // Corresponds to v in ey model.

    // Find the function of numerator variable and apply its function on it.
    auto && num_const_g = pair_func_map_[numkey](num_den_args_of_g.first);

    // Update numerator of G(s) and denominator of QGinv
    g_tf_.update_num_coef(num_const_g);
    q_g_inv_tf_.update_den_coef(num_const_g);  // since they are inverted.

    // ns_utils::print("State and its function : ", numkey, " : ", num_const_g);
  }

  if (num_den_constant_names_g_.second != "1") {
    auto && denkey = num_den_constant_names_g_.second;  // Corresponds to delta in ey model.
    auto && den_const_g = pair_func_map_[denkey](num_den_args_of_g.second);

    g_tf_.update_den_coef(den_const_g);
    q_g_inv_tf_.update_num_coef(den_const_g);  // since they are inverted.
    // ns_utils::print("State and its function : ", denkey, " : ", den_const_g);
  }

  // If any of num den constant changes, update the state-space models.
  if (num_den_constant_names_g_.first != "1" || num_den_constant_names_g_.second != "1") {
    // Update Gss accordingly from the TF version.
    g_ss_.updateStateSpace(g_tf_);
    q_g_inv_ss_.updateStateSpace(q_g_inv_tf_);

    // ns_utils::print("G(s) and Q(s)/G(s) are updated");
  }

  // Simulate Qs, Gs, Qs/Gs/
  // set previous_input u of Q(s) to get the filtered output. u--> Q(s) -->uf
  /**
   * @brief Filtered control command that has been not delayed yet.
   * u(controller command nondelayed)--> Q(s) -->uf
   * */
  auto const && uf = q_filter_ss_.simulateOneStep(x0_qfilter_, static_cast<double>(previous_input));

  // set the input to measured state of Q(s)/G(s) to get the input estimate.
  // ey, eyaw, or eVel --> Q(s)/G(s) -->u - du
  /**
   * @brief u(-Td) = u - ud : is an estimate of what enters to the system as control. It is the
   * delayed control signal estimate where u represent the non-delayed control signal generated by
   * the controller.
   * ey, eyaw, or eVel --> Q(s)/G(s) -->U(-T) = u(controller) - du
   * */
  // x0_inv_system_.setZero();
  auto const && u_du =
    q_g_inv_ss_.simulateOneStep(x0_inv_system_, measured_model_state);  // output is u-du.

  // Get difference of uf-(u-du) ~=du
  auto const && du = uf - u_du;

  // Send du to the G(s) as the previous_input du --> G(s) --> dyu to obtain compensation signal.
  // output is ydu (i.e ey, eyaw, ...).

  x0_gsystem_.setZero();
  auto const && ydu = g_ss_.simulateOneStep(x0_gsystem_, du);
  // Get outputs.
  /**
   * @brief Outputs of the delay compensator.
   * uf: u_filtered,Q(s)*u where u is the previous_input sent to the system.
   * u_du: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
   * y2: du = uf - y2 where du is the estimated disturbance previous_input
   * ydu: ydu = G(s)*du where ydu is the response of the system to du.
   * */

  std::fill(y_outputs_.begin(), y_outputs_.end(), 0.);
  // y_outputs_ = std::vector<float64_t>(num_of_outputs, 0.);

  y_outputs_[0] = uf;    // ufiltered
  y_outputs_[1] = u_du;  // u-du
  y_outputs_[2] = du;    // du
  y_outputs_[3] = ydu;   // ydu;   // du->Q(s)/G(s)--> y_du

  // reference correction.
  y_outputs_[4] = (measured_model_state + ydu);  // yu = y_measured (yu-ydu) + ydu  where
}

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
