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
#include <memory>
#include <vector>

namespace observers
{
// -- HELPER METHODS --
tf_t get_nthOrderTF(
  const autoware::common::types::float64_t &w_cut_off_hz, const int &n)
{
  auto const &wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

  // float64_t time_constant_of_qfilter{};
  auto const &tau = 1.0 / wc_rad_sec;

  // --------------- Qfilter Construction --------------------------------------
  // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
  // Calculate the transfer function.
  ns_control_toolbox::tf_factor denominator{std::vector<double>{tau, 1.}};  // (tau*s+1)

  // Take power of the denominator.
  denominator.power(static_cast<unsigned int>(n));

  // Create the transfer function from a numerator an denominator.
  auto const &&q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

  return q_tf;
}

tf_t get_nthOrderTFwithDampedPoles(
  const autoware::common::types::float64_t &w_cut_off_hz,
  const int &remaining_order,
  const autoware::common::types::float64_t &damping_val)
{
  auto const &wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

  // float64_t time_constant_of_qfilter{};
  auto const &tau = 1.0 / wc_rad_sec;

  // A second order damped transfer function.
  auto q_tf_damped = tf_t({1.}, {tau * tau, 2 * damping_val * tau, 1.});

  if (remaining_order > 0)
  {
    auto q_remaining_tf = get_nthOrderTF(w_cut_off_hz, remaining_order);
    q_tf_damped = q_tf_damped * q_remaining_tf;
  }

  return q_tf_damped;
}

// ------------------- Communication Delay Compensator using Forward Dynamics. ---------

LateralCommunicationDelayCompensator::LateralCommunicationDelayCompensator(
  obs_model_ptr_t observer_vehicle_model,
  model_ptr_t vehicle_model,
  const tf_t &qfilter_lateral,
  sLyapMatrixVecs const &lyap_matsXY,
  const float64_t &dt)
  : observer_vehicle_model_ptr_(std::move(observer_vehicle_model)),
    vehicle_model_ptr_(std::move(vehicle_model)),
    tf_qfilter_lat_{qfilter_lateral},
    vXs_{lyap_matsXY.vXs},
    vYs_{lyap_matsXY.vYs},
    dt_{dt},
    qfilter_order_{qfilter_lateral.order()}
{
  // Compute the state-space model of QGinv(s)
  // Do not forget to enter the time step dt.
  ss_qfilter_lat_ = std::make_unique<ss_t>(tf_qfilter_lat_, dt_);

  /**
   * Compute 1-Q transfer function and the ss representation
   * */
  ns_control_toolbox::tf_factor num_fc_({tf_qfilter_lat_.num()});
  ns_control_toolbox::tf_factor den_fc_({tf_qfilter_lat_.den()});

  auto den_num = den_fc_ - num_fc_;

  tf_one_minus_qfilter_lat_ = tf_t(den_num(), den_fc_());
  ss_one_min_qfilter_lat_ = std::make_unique<ss_t>(tf_one_minus_qfilter_lat_, dt_);

  /** initialize (1-Q) filter states */
  xey0_ = Eigen::MatrixXd(tf_one_minus_qfilter_lat_.order(), 1);
  xeyaw0_ = Eigen::MatrixXd(tf_one_minus_qfilter_lat_.order(), 1);
  xsteer0_ = Eigen::MatrixXd(tf_one_minus_qfilter_lat_.order(), 1);

  xey0_.setZero();
  xeyaw0_.setZero();
  xsteer0_.setZero();

  /** Initialize input and disturbance filter states */
  xu0_ = Eigen::MatrixXd(qfilter_order_, 1);
  xd0_ = Eigen::MatrixXd(qfilter_order_, 1);

  xu0_.setZero();
  xd0_.setZero();
}

void LateralCommunicationDelayCompensator::printQfilterTFs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("Transfer function of qfilter of lateral error : \n");
  tf_qfilter_lat_.print();

  ns_utils::print("Transfer function of [1-Q] of lateral error : \n");
  tf_one_minus_qfilter_lat_.print();
}

void LateralCommunicationDelayCompensator::printQfilterSSs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("State-Space Matrices of qfilter of lateral error : \n");
  ss_qfilter_lat_->print();

  ns_utils::print("State-Space Matrices of [1_Q] qfilter of lateral error : \n");
  ss_one_min_qfilter_lat_->print();
}

void LateralCommunicationDelayCompensator::printLyapMatrices() const
{
  ns_utils::print("Lyapunov X matrices : ");
  if (!vXs_.empty())
  {
    for (auto const &item : vXs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }

  ns_utils::print("Lyapunov Y matrices : ");
  if (!vYs_.empty())
  {
    for (auto const &item : vYs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }
}

void LateralCommunicationDelayCompensator::setInitialStates()
{
  if (!is_observer_model_initial_states_set_ &&
      observer_vehicle_model_ptr_->areInitialStatesSet())
  {
    auto const vehicle_states = observer_vehicle_model_ptr_->getInitialStates();

    xhat0_prev_ << vehicle_states(0), vehicle_states(1), vehicle_states(2), 0.;
    is_observer_model_initial_states_set_ = true;
  }
}

void LateralCommunicationDelayCompensator::resetInitialState()
{
  xhat0_prev_.setZero();
  is_observer_model_initial_states_set_ = false;
}

void
LateralCommunicationDelayCompensator::simulateOneStep(
  const state_vector_vehicle_t &current_measurements,
  float64_t const &prev_steering_control_cmd,
  float64_t const &current_steering_cmd,
  std::shared_ptr<DelayCompensatatorMsg> const
  &msg_compensation_results,
  std::shared_ptr<DelayCompensatorDebugMsg>
  const &msg_debug_results)
{
  // If the initial states of the state observer are not set, sets it.
  setInitialStates();

  // Compute the observer gain matrix for the current operating conditions.
  computeObserverGains(current_measurements);

  // Filter the current input and store it in the as the filtered previous input.
  qfilterControlCommand(current_steering_cmd);

  // Run the state observer to estimate the current state.
  estimateVehicleStatesQ(current_measurements, prev_steering_control_cmd, current_steering_cmd);

  // Final assignment steps.
  msg_debug_results->lat_uf = current_qfiltered_control_cmd_;
  msg_debug_results->lat_ey_hat = prev_yobs_(0);
  msg_debug_results->lat_eyaw_hat = prev_yobs_(1);
  msg_debug_results->lat_steering_hat = prev_yobs_(2);
  msg_debug_results->lat_duf = df_d0_;

  msg_compensation_results->lateral_deviation_error_compensation_ref = yv_d0_(0);
  msg_compensation_results->heading_angle_error_compensation_ref = yv_d0_(1);
  msg_compensation_results->steering_compensation_ref = yv_d0_(2);

  // Debug
  // end of debug
}

void LateralCommunicationDelayCompensator::computeObserverGains(
  const state_vector_vehicle_t &current_measurements)
{
  // Get the current nonlinear Lyapunov parameters.
  theta_params_.setZero();
  observer_vehicle_model_ptr_->evaluateNonlinearTermsForLyap(theta_params_, current_measurements);

  // Compute the parametric lyapunov matrices.
  auto Xc = vXs_.back();  // X0, Y0 are stored at the end.
  auto Yc = vYs_.back();

  // P(th) = P0 + th1*P1 + ...
  for (size_t k = 0; k < vXs_.size() - 1; ++k)
  {
    Xc += theta_params_(static_cast<Eigen::Index>(k)) * vXs_[k];
    Yc += theta_params_(static_cast<Eigen::Index>(k)) * vYs_[k];
  }
  Lobs_ = Yc * Xc.inverse();
}

void LateralCommunicationDelayCompensator::qfilterControlCommand(
  const autoware::common::types::float64_t &current_control_cmd)
{
  // First give the output, then update the states.
  prev_qfiltered_control_cmd_ = current_qfiltered_control_cmd_;
  current_qfiltered_control_cmd_ = ss_qfilter_lat_->simulateOneStep(xu0_, current_control_cmd);
}

/**
 * @brief Two-steps estimation method is used as we need the current state estimate. To do this,
 * we use the previous values for the vehicle model. Upon estimating the current states, we
 * predict the next states and broadcast it.
 * */
void LateralCommunicationDelayCompensator::estimateVehicleStates(
  state_vector_vehicle_t const &current_measurements,
  float64_t const &, /*prev_steering_control_cmd,*/
  float64_t const & /*current_steering_cmd*/)
{
  /**
  * xbar = A @ x0_hat + B * u_prev + Bwd
  * ybar = C @ xbar + D * uk_qf
  * */
  // FIRST STEP: propagate the previous states.
  xbar_temp_ = xhat0_prev_.eval();
  observer_vehicle_model_ptr_->simulateOneStep(prev_yobs_, xbar_temp_, prev_qfiltered_control_cmd_);

  xhat0_prev_ = xbar_temp_ + Lobs_.transpose() * (prev_yobs_ - current_measurements);

  // Before updating the observer states, use xhat0 current estimate to
  // simulate the disturbance input.
  // Apply the q-filter to the disturbance state
  auto dist_state = xhat0_prev_.eval().bottomRows<1>()(0);

  // UPDATE the OBSERVER STATE: Second step: simulate the current states and controls.
  observer_vehicle_model_ptr_->simulateOneStep(
    current_yobs_, xhat0_prev_,
    current_qfiltered_control_cmd_);
  /**
    * d = (u - ue^{-sT})
    * uf - dfilt = ue^{-sT}
    * We filter this state because in the current_yobs_ C row is zero, we cannot observe it from this output.
    * */
  df_d0_ = current_qfiltered_control_cmd_ - ss_qfilter_lat_->simulateOneStep(xd0_, dist_state);

  // Send the qfiltered disturbance input to the vehicle model to get the response.

  xv_d0_ = current_measurements.eval();
  vehicle_model_ptr_->simulateOneStepZeroState(yv_d0_, xv_d0_, df_d0_);
}

void LateralCommunicationDelayCompensator::estimateVehicleStatesQ(
  state_vector_vehicle_t const &current_measurements,
  float64_t const &prev_steering_control_cmd,
  float64_t const &current_steering_cmd)
{
  /**
  * xbar = A @ x0_hat + B * u_prev + Bwd
  * ybar = C @ xbar + D * uk_qf
  * */

  // FIRST STEP: propagate the previous states.
  xbar_temp_ << xhat0_prev_.eval();
  observer_vehicle_model_ptr_->simulateOneStep(prev_yobs_, xbar_temp_, prev_steering_control_cmd);

  xhat0_prev_ = xbar_temp_ + Lobs_.transpose() * (prev_yobs_ - current_measurements);

  /**
   * Compute (1-Q) outputs
   * */

  y_one_minus_Q_(0) = ss_one_min_qfilter_lat_->simulateOneStep(xey0_, current_measurements(0));
  y_one_minus_Q_(1) = ss_one_min_qfilter_lat_->simulateOneStep(xeyaw0_, current_measurements(1));
  y_one_minus_Q_(2) = ss_one_min_qfilter_lat_->simulateOneStep(xsteer0_, current_measurements(2));


  // get the current estimated state before updating for the next state.
  xv_hat0_current_ = xhat0_prev_.eval().topRows(3);

  // UPDATE the OBSERVER STATE: Second step: simulate the current states and controls.
  observer_vehicle_model_ptr_->simulateOneStep(current_yobs_, xhat0_prev_, current_steering_cmd);

  /**
   *  Simulate the current qfiltered command
   * */
  vehicle_model_ptr_->simulateOneStepZeroState(
    yv_hat0_current_, xv_hat0_current_,
    current_qfiltered_control_cmd_);

  yv_d0_ = yv_hat0_current_ + y_one_minus_Q_;

  // Debug
  // ns_eigen_utils::printEigenMat(yv_d0_, "Delay Compensation References :");
}

LateralDisturbanceCompensator::LateralDisturbanceCompensator(
  LateralDisturbanceCompensator::obs_model_ptr_t observer_vehicle_model,
  const tf_t &qfilter_lateral,
  const sLyapMatrixVecs &lyap_matsXY,
  const autoware::common::types::float64_t &dt)
  : observer_vehicle_model_ptr_(std::move(observer_vehicle_model)),
    tf_qfilter_lat_{qfilter_lateral},
    vXs_{lyap_matsXY.vXs},
    vYs_{lyap_matsXY.vYs},
    dt_{dt},
    qfilter_order_{qfilter_lateral.order()}
{
  // Compute the state-space model of QGinv(s)
  ss_qfilter_lat_ = std::make_unique<ss_t>(tf_qfilter_lat_, dt_);  // Do not forget to enter the time step dt.

  /**
   * Initialize the vectors.
   * */
  xu0_ = Eigen::MatrixXd(qfilter_order_, 1);
  xd0_ = Eigen::MatrixXd(qfilter_order_, 1);

  xu0_.setZero();
  xd0_.setZero();
}

void LateralDisturbanceCompensator::printQfilterTFs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("Transfer function of qfilter of lateral error : \n");
  tf_qfilter_lat_.print();
}

void LateralDisturbanceCompensator::printQfilterSSs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("State-Space Matrices of qfilter of lateral error : \n");
  ss_qfilter_lat_->print();
}

void LateralDisturbanceCompensator::printLyapMatrices() const
{
  ns_utils::print("Lyapunov X matrices : ");
  if (!vXs_.empty())
  {
    for (auto const &item : vXs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }

  ns_utils::print("Lyapunov Y matrices : ");
  if (!vYs_.empty())
  {
    for (auto const &item : vYs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }

}

void LateralDisturbanceCompensator::setInitialStates()
{
  if (!is_vehicle_initial_states_set_ && observer_vehicle_model_ptr_->areInitialStatesSet())
  {
    auto const vehicle_states = observer_vehicle_model_ptr_->getInitialStates();

    xhat0_prev_ << vehicle_states(0), vehicle_states(1), vehicle_states(2), 0.;
    is_vehicle_initial_states_set_ = true;
  }
}

void LateralDisturbanceCompensator::computeObserverGains(
  const state_vector_vehicle_t &current_measurements)
{
  // Get the current nonlinear Lyapunov parameters.
  theta_params_.setZero();
  observer_vehicle_model_ptr_->evaluateNonlinearTermsForLyap(theta_params_, current_measurements);

  // Compute the parametric lyapunov matrices.
  auto Xc = vXs_.back();  // X0, Y0 are stored at the end.
  auto Yc = vYs_.back();

  // P(th) = P0 + th1*P1 + ...
  for (size_t k = 0; k < vXs_.size() - 1; ++k)
  {
    Xc += theta_params_(static_cast<Eigen::Index>(k)) * vXs_[k];
    Yc += theta_params_(static_cast<Eigen::Index>(k)) * vYs_[k];
  }

  Lobs_ = Yc * Xc.inverse();
}

void
LateralDisturbanceCompensator::simulateOneStep(
  const state_vector_vehicle_t &current_measurements,
  const autoware::common::types::float64_t &prev_steering_control_cmd,
  const autoware::common::types::float64_t &current_steering_cmd,
  std::shared_ptr<DelayCompensatatorMsg> const &msg_compensation_results)
{
  setInitialStates();

  // Compute the observer gain matrix for the current operating conditions.
  computeObserverGains(current_measurements);

  // Filter the current input and store it in the as the filtered previous input.
  qfilterControlCommand(current_steering_cmd);

  // Run the state observer to estimate the current state.
  estimateVehicleStates(current_measurements, prev_steering_control_cmd, current_steering_cmd);

  // Final assignment steps.
  msg_compensation_results->steering_dob = -dist_input_;
}

void LateralDisturbanceCompensator::qfilterControlCommand(
  const float64_t &current_control_cmd)
{
  // First give the output, then update the states.
  current_qfiltered_control_cmd_ = ss_qfilter_lat_->simulateOneStep(xu0_, current_control_cmd);
}

void LateralDisturbanceCompensator::estimateVehicleStates(
  const state_vector_vehicle_t &current_measurements,
  const float64_t &prev_steering_control_cmd,
  const float64_t &current_steering_cmd)
{
  /**
  *   xbar = A @ x0_hat + B * u_prev + Bwd + Lobs*(yhat - ytime_delay_compensator)
  *   ybar = C @ xbar + D * uk_qf
  *
  * x and y belong to the VEHICLE state observer.
  * */

  /**
  * Qfilter the time-delay state estimator output ytime_delay = y(without delay) - dcurvature
  * dcurvature is an output disturbance, we want to find out an input causing this output.
  * */


  // FIRST STEP: propagate the previous states.
  xbar_temp_ = xhat0_prev_.eval();
  observer_vehicle_model_ptr_->simulateOneStep(ybar_temp_, xbar_temp_, prev_steering_control_cmd);

  /**
  * Here using yv_d0_ is the point of DDOB. The last row of xhat is
   * the disturbance input estimated for compensation.
  * */
  xhat0_prev_ = xbar_temp_ + Lobs_.transpose() * (ybar_temp_ - current_measurements);
  dist_input_ = ss_qfilter_lat_->simulateOneStep(xd0_, xhat0_prev_.eval().bottomRows<1>()(0));

  /**
   * UPDATE the OBSERVER STATE: Second step: simulate the current states and controls.
   * */
  observer_vehicle_model_ptr_->simulateOneStep(current_yobs_, xhat0_prev_, current_steering_cmd);
}

void LateralDisturbanceCompensator::resetInitialState()
{
  xhat0_prev_.setZero(); //.bottomRows<1>()(0) = 0; // .setZero();
  is_vehicle_initial_states_set_ = false;
}
}  // namespace observers