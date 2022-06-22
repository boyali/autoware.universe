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

#include "vehicle_models/vehicle_kinematic_error_model.hpp"

#include "autoware_control_toolbox.hpp"

NonlinearVehicleKinematicModel::NonlinearVehicleKinematicModel(
  double const & wheelbase, double const & tau_vel, double const & tau_steer,
  double const & deadtime_vel, double const & deadtime_steer, double const & dt)
: wheelbase_{wheelbase},
  tau_steer_{tau_steer},
  tau_vel_{tau_vel},
  dead_time_steer_{deadtime_steer},
  dead_time_vel_{deadtime_vel},
  dt_{dt}
{
  size_t const order_pade = 2;
  auto tf_steer_input = ns_control_toolbox::pade(deadtime_steer, order_pade);
  auto tf_vel_input = ns_control_toolbox::pade(deadtime_vel, order_pade);

  if (!ns_utils::isEqual(deadtime_vel, 0.)) {
    use_delay_vel = true;
    deadtime_velocity_model_ = ns_control_toolbox::tf2ss(tf_vel_input, dt);

    ns_utils::print("deadtime vel model");
    deadtime_velocity_model_.print_discrete_system();
  }

  if (!ns_utils::isEqual(deadtime_steer, 0.)) {
    use_delay_steer = true;
    deadtime_steering_model_ = ns_control_toolbox::tf2ss(tf_steer_input, dt);

    ns_utils::print("deadtime steer model");
    deadtime_steering_model_.print_discrete_system();
  }

  xv0_ = Eigen::MatrixXd::Zero(order_pade, 1);
  xs0_ = Eigen::MatrixXd::Zero(order_pade, 1);

  //	ns_utils::print("delay internal state of vel");
  //	ns_eigen_utils::printEigenMat(xv0_);
  //	int a = 1;
}

void NonlinearVehicleKinematicModel::getInitialStates(std::array<double, 4> & x0) { x0 = x0_; }

/**
 *@brief Integrates the nonlinear dynamics one-step.
 * */
std::array<double, 4> NonlinearVehicleKinematicModel::simulateNonlinearOneStep(
  const double & desired_velocity, double const & desired_steering)
{
  auto Vd = desired_velocity;
  auto delta_d = desired_steering;

  if (use_delay_vel) {
    // ns_utils::print("deadtime vel model");
    // deadtime_velocity_model_.print_discrete_system();

    auto && Ad = deadtime_velocity_model_.Ad();
    auto && Bd = deadtime_velocity_model_.Bd();
    auto && Cd = deadtime_velocity_model_.Cd();
    auto && Dd = deadtime_velocity_model_.Dd();

    ns_utils::print("in sim vel delay");
    ns_eigen_utils::printEigenMat(Cd * xv0_);
    ns_eigen_utils::printEigenMat(Bd * desired_velocity);
    //		ns_eigen_utils::printEigenMat((Cd * xv0_ + Dd * desired_velocity));

    // Yield y first
    Vd = (Cd * xv0_ + Dd * desired_velocity)(0);

    // Update x0v
    xv0_ = Ad * xv0_ + Bd * desired_velocity;

    // int a = 1;
  }

  if (use_delay_steer) {
    // ns_utils::print("deadtime steer model");
    // deadtime_steering_model_.print_discrete_system();

    auto Ad = deadtime_steering_model_.Ad();
    auto Bd = deadtime_steering_model_.Bd();
    auto Cd = deadtime_steering_model_.Cd();
    auto Dd = deadtime_steering_model_.Dd();

    // Yield y first
    delta_d = (Cd * xv0_ + Dd * desired_steering)(0);

    ns_utils::print("in sim steering delay");
    ns_eigen_utils::printEigenMat(Cd);
    //	ns_eigen_utils::printEigenMat((Cd * xv0_ + Bd * desired_steering));

    // Update x0v
    xv0_ = Ad * xv0_ + Bd * desired_steering;

    // int a = 1;
  }

  // Get the previous states.
  auto && ey0 = x0_[0];
  auto && eyaw0 = x0_[1];
  auto && delta0 = x0_[2];
  auto && V0 = x0_[3];

  x0_[0] = ey0 + dt_ * V0 * sin(eyaw0);                                     // ey
  x0_[1] = eyaw0 + dt_ * (V0 / wheelbase_) * (tan(delta0) - tan(delta_d));  // eyaw
  x0_[2] = delta0 - dt_ * (1 / tau_steer_) * (delta0 - delta_d);            // delta
  x0_[3] = V0 - dt_ * (1 / tau_vel_) * (V0 - Vd);                           // v

  return x0_;
}

/**
 *@brief Integrates the linear dynamics one-step.
 * */
std::array<double, 4> NonlinearVehicleKinematicModel::simulateLinearOneStep(
  const double & desired_velocity, double const & desired_steering)
{
  auto Vd = desired_velocity;
  auto delta_d = desired_steering;

  if (use_delay_vel) {
    // ns_utils::print("deadtime vel model");
    // deadtime_velocity_model_.print_discrete_system();

    auto && Ad = deadtime_velocity_model_.Ad();
    auto && Bd = deadtime_velocity_model_.Bd();
    auto && Cd = deadtime_velocity_model_.Cd();
    auto && Dd = deadtime_velocity_model_.Dd();

    ns_utils::print("in sim vel delay");
    ns_eigen_utils::printEigenMat(Cd * xv0_);
    ns_eigen_utils::printEigenMat(Bd * desired_velocity);
    //		ns_eigen_utils::printEigenMat((Cd * xv0_ + Dd * desired_velocity));

    // Yield y first
    Vd = (Cd * xv0_ + Dd * desired_velocity)(0);

    // Update x0v
    xv0_ = Ad * xv0_ + Bd * desired_velocity;

    // int a = 1;
  }

  if (use_delay_steer) {
    // ns_utils::print("deadtime steer model");
    // deadtime_steering_model_.print_discrete_system();

    auto Ad = deadtime_steering_model_.Ad();
    auto Bd = deadtime_steering_model_.Bd();
    auto Cd = deadtime_steering_model_.Cd();
    auto Dd = deadtime_steering_model_.Dd();

    // Yield y first
    delta_d = (Cd * xv0_ + Dd * desired_steering)(0);

    ns_utils::print("in sim steering delay");
    ns_eigen_utils::printEigenMat(Cd);
    //	ns_eigen_utils::printEigenMat((Cd * xv0_ + Bd * desired_steering));

    // Update x0v
    xv0_ = Ad * xv0_ + Bd * desired_steering;

    // int a = 1;
  }

  // Get the previous states.
  auto && ey0 = x0_[0];
  auto && eyaw0 = x0_[1];
  auto && delta0 = x0_[2];
  auto && V0 = x0_[3];

  x0_[0] = ey0 + dt_ * V0 * eyaw0;                                                    // ey
  x0_[1] = eyaw0 + V0 * delta0 / (wheelbase_ * std::cos(delta0) * std::cos(delta0));  // eyaw
  x0_[2] = delta0 - dt_ * (1 / tau_steer_) * (delta0 - delta_d);                      // delta
  x0_[3] = V0 - dt_ * (1 / tau_vel_) * (V0 - Vd);                                     // v

  return x0_;
}

// LINEAR KINEMATIC ERROR VEHICLE MODEL

observers::LinearKinematicErrorModel::LinearKinematicErrorModel(
  const float64_t & wheelbase, const float64_t & tau_steering, const float64_t & dt)
: wheelbase_{wheelbase},
  tau_steering_{tau_steering},
  dt_{dt},
  A_{state_matrix_vehicle_t::Zero()},
  B_{input_matrix_vehicle_t::Zero()},
  C_{state_matrix_vehicle_t ::Identity()},
  D_{input_matrix_vehicle_t::Zero()},
  Ad_{state_matrix_vehicle_t::Zero()},
  Bd_{input_matrix_vehicle_t::Zero()},
  Cd_{state_matrix_vehicle_t ::Identity()},
  Dd_{input_matrix_vehicle_t::Zero()},
  I_At2_{state_matrix_vehicle_t::Identity()},
  x0_{state_vector_vehicle_t ::Zero()}

{
  // Assuming tau does not change.
  A_(2, 2) = -1. / tau_steering_;
  B_(2, 0) = 1. / tau_steering_;
}
void observers::LinearKinematicErrorModel::printContinuousSystem()
{
  ns_utils::print("Matrix A: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(A_));

  ns_utils::print("Matrix B: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(B_));

  ns_utils::print("Matrix C: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(C_));

  ns_utils::print("Matrix D: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(D_));
}

void observers::LinearKinematicErrorModel::printDiscreteSystem()
{
  ns_utils::print("Matrix Ad: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Ad_));

  ns_utils::print("Matrix Bd: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Bd_));

  ns_utils::print("Matrix Cd: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Cd_));

  ns_utils::print("Matrix Dd: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Dd_));
}
void observers::LinearKinematicErrorModel::updateStateSpace(
  const float64_t & vref, const float64_t & steering_ref)
{
  auto const && cos_sqr = std::cos(steering_ref) * std::cos(steering_ref);

  A_(0, 1) = vref;
  A_(1, 2) = vref / (wheelbase_ * cos_sqr);

  //  auto IA = state_matrix_vehicle_t::Identity() - A_ * dt_ / 2;
  //  auto Ainv = IA.inverse();

  updateI_Ats2(vref, cos_sqr);

  // Discretisize.
  auto const & I = state_matrix_vehicle_t::Identity();

  Ad_ = I_At2_ * (I + A_ * dt_ / 2.);
  Bd_ = I_At2_ * B_ * dt_;
  Cd_ = C_ * I_At2_;
  Dd_ = D_ + C_ * Bd_ / 2.;

  // Debug
  //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Ainv));
  //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(I_At2_));
}
void observers::LinearKinematicErrorModel::updateI_Ats2(
  float64_t const & vref, float64_t const & cos_steer_sqr)
{
  auto const & L = wheelbase_;
  auto const asqr = cos_steer_sqr;
  auto const & tau = tau_steering_;
  auto const & var = 2 * tau + dt_;

  I_At2_(0, 1) = vref * dt_ / 2.;
  I_At2_(0, 2) = tau_steering_ * vref * vref * dt_ * dt_ / (2. * L * asqr * var);

  I_At2_(1, 2) = tau * vref * dt_ / (L * asqr * var);
  I_At2_(2, 2) = 2. * tau / var;
}
void observers::LinearKinematicErrorModel::simulateOneStep(
  state_vector_vehicle_t & y0, const float64_t & u)
{
  // first compute the outputs using the current initial conditions, then update
  // the initial states.
  y0 = Cd_ * x0_.eval() + Dd_ * u;
  x0_ = Ad_ * x0_.eval() + Bd_ * u;
}

void observers::LinearKinematicErrorModel::simulateOneStep_withPastStates(
  state_vector_vehicle_t & y0, state_vector_vehicle_t & x0,
  const autoware::common::types::float64_t & u)
{
  // first compute the outputs using the current initial conditions, then update
  // the initial states.
  x0 = Ad_ * x0.eval() + Bd_ * u;
  y0 = Cd_ * x0.eval() + Dd_ * u;
}

void observers::LinearKinematicErrorModel::simulateOneStep(
  state_vector_vehicle_t & y0, state_vector_vehicle_t & x0, float64_t const & u)
{
  // first compute the outputs using the current initial conditions, then update
  // the initial states.

  // first update the output
  // y0 = x0 + dt_ * (A_ * x0 + B_ * u);
  y0 = Cd_ * x0.eval() + Dd_ * u;
  x0 = Ad_ * x0.eval() + Bd_ * u;
}

void observers::LinearKinematicErrorModel::updateInitialStates(state_vector_vehicle_t const & x0)

{
  x0_ << x0;
  are_initial_states_set_ = true;

  // ns_eigen_utils::printEigenMat(Eigen::MatrixXd(x0_));
}

void observers::LinearKinematicErrorModel::updateInitialStates(Eigen::MatrixXd const & x0)

{
  x0_ << x0;
  are_initial_states_set_ = true;

  // ns_eigen_utils::printEigenMat(Eigen::MatrixXd(x0_));
}
void observers::LinearKinematicErrorModel::updateInitialStates(
  float64_t const & ey, float64_t const & eyaw, float64_t const & steering)
{
  x0_ << ey, eyaw, steering;
  are_initial_states_set_ = true;
}
state_vector_vehicle_t observers::LinearKinematicErrorModel::getInitialStates() const
{
  return x0_;
}
