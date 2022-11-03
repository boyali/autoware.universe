// Copyright 2022 Tier IV, Inc.
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

#include "nonlinear_mpc_core/initialization_lpv.hpp"

#include <vector>

bool LPVinitializer::simulateWithFeedback(Model::model_ptr_t const &model_ptr,
                                          SplineInterpolation const &piecewise_interpolator,
                                          ns_data::param_lpv_type_t const &params_lpv,
                                          ns_data::ParamsOptimization const &params_opt,
                                          ns_data::data_nmpc_core_type_t &nmpc_data)
{
  // Get the size of the trajectory.
  // number of state vectors stored in the std::vector.
  size_t const &K = nmpc_data.trajectory_data.nX();
  double const &dt = nmpc_data.mpc_prediction_dt;

  // Prepare an error state vector.
  /**
   * Full states are [x, y, psi, s, e_y, e_yaw, v, delta, ay],
   * and the error states [e_y, e_yaw, v, delta].
   * */
  Model::error_state_vector_t x_error{Model::error_state_vector_t::Zero()};

  // Set instrumental x and u to keep the results of the simulation (inplace integration).
  auto xk = nmpc_data.trajectory_data.X[0];  // current value is x0.
  auto uk = nmpc_data.trajectory_data.U[0];

  //  ns_utils::print("x0 and u0");
  //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(xk));
  //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(uk));

  /**
   * x  =[xw, yw, psi, s, e_y, e_yaw, v, delta]
   * */
  // Define placeholders for the system matrices.
  Model::state_matrix_t Ac{Model::state_matrix_t::Zero()};
  Model::control_matrix_t Bc{Model::control_matrix_t::Zero()};

  // Define Lyapunov matrices placeholders.
  Model::state_matrix_X_t Xr{Model::state_matrix_X_t::Zero()};
  Model::input_matrix_Y_t Yr{Model::input_matrix_Y_t::Zero()};

  // Prepare the initial and final cost,
  // so that we can observe whether the controller diverges.
  // double initial_error_cost{};
  // double final_error_cost{};

  /**
   * @brief feedback control value resulting in primal_infeasible in the optimization.
   * We further narrow down the control values by the following percentage.
   */

  // Prepare param vector placeholder.
  Model::param_vector_t params{Model::param_vector_t::Zero()};

  for (size_t k = 0; k < K - 1; ++k)
  {
    // Compute feedback control using the Lyapunov matrices X[k] = sum(theta_i*X_i) and Y...
    // respectively. We need the nonlinear theta parameters that represent the nonlinear
    // terms in the xdot = A(theta)*x + B(theta)*u

    // Update the error model states. [ey, e_yaw, eV, delta]
    x_error << xk.middleRows(4, Model::estate_dim);

    auto const &vtarget =
      nmpc_data.target_reference_states_and_controls.X[k](ns_utils::toUType(VehicleStateIds::vx));

    // [ey, epsi, error_vx, delta]
    x_error(2) = xk(ns_utils::toUType(VehicleStateIds::vx)) - vtarget;

    // ns_utils::print("In feedback vx vs vtarget", xk(ns_utils::toUType(VehicleStateIds::vx)),
    // vtarget);

    // Get the s-state (distance travelled) and interpolate for the curvature value at this distance
    // point.

    auto const &s0 = xk(ns_utils::toUType(VehicleStateIds::s));
    auto kappa0 = piecewise_interpolator.interpolatePoint(s0);

    ns_utils::print("curvature and x_error in feedback", kappa0);
    ns_eigen_utils::printEigenMat(Eigen::MatrixXd(x_error));

    // Compute the state transition matrices to get the values of the nonlinear terms
    // in the state transition mat Ac.

    params(ns_utils::toUType(VehicleParamIds::curvature)) = kappa0;
    params(ns_utils::toUType(VehicleParamIds::target_vx)) = vtarget;
    model_ptr->computeJacobians(xk, uk, params, Ac, Bc);

    // Compute the thetas - values of the non-linearities in the state transition matrix Ac.
    // We use the only one-block Ac where the error states reside.
    // auto const & Ac_error_block = Ac.template block<estate_dim, estate_dim>(4, 4);
    auto const &Ac_error_block = Ac.block<4, 4>(4, 4);

    ns_eigen_utils::printEigenMat(Ac, "Ac in LPV");
    ns_eigen_utils::printEigenMat(Bc, "Bc in LPV");

    auto const &th1 = Ac_error_block(0, 1);
    auto const &th2 = Ac_error_block(0, 2);

    auto const &th3 = Ac_error_block(1, 0);
    auto const &th4 = Ac_error_block(1, 1);
    auto const &th5 = Ac_error_block(1, 2);
    auto const &th6 = Ac_error_block(1, 3);

    // Compute parametric Lyapunov matrices.
    /**
     * X = sum(theta_i * X_i).
     **/

    thetas_ = std::vector<double>{th1, th2, th3, th4, th5, th6};

    // Extract the first X0, Y0, we save the first X0 and Y0 at the end.
    Xr = params_lpv.lpvXcontainer.back();  // We keep the first X0, Y0 at the end of the
    Yr = params_lpv.lpvYcontainer.back();

    for (size_t j = 0; j < ntheta_ - 1; j++)
    {
      Xr += thetas_[j] * params_lpv.lpvXcontainer[j];
      Yr += thetas_[j] * params_lpv.lpvYcontainer[j];
    }

    // Compute Feedback coefficients.
    auto const &Pr = Xr.inverse();  // Cost matrix P.
    auto const &Kfb = Yr * Pr;      // State feedback coefficients matrix.
    uk << Kfb * x_error;             // Feedback control signal.

    for (auto j = 0; j < uk.size(); ++j)
    {
      uk(j) = ns_utils::clamp(uk(j), params_opt.ulower(j), params_opt.uupper(j));
    }

    ns_sim::simulateNonlinearModel_zoh(model_ptr, uk, params, dt, xk);

    // Saturate all states
    for (auto j = 0; j < xk.size(); ++j)
    {
      if (params_opt.xlower(j) > -kInfinity && params_opt.xupper(j) < kInfinity)
      {
        xk(j) = ns_utils::clamp(xk(j), params_opt.xlower(j), params_opt.xupper(j));
      }
    }

    // Unwrap error and yaw angles.
    xk(ns_utils::toUType(VehicleStateIds::yaw)) =
      ns_utils::angleDistance(xk(ns_utils::toUType(VehicleStateIds::yaw)));

    // Update xk, uk
    nmpc_data.trajectory_data.X[k + 1] << xk.eval();
    nmpc_data.trajectory_data.U[k] << uk.eval();
  }

  // Copy the last input
  nmpc_data.trajectory_data.U.rbegin()[0] = nmpc_data.trajectory_data.U.rbegin()[1];

  // DEBUG
  // Get trajectories as a matrix and print for debugging purpose.
  // TO-DO {ali}: disable the debugs.
  //  auto && Xtemp = ns_eigen_utils::getTrajectory(nmpc_data.trajectory_data.X);
  //  auto && Utemp = ns_eigen_utils::getTrajectory(nmpc_data.trajectory_data.U);
  //
  //  ns_utils::print("\nComputed LPV trajectories : ");
  //  ns_eigen_utils::printEigenMat(Xtemp.transpose());  // [x, y, psi, s, ey, epsi, vx, delta, vy]
  //
  //  ns_utils::print("\nComputed LPV trajectories U : ");
  //  ns_eigen_utils::printEigenMat(Utemp.transpose());

  return true;
}
