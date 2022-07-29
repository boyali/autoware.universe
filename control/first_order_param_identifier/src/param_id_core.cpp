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

#include "param_id_core.hpp"

namespace sys_id
{

ParamIDCore::ParamIDCore(const sNodeParameters &node_params)
  : dt_{node_params.sys_dt},
    smoothing_eps_{node_params.smoother_eps},
    forgetting_factor_{node_params.forgetting_factor},
    M0_{node_params.param_normalized_upper_bound},
    a_lower_bound_{node_params.a_lower_bound},
    a_upper_bound_{node_params.a_upper_bound},

    b_lower_bound_{node_params.b_lower_bound},
    b_upper_bound_{node_params.b_upper_bound},

    deadzone_thr_{node_params.deadzone_threshold},
    delta0_norm_{node_params.delta0_norm_},
    use_switching_sigma_{node_params.use_switching_sigma},
    use_deadzone_{node_params.use_deadzone},
    use_dynamic_normalization_{node_params.use_dynamic_normalization}
{


  // Compute the normalization coefficients.
  c0_(0, 0) = 2. / (a_upper_bound_ - a_lower_bound_);
  d0_(0) = 1. - c0_(0, 0) * a_upper_bound_;

  c0_(1, 1) = 2. / (b_upper_bound_ - b_lower_bound_);
  d0_(1) = 1. - c0_(1, 1) * b_upper_bound_;

  // Initial estimate for the parameter: am_ab_hat_;
  am_ab_hat_(0) = (a_upper_bound_ + a_lower_bound_) / 2. - am_ / 2.;
  am_ab_hat_(1) = (b_upper_bound_ + b_lower_bound_) / 2.;

  // Initialize the control models.
  // phi_dot[x, u] = am_ * phi + [x;u] filters the state and input.
  first_order_tf_models_ = tf_t{{1.,}, {1., am_}}; //  1/ (s + am)
  first_order_ss_models_ = ss_t(first_order_tf_models_);

  phi_x_ = Eigen::MatrixXd::Zero(first_order_tf_models_.order(), 1);
  phi_u_ = Eigen::MatrixXd::Zero(first_order_tf_models_.order(), 1);
  phi_ = Eigen::MatrixXd::Zero(2 * first_order_tf_models_.order(), 1);

  zhat_ = Eigen::MatrixXd::Zero(first_order_tf_models_.order(), 1);
  xhat_ = Eigen::MatrixXd::Zero(1, 1);

  if (use_dynamic_normalization_)
  {
    dynamic_normalization_tf_model_ = tf_t{{1.,}, {1., deadzone_thr_}}; //  1/ (s + am)
    dynamic_normalization_ss_model_ = ss_t(dynamic_normalization_tf_model_, dt_);
  }

  ms_x_ = Eigen::MatrixXd::Zero(1, 1);
}

void ParamIDCore::updateParameterEstimate(const float64_t &x_measured, const float64_t &u_applied,
                                          std::array<float64_t, 2> &ab_param_estimate)
{

  auto const &usqr = std::pow(u_applied, 2);
  float64_t ms_sqr{};
  float64_t ns_sqr{};

  if (use_dynamic_normalization_)
  {
    ns_sqr = dynamic_normalization_ss_model_.simulateOneStep(ms_x_, usqr);
    ms_sqr = 1. + usqr + ns_sqr;

  } else
  {
    ns_sqr = usqr; // normalization variable
    ms_sqr = 1. + ns_sqr;
  }

  // Compute the estimation error.
  auto const &ehat = x_measured - xhat_(0) + zhat_(0);

  // Compute theta_dot = gamma * ehat * phi
  Pdot_ *= 0; // Reset the derivative of the parameter estimate.
  Eigen::Vector2d theta_dot = P_ * ehat * phi_;

  // Normalize the estimated parameter
  auto const &ab_hat_normalized = getNormalizedEstimate();

  // Compute the smoothing term.
  auto const &csmoothing_term = (ab_hat_normalized.norm() - 1.) / smoothing_eps_;

  // Check if the parameter satisfies the projection requirements.
  if (auto const &needs_projections = needsProjection(theta_dot, ab_hat_normalized))
  {
    auto const &grad_theta = am_ab_hat_; // gradient of theta

    theta_dot = theta_dot -
                csmoothing_term * P_ * grad_theta * grad_theta.transpose() * theta_dot
                / (grad_theta.transpose() * P_ * grad_theta)(0);

  } else
  {
    Pdot_ = forgetting_factor_ * P_ - csmoothing_term * P_ * phi_ * phi_.transpose() * P_ / ms_sqr;
  }

  // update zhat0
  yout_zhat_ = first_order_ss_models_.simulateOneStep(zhat_, ehat * ns_sqr);

  //update phi
  yout_xhat_ = first_order_ss_models_.simulateOneStep(phi_x_, x_measured);
  yout_u_ = first_order_ss_models_.simulateOneStep(phi_u_, u_applied);

  phi_ << yout_xhat_, yout_u_;


  // update covariance
  P_.noalias() = P_ + dt_ * Pdot_;

  // update parameter estimates.
  am_ab_hat_ += theta_dot * dt_;

  // update xhat
  xhat_ = (am_ab_hat_.transpose() * phi_);


  // DEBUG
  //  ns_utils::print("x_measured: ", x_measured);
  //  ns_utils::print("u_applied: ", u_applied);

  ns_utils::print("Estimated a and b", am_ab_hat_(0), am_ab_hat_(1));
  ns_eigen_utils::printEigenMat(P_, "Covariance P");

  ab_param_estimate[0] = am_ab_hat_(0);
  ab_param_estimate[1] = am_ab_hat_(1);

}

/**
 * @brief normalize the parameter bounds [-1, 1].
 *
 *      xhat_max = c*xmax + d
 *      xhat_min = c*min + d
 *
 *      c = (xhat_max - xhat_min)/(xmax - xmin)
 *      d = xhat_min - c*xmin
 */

Eigen::Vector2d ParamIDCore::getNormalizedEstimate()
{
  return c0_ * am_ab_hat_ + d0_;

}
void ParamIDCore::printModels()
{

  ns_utils::print("The first order TF model: ");
  first_order_tf_models_.print();

  ns_utils::print("The first order SS model: ");
  first_order_ss_models_.print();

  if (use_dynamic_normalization_)
  {
    ns_utils::print("The dynamic normalization TF model: ");
    dynamic_normalization_tf_model_.print();
  }

}
bool8_t ParamIDCore::needsProjection(Eigen::MatrixXd const &theta_dot,
                                     Eigen::Vector2d const &ahat_normalized) const
{
  auto const &grad_ahat = ahat_normalized; // gradient of theta^2 - M < 0
  auto const &params_norm = ahat_normalized.norm();

  auto const &condition1 = params_norm < M0_;
  auto const &condition2 = (theta_dot * grad_ahat)(0) <= 0 && params_norm == M0_;

  return condition1 || condition2;

}
} // namespace sys_id