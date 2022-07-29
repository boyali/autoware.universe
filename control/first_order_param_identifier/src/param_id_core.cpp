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
    param_lower_bound_{node_params.param_lower_bound},
    param_upper_bound_{node_params.param_upper_bound},
    deadzone_thr_{node_params.deadzone_threshold},
    delta0_norm_{node_params.delta0_norm_},
    use_switching_sigma_{node_params.use_switching_sigma},
    use_deadzone_{node_params.use_deadzone},
    use_dynamic_normalization_{node_params.use_dynamic_normalization}
{


  // Compute the normalization coefficients.
  c0_ = 2. / (param_upper_bound_ - param_lower_bound_);
  d0_ = 1. - c0_ * param_upper_bound_;

  // Initial estimate for the parameter: ahat_;
  ahat_ = (param_upper_bound_ + param_lower_bound_) / 2. - am_ / 2.;

  // Initialize the control models.
  // phi_dot[x, u] = am_ * phi + [x;u] filters the state and input.
  first_order_tf_models_ = tf_t{{1.,}, {1., am_}}; //  1/ (s + am)
  first_order_ss_models_ = ss_t(first_order_tf_models_);

  if (use_dynamic_normalization_)
  {
    dynamic_normalization_tf_model_ = tf_t{{1.,}, {1., deadzone_thr_}}; //  1/ (s + am)
    dynamic_normalization_s_model_ = ss_t(dynamic_normalization_tf_model_, dt_);
  }
}

float64_t ParamIDCore::updateParameterEstimate(const float64_t &x_measured, const float64_t &u_applied)
{
  return 0;
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

void ParamIDCore::normalizeTheParameterBounds()
{

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
} // namespace sys_id