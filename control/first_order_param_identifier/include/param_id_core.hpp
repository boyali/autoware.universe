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
#ifndef PARAM_IDENTIFIER_FO_INCLUDE_PARAM_ID_CORE_HPP_
#define PARAM_IDENTIFIER_FO_INCLUDE_PARAM_ID_CORE_HPP_

#include "node_definitions.hpp"

namespace sys_id
{

class ParamIDCore
{
 public:
  ParamIDCore() = default;
  explicit ParamIDCore(sNodeParameters const &node_params);

  void printModels();

  float64_t updateParameterEstimate(float64_t const &x_measured, float64_t const &u_applied);

 private:
  float64_t dt_{};
  float64_t smoothing_eps_{};
  float64_t forgetting_factor_{};

  float64_t M0_{1.}; // @brief square of parameter upper bound (we normalize).

  float64_t param_lower_bound_{};
  float64_t param_upper_bound_{};

  // @brief normalization coefficients (c, d).
  float64_t c0_{};
  float64_t d0_{};

  // @brief parameter estimate.
  float64_t am_{1.}; // @brief a proxy to parameter estimate to prevent instability.
  float64_t ahat_{}; // @brief estimated parameter

  // @brief  booleans
  float64_t deadzone_thr_{};
  float64_t delta0_norm_{};
  bool8_t use_switching_sigma_{};
  bool8_t use_deadzone_{};
  bool8_t use_dynamic_normalization_{};

  // @brief control models
  tf_t first_order_tf_models_{}; // @brief phi[x, u, z],
  ss_t first_order_ss_models_{};

  tf_t dynamic_normalization_tf_model_{};
  ss_t dynamic_normalization_s_model_{};

  void normalizeTheParameterBounds();

};

} // namespace sys_id

#endif //PARAM_IDENTIFIER_FO_INCLUDE_PARAM_ID_CORE_HPP_
