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

#ifndef PARAM_IDENTIFIER_FO_INCLUDE_NODE_DEFINITIONS_HPP_
#define PARAM_IDENTIFIER_FO_INCLUDE_NODE_DEFINITIONS_HPP_

#include "common/types.hpp"

#include "autoware_control_toolbox.hpp"
#include <chrono>

namespace sys_id
{
using namespace std::chrono_literals;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using tf_t = ns_control_toolbox::tf;
using ss_t = ns_control_toolbox::tf2ss;

struct sNodeParameters
{
  float64_t sys_dt{0.033};

  // robust adaptation laws
  bool8_t use_switching_sigma{};
  bool8_t use_deadzone{};
  bool8_t use_dynamic_normalization{};

  float64_t sigma_0{0.01};
  float64_t deadzone_threshold{};
  float64_t delta0_norm_{}; // normalization dynamics time-constant

  // projection options
  float64_t smoother_eps{};
  float64_t forgetting_factor{};

  // parameter variables (parameter to be identified).
  float64_t a_upper_bound{};
  float64_t a_lower_bound{};

  float64_t b_upper_bound{};
  float64_t b_lower_bound{};
  float64_t param_normalized_upper_bound{};

};

} // namespace sys_id

#endif //PARAM_IDENTIFIER_FO_INCLUDE_NODE_DEFINITIONS_HPP_

