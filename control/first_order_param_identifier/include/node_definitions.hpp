//
// Created by ali on 28/07/22.
//

#ifndef PARAM_IDENTIFIER_FO_INCLUDE_NODE_DEFINITIONS_HPP_
#define PARAM_IDENTIFIER_FO_INCLUDE_NODE_DEFINITIONS_HPP_

#include "common/types.hpp"

// Delay Compensation Libraries.
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

} // namespace sys_id

#endif //PARAM_IDENTIFIER_FO_INCLUDE_NODE_DEFINITIONS_HPP_

