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

#ifndef DELAY_COMPENSATION_VEHICLE_DEFINITIONS_HPP
#define DELAY_COMPENSATION_VEHICLE_DEFINITIONS_HPP

#include "eigen3/Eigen/Core"
#include "utils_delay_observer/delay_compensation_utils.hpp"
#include "visibility_control.hpp"

/**
 * @brief Kinematic Vehicle Lateral Error Model state and control definitions.
 * */
enum class CDOB_PUBLIC KinematicErrorDims : int
{
	STATE_DIM = 3,
	INPUT_DIM = 1
};

using state_vector_vehicle_t = Eigen::Matrix<double, toUnderlyingType(KinematicErrorDims::STATE_DIM), 1>;
using input_vector_vehicle_t = Eigen::Matrix<double, toUnderlyingType(KinematicErrorDims::INPUT_DIM), 1>;


// General Template for enum class types.
template<int Nnum_of_states>
using state_vector_qfilter = Eigen::Matrix<double, Nnum_of_states, 1>;

template<typename T>
using func_type = std::function<T(T)>;

template<int Norder>
struct MatTypes
{
	using Atype = Eigen::Matrix<double, Norder, Norder>;
	using Btype = Eigen::Matrix<double, Norder, 1>;
	using Ctype = Eigen::Matrix<double, 1, Norder>;
	using Dtype = Eigen::Matrix<double, 1, 1>;
};


//constexpr Eigen::Index STATE_DIM = 3;
//constexpr Eigen::Index INPUT_DIM = 1;
#endif //DELAY_COMPENSATION_VEHICLE_DEFINITIONS_HPP
