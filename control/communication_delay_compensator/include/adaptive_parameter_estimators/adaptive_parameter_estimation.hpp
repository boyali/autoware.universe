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

#ifndef DELAY_COMPENSATOR_INCLUDE_ADAPTIVE_PARAMETER_ESTIMATORS_ADAPTIVE_PARAMETER_ESTIMATION_HPP
#define DELAY_COMPENSATOR_INCLUDE_ADAPTIVE_PARAMETER_ESTIMATORS_ADAPTIVE_PARAMETER_ESTIMATION_HPP

#include "node_denifitions/node_definitions.hpp"

/**
 * Ioannou, P.A. and Sun, J., 1996. Robust adaptive control (Vol. 1). Upper Saddle River, NJ: PTR Prentice-Hall.
 * Chapter 4.4
 * */

namespace observers
{
		class AdaptiveParameterEstimatorFirstOrderSystem
		{
		public:
				AdaptiveParameterEstimatorFirstOrderSystem();
		private:
				float64_t dt_{};

				/**
				 * @brief Parameters to be updated for real-time estimation.
				 * hat{x_dot} = hat{a}x + hat{b}u where hat denotes the predicted variables.
				 *
				 * */
				std::array<float64_t, 2> theta_params_predicted_{}; // xdot = -a*x + bu where a and b ara unknown.

				// Value of Jacobian with respec to the parameters
				void Jacobian(float64_t const& ymeasured, float64_t const& ypredicted)

				// Class methods.
				/**
				 * @brief project a parameter on to a convex set, using vector projection methods.
				 * */
				void projetParameters();

		};
} // namespace observers
#endif //DELAY_COMPENSATOR_INCLUDE_ADAPTIVE_PARAMETER_ESTIMATORS_ADAPTIVE_PARAMETER_ESTIMATION_HPP
