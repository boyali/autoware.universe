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

NonlinearVehicleKinematicModel::NonlinearVehicleKinematicModel(double const& wheelbase,
                                                               double const& tau_vel,
                                                               double const& tau_steer) : wheelbase_{ wheelbase },
                                                                                          tau_steer_{ tau_steer },
                                                                                          tau_vel_{ tau_vel }
	{
	
	}

void NonlinearVehicleKinematicModel::getInitialStates(std::array<double, 4>& x0)
	{
		x0 = x0_;
		
	}

/**
 *@brief Integrates the nonlinear dynamics one-step.
 * */
std::array<double, 4> NonlinearVehicleKinematicModel::simulateOneStep(const double& desired_velocity,
                                                                      double const& desired_steering,
                                                                      double const& dt)
	{
		auto&& Vd      = desired_velocity;
		auto&& delta_d = desired_steering;
		
		// Get the previous states.
		auto&& ey0    = x0_[0];
		auto&& eyaw0  = x0_[1];
		auto&& delta0 = x0_[2];
		auto&& V0     = x0_[3];
		
		x0_[0] = ey0 + dt * V0 * sin(eyaw0);
		x0_[1] = eyaw0 + dt * (V0 / wheelbase_) * (tan(delta0) - tan(delta_d));
		x0_[2] = delta0 - dt * (1 / tau_steer_) * (delta0 - delta_d);
		x0_[3] = V0 - dt * (1 / tau_vel_) * (V0 - Vd);
		
		return x0_;
	}
