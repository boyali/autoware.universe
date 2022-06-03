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
                                                               double const& tau_steer,
                                                               double const& deadtime_vel,
                                                               double const& deadtime_steer)
		: wheelbase_{ wheelbase },
		  tau_steer_{ tau_steer },
		  tau_vel_{ tau_vel },
		  dead_time_steer_{ deadtime_steer },
		  dead_time_vel_{ deadtime_vel }
{
	size_t order_pade = 2;
	auto tf_steer_input = ns_control_toolbox::pade(deadtime_steer, order_pade);
	auto tf_vel_input = ns_control_toolbox::pade(deadtime_vel, order_pade);

	// Create the state space models for the dead-times.
	auto deadtime_vel_ss = ns_control_toolbox::tf2ss(tf_vel_input);
	auto deadtime_steer_ss = ns_control_toolbox::tf2ss(tf_steer_input);

	deadtime_vel_ss_discrete_ = deadtime_vel_ss.get_ssABCD_discrete();
	deadtime_steer_ss_discrete_ = deadtime_steer_ss.get_ssABCD_discrete();

	// Prepare delay initial states.
	x0_delay_vel_ = Eigen::MatrixXd(order_pade, 1);
	x0_delay_steer_ = Eigen::MatrixXd(order_pade, 1);


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

	auto use_vel_deadtime = ns_utils::isEqual(dead_time_vel_, 0.);
	auto use_steer_deadtime = ns_utils::isEqual(dead_time_steer_, 0.);

	double udelayed_vel{ desired_velocity };
	double udelayed_steer{ desired_steering };

	if (use_vel_deadtime)
	{
		auto&& Ad = deadtime_vel_ss_discrete_.A;
		auto&& Bd = deadtime_vel_ss_discrete_.B;
		auto&& Cd = deadtime_vel_ss_discrete_.C;
		auto&& Dd = deadtime_vel_ss_discrete_.D;

		x0_delay_vel_.noalias() = Ad * x0_delay_vel_ + Bd * desired_velocity;
		udelayed_vel = (Cd * x0_delay_vel_ + Dd * desired_velocity)(0);

	}


	if (use_steer_deadtime)
	{
		auto&& Ad = deadtime_steer_ss_discrete_.A;
		auto&& Bd = deadtime_steer_ss_discrete_.B;
		auto&& Cd = deadtime_steer_ss_discrete_.C;
		auto&& Dd = deadtime_steer_ss_discrete_.D;

		x0_delay_steer_.noalias() = Ad * x0_delay_steer_ + Bd * desired_steering;
		udelayed_vel = (Cd * x0_delay_steer_ + Dd * desired_steering)(0);
	}


	auto&& Vd = udelayed_vel;
	auto&& delta_d = udelayed_steer;

	// Get the previous states.
	auto&& ey0 = x0_[0];
	auto&& eyaw0 = x0_[1];
	auto&& delta0 = x0_[2];
	auto&& V0 = x0_[3];

	x0_[0] = ey0 + dt * V0 * sin(eyaw0);
	x0_[1] = eyaw0 + dt * (V0 / wheelbase_) * (tan(delta0) - tan(delta_d));
	x0_[2] = delta0 - dt * (1 / tau_steer_) * (delta0 - delta_d);
	x0_[3] = V0 - dt * (1 / tau_vel_) * (V0 - Vd);

	return x0_;
}
