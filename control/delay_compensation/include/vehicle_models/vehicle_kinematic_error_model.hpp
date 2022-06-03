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

#ifndef DELAY_COMPENSATION_VEHICLE_KINEMATIC_ERROR_MODEL_HPP
#define DELAY_COMPENSATION_VEHICLE_KINEMATIC_ERROR_MODEL_HPP

#include <vector>
#include <string>
#include <memory>
#include "visibility_control.hpp"
#include "vehicle_definitions.hpp"
#include "utils_delay_observer/delay_compensation_utils.hpp"
#include <eigen3/Eigen/Core>
#include <iostream>
#include "autoware_control_toolbox.hpp"
#include "qfilters.hpp"


/**
 * @brief Implemented to test the current packages and inverted model performance.
 * */
class CDOB_PUBLIC NonlinearVehicleKinematicModel
{
public:

	// Constructors.
	NonlinearVehicleKinematicModel() = default;

	NonlinearVehicleKinematicModel(double const& wheelbase,
	                               double const& tau_vel,
	                               double const& tau_steer,
	                               double const& deadtime_vel,
	                               double const& deadtime_steer,
	                               double const& dt);


	// Public methods.
	std::array<double, 4> simulateOneStep(const double& desired_velocity,
	                                      double const& desired_steering);

	void getInitialStates(std::array<double, 4>& x0);


private:
	double wheelbase_{ 2.74 };
	double tau_steer_{};
	double tau_vel_{};
	double dead_time_steer_{ 0 };
	double dead_time_vel_{ 0 };
	double dt_{ 0.1 };

	// Bool agains static gain discretizaiton.
	bool is_discretisize_vel_delay_{ false };
	bool is_discretisize_steering_delay_{ false };


	std::vector<std::string> state_names_{ "ey", "eyaw", "delta", "V" }; // state names.
	std::vector<std::string> control_names_{ "desired_vel", "delta_desired" }; // control names.

	// Deadtime inputs
	ns_control_toolbox::tf2ss deadtime_velocity_model_{};
	ns_control_toolbox::tf2ss deadtime_steering_model_{};

	// Initial state
	std::array<double, 4> x0_{ 0., 0., 0., 20. }; // this state is updated.

	// delayed input states.
	Eigen::MatrixXd xv0_{ Eigen::MatrixXd::Zero(2, 1) }; // delayed speed input states
	Eigen::MatrixXd xs0_{ Eigen::MatrixXd::Zero(2, 1) }; // delayed steeering input states.

};


#endif //DELAY_COMPENSATION_VEHICLE_KINEMATIC_ERROR_MODEL_HPP
