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


class CDOB_PUBLIC VehicleKinematicModelLinearized {
public:
	
	// Using typedefs to make the code more readable and faster.
	VehicleKinematicModelLinearized() = default;
	
	explicit VehicleKinematicModelLinearized(double const& wheelbase);
	
	void getInitialStates(state_vector_vehicle_t& x0);


private:
	double                   wheelbase_{ 2.74 };
	std::vector<std::string> state_names_{ "ey", "eyaw", "delta" }; // state names.
	std::vector<std::string> control_names_{ "delta" }; // control names.
	
	// Initial state
	state_vector_vehicle_t x0_{ 0.0, 0.0, 0.0 };
	
	
};


/**
 * @brief An real-time polymorphic class for the inverse kinematic model of a vehicle with Qfilters.
 *
 * */
class CDOB_PUBLIC InverseModelwithQfilter {
public:

private:


};


#endif //DELAY_COMPENSATION_VEHICLE_KINEMATIC_ERROR_MODEL_HPP
