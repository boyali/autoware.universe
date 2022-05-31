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

#include <iostream>
#include "delay_observer.hpp"
#include "qfilters.hpp"
#include "vehicle_models/vehicle_kinematic_error_model.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"

int main()
	{
		double const wheelbase{ 2.9 };

		// TEST DELAY OBSERVER MAIN
		DelayObserver observer(wheelbase);
		std::cout << "hello world" << std::endl;


		// Test autoware control toolbox
		std::vector<double> num{ -1, -1, 5 };
		std::vector<double> den{ 1, 5, 1 };

		// With a num, den
		ns_control_toolbox::tf sys(num, den);

		// Print sys
		sys.print();

		// TEST QFilterBase constructors.
		StrongTypeDef<double, s_cut_off_tag> sf_cutoff{ 0.1 };
		QFilterBase                          qfilter_cutoff(sf_cutoff, 4); // 4th order low-pass filter

		StrongTypeDef<double, s_time_const_tag> sf_tconstant{ 0.1 };
		QFilterBase                             qfilter_timeconstant(sf_tconstant, 4); // 4th order low-pass filter

		// TEST VEHICLE MODEL

		// Create a vehicle model for the qfilter inversion.
		VehicleKinematicModelLinearized model_1;
		VehicleKinematicModelLinearized vehicle_model{ wheelbase };

		state_vector_vehicle_t x0_;
		x0_.setZero();
		vehicle_model.getInitialStates(x0_);

		ns_utils::print("x0_ .....; \n");

		ns_eigen_utils::printEigenMat(x0_);

		return 0;
	}