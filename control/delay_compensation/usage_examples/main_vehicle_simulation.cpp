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

#include "delay_observer.hpp"
#include "qfilters.hpp"

int main()
	{
		// Create a dummy output signal for ey, epsi and delta.
		double       tfinal{ 10. };     // signal length in time
		unsigned int frequency{ 40 };   // Hz
		double       dt{ 1. / frequency };
		
		
		// Create an inverse vehicle model for these signal channels with Q-filters.
		// First create Q-filter for ey.
		double cut_off_frequency_ey    = 20.; // [Hz]
		double cut_off_frequency_eyaw  = 15.; // [Hz]
		double cut_off_frequency_delta = 10.;
		
		
		int order_ey    = 3;    // order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
		int order_e_yaw = 2;    // order of the filter for yaw error.
		int order_delta = 1;    // order of stereing model.
		
		
		// Base class construction.
		StrongTypeDef<double, s_cut_off_tag> sf_cutoff_ey{ cut_off_frequency_ey };
		StrongTypeDef<double, s_cut_off_tag> sf_cutoff_eyaw{ cut_off_frequency_eyaw };
		StrongTypeDef<double, s_cut_off_tag> sf_cutoff_delta{ cut_off_frequency_delta };
		
		
		// Specialized Qfilters for ey and eyaw.
		Qfilter<state_vector_qfilter_ey_t>    qfilter_ey{ sf_cutoff_ey, order_ey, dt };
		Qfilter<state_vector_qfilter_e_yaw_t> qfilter_epsi{ sf_cutoff_eyaw, order_e_yaw, dt };
		Qfilter<state_vector_qfilter_delta>   qfilter_delta{ sf_cutoff_delta, order_delta, dt };
		
		// Create a nonlinear vehicle model.

#ifndef NDEBUG
		
		
		
		// Create and simulate an inverse vehicle model.
		/***
		 *   steering --> Vehicle Model --> ey, epsi, delta ---> Q * inverse vehicle model
		 *   ....  ey, epsi, delta ---> Q * inverse vehicle model --> ey, epsi, delta to subtract from refs.
		 * */
		
		std::cout << "In the DEBUG mode ... " << std::endl;
#else
		std::cout << "In the RELEASE mode " << std::endl;
#endif
		
		
		return 0;
	}