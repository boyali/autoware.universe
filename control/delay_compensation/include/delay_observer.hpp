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

#ifndef DELAY_COMPENSATION__DELAY_OBSERVER_H
#define DELAY_COMPENSATION__DELAY_OBSERVER_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include "visibility_control.hpp"
#include "qfilters.hpp"
#include "autoware_control_toolbox.hpp"

// Lazy transfer function


// class __attribute__((__visibility__("default"))) DelayObserver
class CDOB_PUBLIC DelayObserver
	{
public:
	
	// Constructors.
	DelayObserver() = default;
	
	explicit DelayObserver(double const& wheelbase);


private:
	// Model parameters
	double m_wheelbase_{ 2.94 };
	double m_tau_steer_{};
	
	// Associated Qfilter parameters
	int    q_order_{ 1 }; // order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	double q_cut_off_frequency_{}; // Cut-off frequency in Hz.
	double q_time_constant_tau_{};
	
	// Qfilter transfer function.
	ns_control_toolbox::tf Qfilter_tf_{};
	
	// Model transfer function.
	ns_control_toolbox::tf G_{};
	
	
	// Inverse vehicle model with Q filtered.
	// Qfilters.
	
	};


#endif // DELAY_COMPENSATION__DELAY_OBSERVER_H
