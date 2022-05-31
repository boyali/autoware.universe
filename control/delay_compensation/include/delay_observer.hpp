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
#include "visibility_control.hpp"
#include "autoware_control_toolbox.hpp"
#include <eigen3/Eigen/Core>

// class __attribute__((__visibility__("default"))) DelayObserver
class CDOB_PUBLIC DelayObserver {
public:
	
	DelayObserver() = default;
	
	explicit DelayObserver(double const& wheelbase);

private:
	double wheelbase_{ 2.74 };
	
};


#endif // DELAY_COMPENSATION__DELAY_OBSERVER_H
