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

template<typename eigenT>
DelayCompensator<eigenT>::DelayCompensator(s_filter_data const& Qfilter_data, s_model_G_data& Gdata) :
		q_cut_off_frequency_{ Qfilter_data.cut_off_frq },
		q_time_constant_tau_{ Qfilter_data.time_constant },
		Qfilter_tf_(Qfilter_data.TF),
		num_den_constant_names_G_{ Gdata.num_den_coeff_names },
		G_(Gdata.TF)
{

	// Compute Q/G
	auto tempG = std::move(Gdata.TF);
	tempG.inv();

	QGinv_ = Qfilter_tf_ * tempG;

	// Invert the num den constant names.
	num_den_constant_names_QGinv_ = pairs(num_den_constant_names_G_.second, num_den_constant_names_G_.first);
}

template<typename eigenT>
void DelayCompensator<eigenT>::print() const
{
	ns_utils::print("Delay Compensator Summary :  \n");
	ns_utils::print("Qfilter Model : \n");
	Qfilter_tf_.print();

	ns_utils::print("Forward model G(s) : \n\n");
	G_.print();

	ns_utils::print("G(s) num and den constant names :", num_den_constant_names_G_.first,
	                num_den_constant_names_G_.second, "\n");

	ns_utils::print("Forward model Q/G(s) :  \n");
	QGinv_.print();

	ns_utils::print("Q/G(s) num and den constant names :", num_den_constant_names_QGinv_.first,
	                num_den_constant_names_QGinv_.second);


}

template<typename eigenT>
std::array<double, 4> DelayCompensator<eigenT>::simulateOneStep()
{
	return std::array<double, 4>();
}
