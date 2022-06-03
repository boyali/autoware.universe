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

#include "qfilters.hpp"

#include <utility>
#include "visibility_control.hpp"

/**
 * @brief Construct a new Qfilters::Qfilters object with a cut-off frequency type
 * @param sCutOffFrequencyType in double to the cut-off frequency in Hz (e.g. 5.0)
 * @param order is the order of denominator in the transfer function.  *
 * */
QFilterBase::QFilterBase(StrongTypeDef<double, s_cut_off_tag> const& cutOffFrequency, int const& order,
                         double const& dt) : order_(order), cut_off_frequency_{ cutOffFrequency.get() }, dt_{ dt }
{
	// Calculate the time constant.

	auto w_c = 2.0 * M_PI * cut_off_frequency_; // in [rad/sec]



	// Put ROS Error message if zero division.
	// RCLCPP_ERROR(rclcpp::get_logger("mpc_utils"), "trajectory size has no consistency.");
//		RCLCPP_WARN_SKIPFIRST_THROTTLE(
//				logger, clock, 5000, "[calcNearestPoseInterp] fail to get nearest. traj.size = %zu",
//				traj.size());

//		RCLCPP_DEBUG(
//				get_logger(), "waiting data. current_steering = %d",
//				m_current_steering_ptr != nullptr);

	if (std::fabs(w_c) >= EPS)
	{
		time_constant_tau_ = 1.0 / w_c;
	}
	else
	{
		throw std::invalid_argument("The cut-off frequency cannot be zero.");
	}


	// Calculate the transfer function.
	// Calculate the transfer function.
	ns_control_toolbox::tf_factor denominator{ std::vector<double>{ time_constant_tau_, 1. }}; // (s+1)

	// Take power of the denominator.
	denominator.power(static_cast<unsigned int>(order));

	// Create the transfer function from a numerator an denominator.
	tf_ = ns_control_toolbox::tf{ std::vector<double>{ 1 }, denominator() };
	ss_ = ns_control_toolbox::tf2ss(tf_, dt); // Convert to state space.


	// DEBUG
	ns_utils::print("TF of Qfilter is constructed from cutoff ... ");
	tf_.print();

	ns_utils::print("SS of Qfilter is constructed from cutoff ... ");
	ss_.print();

}

/**
 * @brief Construct a new Qfilters::Qfilters object with a cut-off frequency type
 * @param sTimeConstantType in double to time constant of the filter ; 1/(tau*s + 1) ^ order.
 * @param order is the order of denominator in the transfer function.  *
 * */
QFilterBase::QFilterBase(QFilterBase::t_timeConst const& timeConst, int const& order, double const& dt) : order_{
		order }, time_constant_tau_{ timeConst.get() }, dt_{ dt }
{
	// Calculate the cut-off frequency.

	if (std::fabs(time_constant_tau_) >= EPS)
	{
		cut_off_frequency_ = 1.0 / time_constant_tau_; // in [rad/sec]
	}
	else
	{
		throw std::invalid_argument("The cut-off frequency cannot be zero.");
	}


	// Calculate the transfer function.
	ns_control_toolbox::tf_factor denominator{ std::vector<double>{ time_constant_tau_, 1. }}; // (s+1)

	// Take power of the denominator.
	denominator.power(static_cast<unsigned int>(order));

	// Create the transfer function from a numerator an denominator.
	tf_ = ns_control_toolbox::tf{ std::vector<double>{ 1 }, denominator() };
	ss_ = ns_control_toolbox::tf2ss(tf_, dt); // Convert to state space.

}

void QFilterBase::print_tf() const
{
	ns_utils::print("\n Transfer Function of the Q-filter \n");
	tf_.print();
}

void QFilterBase::print_ss() const
{
	// ns_utils::print("\n State Space Model of the Q-filter \n");
	ss_.print();
}

std::vector<double> QFilterBase::num() const
{
	return tf_.num();
}

std::vector<double> QFilterBase::den() const
{
	return tf_.den();
}

ns_control_toolbox::tf QFilterBase::TF() const
{
	return tf_;
}

void QFilterBase::getTimeConstantCutOffFrq(double& tc, double& fc) const
{
	tc = time_constant_tau_;
	fc = cut_off_frequency_;
}


// @brief Temporarily copies the data from a Qfilter
s_filter_data::s_filter_data(QFilterBase const& Qf) : order{ Qf.order() }
{
	Qf.getTimeConstantCutOffFrq(time_constant, cut_off_frq);
	TF = Qf.TF();
}

s_model_G_data::s_model_G_data(pairs_t params_names,
                               pairs_func_maps_t funcs, tf Gs) : num_den_coeff_names{ std::move(params_names) },
                                                                 funcs{ std::move(funcs) },
                                                                 TF{ std::move(Gs) }
{


}
