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

#ifndef DELAY_COMPENSATION_QFILTERS_HPP
#define DELAY_COMPENSATION_QFILTERS_HPP

#include <memory>
#include "autoware_control_toolbox.hpp"
#include "utils_delay_observer/delay_compensation_utils.hpp"
#include "vehicle_models/vehicle_definitions.hpp"
#include "visibility_control.hpp"


struct CDOB_PUBLIC s_cut_off_tag
	{
	};

struct CDOB_PUBLIC s_time_const_tag
	{
	};


class CDOB_PUBLIC QFilterBase
	{
	public:
		using t_cutOffFreq = StrongTypeDef<double, s_cut_off_tag>;
		using t_timeConst = StrongTypeDef<double, s_time_const_tag>;

		QFilterBase(t_cutOffFreq cutOffFreq, const int& order);

		QFilterBase(t_timeConst timeConst, const int& order);

		// Member functions.
		void print_tf() const;

		void print_ss() const;


	protected:
		int                       order_{ 1 }; // order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
		double                    cut_off_frequency_{}; // Cut-off frequency in Hz.
		double                    time_constant_tau_{};
		ns_control_toolbox::tf    tf_{}; // Transfer function of the q-filter.
		ns_control_toolbox::tf2ss ss_{}; // State space representation of the q-filter.

	};

template<typename eigenT>
class CDOB_PUBLIC Qfilter : public QFilterBase
	{
	public:

		using QFilterBase::QFilterBase;

		// Member functions
		// xdot = f(x)
		eigenT xdot_fx(double const& u);

		// Single output y. Uses Euler integration.
		double y_hx(double const& u, double const& dt);

		void print_x0() const;

		void set_initial_state_x0(eigenT const& xval);

		void reset_initial_state_x0();


	private:

		eigenT x0_{ eigenT::Zero() }; // Filter state
		eigenT xdot0_{ eigenT::Zero() }; // Filter xdot place holder
	};

/**
 * @brief given an control signal, computes xdot = f(x, u), x0 is stored as a state in.
 * @param u is a SISO input double.
 * */
template<typename eigenT>
eigenT Qfilter<eigenT>::xdot_fx(const double& u)
	{

		xdot0_ = ss_.A_ * x0_ + ss_.B_ * u;
		// ns_eigen_utils::printEigenMat(xdot);

		return xdot0_;
	}

/**
* @brief given an control signal, computes y = h(x, u), x0 is stored as a state in. The Euler integration is used.
 *
* @param u is a SISO input double.
* */

template<typename eigenT>
double Qfilter<eigenT>::y_hx(double const& u, const double& dt)
	{
		// Compute xdot.
		// auto&& xdot = xdot_fx(u);
		xdot0_ = ss_.A_ * x0_ + ss_.B_ * u;
		x0_.noalias() = x0_ + dt * xdot0_;

		// Compute y
		auto y = ss_.C_ * x0_ + ss_.D_ * u;

		// print xdot
//		print_x0();
//		ns_eigen_utils::printEigenMat(xdot);
//		print_x0();

		return y(0);
	}

template<typename eigenT>
void Qfilter<eigenT>::print_x0() const
	{

		ns_eigen_utils::printEigenMat(x0_);
	}

template<typename eigenT>
void Qfilter<eigenT>::set_initial_state_x0(eigenT const& xval)
	{
		x0_ = xval;
	}

template<typename eigenT>
void Qfilter<eigenT>::reset_initial_state_x0()
	{
		x0_.setZero();

	}

//

#endif //DELAY_COMPENSATION_QFILTERS_HPP
