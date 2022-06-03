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

	QFilterBase(t_cutOffFreq const& cutOffFreq, const int& order, double const& dt);

	QFilterBase(t_timeConst const& timeConst, const int& order, double const& dt);

	// Member functions.
	void print_tf() const;

	void print_ss() const;

	[[nodiscard]] std::vector<double> num() const;

	[[nodiscard]] std::vector<double> den() const;

	// Filter data.
	[[nodiscard]] ns_control_toolbox::tf TF() const;

	void getTimeConstantCutOffFrq(double& tc, double& fc) const;

	[[nodiscard]] int order() const
	{
		return order_;
	}

protected:
	int order_{ 1 }; // order of the filter (denominator) as power ; 1/(tau*s + 1) ^ order.
	double cut_off_frequency_{}; // Cut-off frequency in Hz.
	double time_constant_tau_{};
	double dt_{}; // time step for filter discretization.
	ns_control_toolbox::tf tf_{}; // Transfer function of the q-filter.
	ns_control_toolbox::tf2ss ss_{}; // State space representation of the q-filter.

};

template<int Norder, typename state_t = Eigen::Matrix<double, Norder, 1>>
class CDOB_PUBLIC Qfilter : public QFilterBase
{
public:

	using QFilterBase::QFilterBase;

	// Member functions
	// xdot = f(x)
	state_t xknext_fx(double const& u);

	// Single output y. Uses Euler integration.
	double y_hx(double const& u);

	void print_x0() const;

	void set_initial_state_x0(state_t const& xval);

	void reset_initial_state_x0();


private:

	state_t x0_{ state_t::Zero() }; // Filter current state


};

/**
 * @brief given an control signal, computes xdot = f(x, u), x0 is stored as a state in.
 * @param u is a SISO input double.
 * */
template<int Norder, typename state_t>
state_t Qfilter<Norder, state_t>::xknext_fx(const double& u)
{

	auto xnext = ss_.Ad() * x0_ + ss_.Bd() * u;
	// ns_eigen_utils::printEigenMat(xdot);

	return xnext;
}

/**
* @brief given an control signal, computes y = h(x, u), x0 is stored as a state in. The Euler integration is used.
 *
* @param u is a SISO input double.
* */

template<int Norder, typename state_t>
double Qfilter<Norder, state_t>::y_hx(double const& u)
{


	// Compute y
	double y = (ss_.Cd() * x0_ + ss_.Dd() * u)(0);

	// Update x.
	x0_ = ss_.Ad() * x0_ + ss_.Bd() * u;


	return y;
}

template<int Norder, typename state_t>
void Qfilter<Norder, state_t>::print_x0() const
{
	ns_eigen_utils::printEigenMat(x0_);
}

template<int Norder, typename state_t>
void Qfilter<Norder, state_t>::set_initial_state_x0(state_t const& xval)
{
	x0_ = xval;
}

template<int Norder, typename state_t>
void Qfilter<Norder, state_t>::reset_initial_state_x0()
{
	x0_.setZero();

}

/**
 * @brief Temporarily copies the data from a Qfilter
 */

struct CDOB_PUBLIC s_filter_data
{
	s_filter_data() = default;

	explicit s_filter_data(QFilterBase const& Qf);

	// Data members.
	int order{};
	double time_constant{}; // @brief tau in 1/(tau*s + 1)^n.
	double cut_off_frq{}; // in Hz.
	ns_control_toolbox::tf TF{};
};

struct CDOB_PUBLIC s_model_g_data
{
	using pairs_t = std::pair<std::string_view, std::string_view>;
	using pairs_func_maps_t = std::unordered_map<std::string_view, func_type<double>>;
	using tf = ns_control_toolbox::tf;

	// Constructors.
	s_model_g_data() = default;

	s_model_g_data(pairs_t params_names,
	               pairs_func_maps_t funcs,
	               tf Gs);


	// Data members.
	pairs_t num_den_coeff_names{};
	pairs_func_maps_t funcs{};

	tf TF{};

};


//

#endif //DELAY_COMPENSATION_QFILTERS_HPP
