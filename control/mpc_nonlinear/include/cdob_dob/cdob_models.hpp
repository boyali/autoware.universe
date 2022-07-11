/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_MODELS_HPP_
#define MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_MODELS_HPP_

#include <eigen3/Eigen/Core>
#include <Eigen/StdVector>
#include "cdob_dob/cdob_dob_definitions.hpp"
#include "utils/nmpc_utils_eigen.hpp"

namespace ns_cdob
{

/*
 * @brief : Vehicle model for disturbance observers
 */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
class LinearVehicleModelsBase
{
 public:

	using Atype = mat_type_t<STATE_DIM, STATE_DIM>;
	using Btype = mat_type_t<STATE_DIM, INPUT_DIM>;
	using Ctype = mat_type_t<MEASUREMENT_DIM, STATE_DIM>;
	using Dtype = mat_type_t<MEASUREMENT_DIM, INPUT_DIM>;
	using Stype = mat_type_t<STATE_DIM + INPUT_DIM, STATE_DIM + INPUT_DIM>;

	using state_vector_t = mat_type_t<STATE_DIM, 1>;
	using measurement_vector_t = mat_type_t<MEASUREMENT_DIM, 1>;

	// Constructors
	LinearVehicleModelsBase() = default;

	LinearVehicleModelsBase(double const &wheelbase, double const &tau_steering, double const &dt);

	// Destructors
	virtual ~LinearVehicleModelsBase() = default;

	virtual void updateStateSpace(double const &vr, double const &steer_r);

	void updateDisturbanceBw(double const &vr, double const &steer_r, double const &cos_sqr);

	void simulateOneStep(measurement_vector_t &y0, state_vector_t &x0, double const &u);

	void simulateOneStepZeroState(measurement_vector_t &y0, state_vector_t &x0, double const &u);

	void updateInitialStates(double const &ey,
													 double const &eyaw,
													 double const &steering,
													 double const &vx,
													 double const &curvature);

	void discretisizeBilinear();
	void discretisizeExact();
	void getIdealRefSteering(double &ref_steering) const;

	void printContinuousSystem();

	void printDiscreteSystem();

	[[nodiscard]] bool areInitialStatesSet() const
	{ return are_initial_states_set_; }

	[[nodiscard]] state_vector_t getInitialStates() const;

	void evaluateNonlinearTermsForLyap(state_vector_observer_t &thetas,
																		 measurement_vector_t const &y0) const;

	/**
	 * @gets the recent output y from the state observer. In case of disturbance, y does not contain this information,
	 * just the vehicle states.
	 * */
	void getObservedValues_y(state_vector_vehicle_t &y);

 protected:
	bool are_initial_states_set_{false};
	double wheelbase_{2.74};
	double tau_steering_{0.3};
	double dt_{0.1};

	// Continuous time state space matrices.
	Atype A_{Atype::Zero()};
	Btype B_{Btype::Zero()};
	Btype Bw_{Btype::Zero()};
	Ctype C_{Ctype::Identity()};
	Dtype D_{Dtype::Zero()};

	// Discrete time state space matrices.
	Atype Ad_{Atype::Zero()};
	Btype Bd_{Btype::Zero()};
	Btype Bwd_{Btype::Zero()};
	Ctype Cd_{Ctype::Zero()};
	Dtype Dd_{Dtype::Zero()};

	Atype I_At2_{Atype::Zero()};  // inv(I - A*ts/2)
	state_vector_t x0_;           // keep initial states.
	double long_velocity_{};
	double curvature_{};

	measurement_vector_t yobs_{measurement_vector_t::Zero()}; // to keep last observed value

};

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::printContinuousSystem()
{
	ns_nmpc_utils::print("Matrix A: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(A_));

	ns_nmpc_utils::print("Matrix B: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(B_));

	ns_nmpc_utils::print("Matrix Bw: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(Bw_));

	ns_nmpc_utils::print("Matrix C: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(C_));

	ns_nmpc_utils::print("Matrix D: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(D_));

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::printDiscreteSystem()
{

	ns_nmpc_utils::print("Matrix Ad: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(Ad_));

	ns_nmpc_utils::print("Matrix Bd: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(Bd_));

	ns_nmpc_utils::print("Matrix Bwd: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(Bwd_));

	ns_nmpc_utils::print("Matrix Cd: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(Cd_));

	ns_nmpc_utils::print("Matrix Dd: ");
	ns_nmpc_eigen_utils::printEigenMat(Eigen::MatrixXd(Dd_));

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::LinearVehicleModelsBase(const double &wheelbase,
																																												const double &tau_steering,
																																												const double &dt)
	: wheelbase_{wheelbase}, tau_steering_{tau_steering}, dt_{dt}
{


	// Assuming tau does not change.
	A_(2, 2) = -1. / tau_steering_;  // constant terms
	B_(2, 0) = 1. / tau_steering_;

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateStateSpace(const double &vr,
																																											const double &steer_r)
{

	auto const &cos_sqr = std::cos(steer_r) * std::cos(steer_r);
	auto const &L = wheelbase_;
	/**
	 * @brief
	 * A matrix
	 *          [ 0, V,                     0]
	 *          [ 0, 0, V/(L*cos(steering)^2)]
	 *          [ 0, 0,                  -1/tau]
	 * */

	/**
	 * Full nonlinear
	 * */
//  auto const &ey = x0_(0);
//  auto const &eyaw = x0_(1);
//  auto const &delta = x0_(2);
//
//  auto kterm = curvature_ / (1. - ey * curvature_);
//  auto ksqr = kterm * kterm;
//
//  A_(0, 1) = vr * cos(eyaw);
//  A_(1, 0) = -ksqr * vr * cos(eyaw);
//  A_(1, 1) = kterm * vr * sin(eyaw);
//  A_(1, 2) = vr / (L * cos_sqr);

	/** IF small angle assumption */
	A_(0, 1) = vr;
	A_(1, 2) = vr / (wheelbase_ * cos_sqr);
	updateDisturbanceBw(vr, steer_r, cos_sqr);

	/** Discretisize. */
	discretisizeBilinear();
	// discretisizeExact();

}

/**
* @brief Simulates the vehicle motion given an input. The resulting states and outputs are
* written into the passed arguments.
* @param y0: Output of the one step integration.
* @param x0: Vehicle initial states, updated after the integration.
* @param u: Steering control signal.
*
* */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::simulateOneStep(measurement_vector_t &y0,
																																										 state_vector_t &x0,
																																										 const double &u)
{

	// first update the output
	// y0 = x0 + dt_ * (A_ * x0 + B_ * steering_and_ideal_steering);
	y0 = Cd_ * x0.eval() + Dd_ * u;
	x0 = Ad_ * x0.eval() + Bd_ * u + Bwd_;

	yobs_ = y0;

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::simulateOneStepZeroState(measurement_vector_t &y0,
																																															state_vector_t &x0,
																																															const double &u)
{

	// first update the output
	// y0 = x0 + dt_ * (A_ * x0 + B_ * steering_and_ideal_steering);
	x0 = Ad_ * x0.eval() + Bd_ * u + Bwd_;
	y0 = Cd_ * x0.eval() + Dd_ * u;

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void
LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateInitialStates(const double &ey,
																																										const double &eyaw,
																																										const double &steering,
																																										const double &vx,
																																										const double &curvature)
{
	/**
	 * @brief The CDOBs use the linear vehicle model to access to its state space. The states of
	 * the parallel model is estimated by a state observer using this state-space.
	 * */
	x0_.topRows(3) << ey, eyaw, steering;

	long_velocity_ = vx;
	curvature_ = curvature;
	are_initial_states_set_ = true;

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
typename LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::state_vector_t
LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::getInitialStates() const
{
	return x0_;
}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::evaluateNonlinearTermsForLyap(
	state_vector_observer_t &thetas,
	measurement_vector_t const &y0) const
{

	// Extract values.
	auto const &ey = y0(0);
	auto const &eyaw = y0(1);
	auto const &steering = y0(2);

	auto const &&kappa_expr = curvature_ / (1. - curvature_ * ey);
	auto const &&ke_sqr = kappa_expr * kappa_expr;

	auto const &&t1 = long_velocity_ * std::cos(eyaw);

	auto const &&t2 = -ke_sqr * t1;
	auto const &&t3 = kappa_expr * long_velocity_ * std::sin(eyaw);
	auto const &&t4 = long_velocity_ / (wheelbase_ * std::cos(steering) * std::cos(steering));

	thetas << t1, t2, t3, t4;
}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::discretisizeBilinear()
{
	/**
	 *
	 * */

	auto const &I = Atype::Identity();
	I_At2_ = (I - A_ * dt_ / 2).inverse();

	Ad_ = I_At2_ * (I + A_ * dt_ / 2.);
	Bd_ = I_At2_ * B_ * dt_;
	Cd_ = C_ * I_At2_;
	Dd_ = D_ + C_ * Bd_.eval() / 2.;

	// Disturbance part
	Bwd_ = I_At2_ * Bw_ * dt_;
}
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::discretisizeExact()
{
	/**
	 * e^{[A, B; 0, 0]  = [Ad, Bd;0 1]}
	 *
	 */

	Stype E(Stype::Zero());
	E.template topLeftCorner<STATE_DIM, STATE_DIM>() = A_;
	E.template topRightCorner<STATE_DIM, INPUT_DIM>() = B_;
	Stype expE = (E * dt_).exp();

	Ad_ = expE.template topLeftCorner<STATE_DIM, STATE_DIM>();
	Bd_ = expE.template topRightCorner<STATE_DIM, INPUT_DIM>();

	// Compute Bw
	E.setZero();
	E.template topLeftCorner<STATE_DIM, STATE_DIM>() = A_;
	E.template topRightCorner<STATE_DIM, INPUT_DIM>() = Bw_;

	expE = (E * dt_).exp();
	Bw_ = expE.template topRightCorner<STATE_DIM, INPUT_DIM>();
	Cd_ = C_;
	Dd_ = D_;

}
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateDisturbanceBw(double const &vr,
																																												 double const &steer_r,
																																												 double const &cos_sqr)
{
	/**
	 * @brief  Bw = [0, 1/tau, 0]^T
	 *
	 * */

	auto const &L = wheelbase_;

	Bw_(1, 0) = vr * tan(steer_r) / L - vr * curvature_ - vr * steer_r / (L * cos_sqr); // WORKING BETTER
	//  Bw_(1, 0) = vr * tan(steer_r) / L - vr * curvature_ - steer_r / (L * cos_sqr);
	//  Bw_(1, 0) = -steer_r * vr / (L * cos_sqr);


}
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM,
														 INPUT_DIM,
														 MEASUREMENT_DIM>::getObservedValues_y(state_vector_vehicle_t &y)
{

	y(0) = yobs_(0);
	y(1) = yobs_(1);
	y(2) = yobs_(2);

}
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::getIdealRefSteering(double &ref_steering) const
{
	ref_steering = atan(curvature_ * wheelbase_);
}

/**
 * @brief Linear vehicle model for state observers
 * */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
class VehicleModelDisturbanceObserver : public LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>
{

	using BASE = LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>;

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Constructors.
	using BASE::LinearVehicleModelsBase::LinearVehicleModelsBase;
	void updateStateSpace(double const &vr, double const &steer_r) override;

};
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void VehicleModelDisturbanceObserver<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateStateSpace(const double &vr,
																																															const double &steer_r)
{
	auto const &cos_sqr = std::cos(steer_r) * std::cos(steer_r);
	auto const &L = this->wheelbase_;
	/**
	 * @brief
	 * A matrix
	 *          [ 0, V,                     0]
	 *          [ 0, 0, V/(L*cos(steering)^2)]
	 *          [ 0, 0,                  -1/tau]
	 * */

	//  auto const &x0_ = this->x0_;
	//  auto curvature = this->curvature_;
	//
	//  auto const &ey = x0_(0);
	//  auto const &eyaw = x0_(1);
	//  auto const &delta = x0_(2);
	//
	//  auto kterm = curvature / (1. - ey * curvature);
	//  auto ksqr = kterm * kterm;
	//
	//  this->A_(0, 1) = vr * cos(eyaw);
	//  this->A_(1, 0) = -ksqr * vr * cos(eyaw);
	//  this->A_(1, 1) = kterm * vr * sin(eyaw);
	//  this->A_(1, 2) = vr / (L * cos_sqr);

	/** Set -B */
	//  auto col_size = this->A_.cols();
	//  this->A_.col(col_size - 1) = -this->B_;

	/* IF small angle assumption */
	this->A_(0, 1) = vr;
	this->A_(1, 2) = vr / (L * cos_sqr);

	auto col_size = this->A_.cols();
	this->A_.col(col_size - 1) = -this->B_;

	this->updateDisturbanceBw(vr, steer_r, cos_sqr);

	/** Discretisize. */
	this->discretisizeBilinear();
	// this->discretisizeExact();

}

/**
* @brief Linear kinematic error vehicle model with three states [ey, eyaw, ]
* */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
class LinearKinematicErrorModel : public LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>
{
	using BASE = LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>;

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	using BASE::LinearVehicleModelsBase::LinearVehicleModelsBase;
	using BASE::LinearVehicleModelsBase::updateStateSpace;

};

using ns_nmpc_utils::toUType;

// Observer model types.
using linear_vehicle_model_t = LinearKinematicErrorModel<toUType(CDOB_KinematicErrorDims::STATE_DIM),
																												 toUType(CDOB_KinematicErrorDims::INPUT_DIM),
																												 toUType(CDOB_KinematicErrorDims::MEASUREMENT_DIM)>;

using linear_state_observer_model_t = VehicleModelDisturbanceObserver<toUType(StateObserverDims::STATE_DIM),
																																			toUType(StateObserverDims::INPUT_DIM),
																																			toUType(StateObserverDims::MEASUREMENT_DIM)>;

} // namespace ns_cdob

#endif //MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_MODELS_HPP_
