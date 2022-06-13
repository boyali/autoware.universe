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

#include "adaptive_parameter_estimators/adaptive_parameter_estimation.hpp"
// observers::AdaptiveParameterEstimator::AdaptiveParameterEstimator()
// {
//
// }

void observers::AdaptiveParameterEstimator::Jacobian(autoware::common::types::float64_t const&
/*ymeasured*/, autoware::common::types::float64_t const& /*ypredicted*/)
{

}

observers::AdaptiveParameterEstimator::AdaptiveParameterEstimator(float64_t const& amin,
                                                                  float64_t const& amax,
                                                                  float64_t const& bmin,
                                                                  float64_t const& bmax,
                                                                  float64_t const& dt) :
	dt_{ dt },
	amax_{ amax },
	amin_{ amin },
	bmax_{ bmax },
	bmin_{ bmin }
{
	P_.setIdentity();
	C0_.setZero();

// Compute normalization coefficients.
	C0_(0, 0) = 2. / (amax_ - amin_);
	C0_(1, 1) = 2. / (bmax_ - bmin_);

	C1_(0, 0) = 1 - C0_(0, 0) * amax_;
	C1_(1, 0) = 1 - C0_(1, 1) * bmax_;

	phi_.setZero(); // filtered x and u
	I_.setIdentity(); // Identity matrix
	Z_.setZero(); // zero matrix.

	theta_ab_(0, 0) = (amax_ + amin_) / 2.;
	theta_ab_(1, 0) = (bmax_ + bmin_) / 2.;

}

bool observers::AdaptiveParameterEstimator::isProjectionNeeded(Eigen::Matrix<double, 2, 1> const& Pe_phi)
{

	// auto projection_results = Pe_phi;

	auto const&& normalized_parameters = C0_ * theta_ab_ + C1_;
	auto grad_g = theta_ab_; // gradient of constraint equation theta**2 -M<0

	auto theta_dot_product = normalized_parameters.transpose() * normalized_parameters;

	bool const&& condition_1 = theta_dot_product < M0_;

	auto const&& Pg = Pe_phi.transpose() * grad_g;
	bool const&& condition_2 = (ns_utils::isEqual(static_cast<double>(theta_dot_product), M0_)) && (Pg <= 0.);

// 	if (condition_1 || condition_2) // apply projection
// 	{
//
// 		// Compute C(theta) smoothing factor function.
// 		auto ctheta = (normalized_parameters.norm() - 1) / epsilon_;
//
// 		projection_results =
// 			(I_ - ctheta * P_ * (grad_g * grad_g.transpose()) / (grad_g.transpose() * P_ * grad_g)) * Pe_phi;
//
// 	}

	ns_utils::print("\nnormalized parameters ");
	ns_eigen_utils::printEigenMat(normalized_parameters);

	return condition_1 || condition_2;

}

void observers::AdaptiveParameterEstimator::updateEstimates(autoware::common::types::float64_t const& x,
                                                            autoware::common::types::float64_t const& u)
{

	// Compute the current error.
	// TODO: replace it with the sufficient richness conditions.
	double eps_pe = 1e-10;
	if (std::fabs(x) > eps_pe && std::fabs(u) > eps_pe)
	{
		auto ns = u; // normalization factor
		auto ms = 1. + ns * ns * 9.;

		// update ehat;
		ehat0_ = x - xhat0_ - zhat0_;

		// update parameter estimates.
		Eigen::Matrix<double, 2, 1> theta_dot = P_ * ehat0_ * phi_;
		auto Pdot = Z_; // prepare a zero matrix.

		// ------------------ Projection Block --------------------------------
		auto const&& normalized_parameters = C0_ * theta_ab_ + C1_;
		auto grad_g = theta_ab_; // gradient of constraint equation theta**2 -M<0

		auto normalized_theta_dot_product = normalized_parameters.transpose() * normalized_parameters;

		bool const&& condition_1 = normalized_theta_dot_product < M0_;

		auto const&& Pg = theta_dot.transpose() * grad_g;
		bool const
			&& condition_2 = (ns_utils::isEqual(static_cast<double>(normalized_theta_dot_product), M0_)) && (Pg <= 0.);

		if (!(condition_1 || condition_2))
		{
			auto ctheta = (normalized_parameters.lpNorm<2>() - 1) / epsilon_;

			// Project theta_dot.
			theta_dot.noalias() = (I_
				- ctheta * P_ * (grad_g * grad_g.transpose()) / (grad_g.transpose() * P_ * grad_g)) * theta_dot;

			// Do nothing to Pdot leave it as zero.
		}
		else
		{
			// Project Pdot.
			Pdot = beta_ * P_ - P_ * phi_ * phi_.transpose() * P_ / (ms * ms);
		}

		// ------------------ Projection Block Ends ---------------------------

		// Update theta_ab.
		theta_ab_.noalias() = theta_ab_ + dt_ * theta_dot;

		// Update the covariance.
		P_.noalias() = P_ + Pdot * dt_;

		// Reset the covariance matrix
		auto eigs = P_.eigenvalues();
		if ((std::abs(eigs(0, 0)) > rho_1 || std::abs(eigs(1, 0)) > rho_1) ||
			(std::abs(eigs(0, 0)) < rho_0 || std::abs(eigs(1, 0)) < rho_0))
		{

			P_ = rho_0 * I_;
		}

		// update phis
		auto phi_x_dot = -am_ * phi_(0, 0) + x;
		auto phi_u_dot = -am_ * phi_(1, 0) + u;

		phi_(0, 0) = phi_(0, 0) + dt_ * phi_x_dot;
		phi_(1, 0) = phi_(1, 0) + dt_ * phi_u_dot;

		// update zhat
		auto zhat_dot = -zhat0_ * am_ + ehat0_ * ns * ns;
		zhat0_ = zhat0_ + dt_ * zhat_dot;

		// update xhat0_;
		auto xhat0_dot = -am_ * x + theta_ab_(0, 0) * x + theta_ab_(1, 0) * u;
		xhat0_ = xhat0_ + dt_ * xhat0_dot;

		// debug
		ns_utils::print("ms : ", ms);

		ns_utils::print("\nCovariance Matrix : ");
		ns_eigen_utils::printEigenMat(P_);

		ns_utils::print("\nEigen Values of P_ : ");
		ns_eigen_utils::printEigenMat(eigs);

		ns_utils::print("\nEstimated ahat : ", am_ - theta_ab_(0, 0));
		ns_utils::print("\nEstimated bhat : ", theta_ab_(1, 0));
		ns_eigen_utils::printEigenMat(P_);

	}
	else
	{
		return;
	}

}
void observers::AdaptiveParameterEstimator::getCurrentEstimates_ab(autoware::common::types::float64_t& a_estimate,
                                                                   autoware::common::types::float64_t& b_estimate)
{
	if (!ns_utils::isEqual(am_ - theta_ab_(0, 0), 0.0))
	{
		a_estimate = 1. / (am_ - theta_ab_(0, 0));
	}

	if (!ns_utils::isEqual(theta_ab_(1, 0), 0.0))
	{
		b_estimate = 1. / theta_ab_(1, 0);
	}

}
