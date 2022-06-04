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

#ifndef AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP
#define AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP

#include "visibility_control.hpp"
#include <vector>
#include "act_utils.hpp"
#include "act_utils_eigen.hpp"
#include "act_definitions.hpp"
#include "transfer_functions.hpp"
#include "balance.hpp"

namespace ns_control_toolbox
{


	/**
	 * @brief tf2ss Converts a transfer function representation in to a state-space form.
	 * We assume the system is SISO type.
	 *
	 * */

	class ACT_PUBLIC tf2ss
	{

	public:
		// Constructors
		tf2ss() = default;

		explicit tf2ss(tf const& sys_tf, const double& Ts = 0.1);

		tf2ss(std::vector<double> const& num, std::vector<double> const& den, const double& Ts = 0.1);


		// Public methods
		// Currently only Tustin - Bilinear discretization is implemented.
		void discretisize(double const& Ts);

		void print() const;

		void print_discrete_system() const;


		// Getters for the system matrices.
		// Discrete time state-space matrices.
		[[nodiscard]] Eigen::MatrixXd Ad() const
		{
			return Ad_;
		}

		[[nodiscard]] Eigen::MatrixXd Bd() const
		{
			return Bd_;
		}

		[[nodiscard]] Eigen::MatrixXd Cd() const
		{
			return Cd_;
		}

		[[nodiscard]] Eigen::MatrixXd Dd() const
		{
			return Dd_;
		}


		// Continuous time state-space matrices.
		[[nodiscard]] Eigen::MatrixXd A() const
		{
			return A_;
		}

		[[nodiscard]] Eigen::MatrixXd B() const
		{
			return B_;
		}

		[[nodiscard]] Eigen::MatrixXd C() const
		{
			return C_;
		}

		[[nodiscard]] Eigen::MatrixXd D() const
		{
			return D_;
		}

		// Class methods.

		/**
		 * @brief Compute the system continuous time system matrices
		 * */
		void computeSystemMatrices(std::vector<double> const& num,
		                           std::vector<double> const& den);


	private:

		double Ts_{};

		// Data members
		// Continuous time state-space model
		Eigen::MatrixXd A_{};
		Eigen::MatrixXd B_{};
		Eigen::MatrixXd C_{};
		Eigen::MatrixXd D_{};

		// Discrete time state-space model
		Eigen::MatrixXd Ad_{};
		Eigen::MatrixXd Bd_{};
		Eigen::MatrixXd Cd_{};
		Eigen::MatrixXd Dd_{};

	};



	// Type definition.

	template<int nx, int ny>
	using mat_type_t = Eigen::Matrix<double, nx, ny>;

} // namespace ns_control_toolbox
#endif //AUTOWARE_CONTROL_TOOLBOX_STATE_SPACE_HPP
