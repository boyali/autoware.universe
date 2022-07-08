// Copyright 2022 The Autoware Foundation.
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

#include "control/balance.hpp"

#include "control/act_definitions.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"

/**
 * @brief Reduces a matrix to a upper Hessenberg form. Source Lapack sgebal.f.
 * @brief P is always an Identity matrix.
 * */
void ns_control_toolbox::permute(Eigen::MatrixXd &)
{
  // will be implementing when necessary
}

/**
 * @brief Balances a matrix using the Lapack algorithms.
 * James, R., Langou, J. and Lowery, B.R., 2014. On matrix balancing and eigenvector computation.
 * arXiv preprint arXiv:1401.5766.
 *
 * Numerical Recipes in C: The Art of Scientific Computing, Second Edition Balancing Chapter 11.
 * */

void ns_control_toolbox::balance_a_matrix(Eigen::MatrixXd &A, Eigen::MatrixXd &Tsimilarity)
{
  const int p = 1;
  bool converged = false;

  do
  {
    converged = true;
    for (Eigen::Index k = 0; k < A.rows(); ++k)
    {
      double c = A.col(k).lpNorm<p>();
      double r = A.row(k).lpNorm<p>();
      double s = std::pow(c, p) + std::pow(r, p);
      double f = 1;

      // Tsimilarity = Eigen::MatrixXd::Identity(A.rows(), A.cols());

      if ((r > EPS) && (c > EPS))
      {
        // In the lapack implementation diagonal element is ignored in the norms.
        while (c < r / RADIX)
        {
          c *= RADIX;
          r /= RADIX;
          f *= RADIX;
        }
        while (c >= r * RADIX)
        {
          c /= RADIX;
          r *= RADIX;
          f /= RADIX;
        }
        if (std::pow(c, p) + std::pow(r, p) <= 0.95 * s)
        {
          converged = false;
          Tsimilarity(k, k) *= f;
          A.col(k) *= f;
          A.row(k) /= f;
        }
      }
    }
  } while (!converged);
}

double ns_control_toolbox::balance_symmetric(double const &a, double const &b)
{
  double f = 1;  // Condition number that multiplies the small argument and divides the larger one.
  double small_number{};
  double large_number{};

  if (a > b)
  {
    large_number = a;
    small_number = b;
  } else
  {
    large_number = b;
    small_number = a;
  }

  do
  {
    if (large_number < small_number)
    {
      break;
    }

    small_number *= RADIX;
    large_number /= RADIX;
    f *= RADIX;

  } while ((large_number > EPS && small_number > EPS));

  return f;
}
