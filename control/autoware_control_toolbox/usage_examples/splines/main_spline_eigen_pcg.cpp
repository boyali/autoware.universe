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

#include "splines/interpolating_spline_pcg.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/timekeep.hpp"
#include "utils_act/writetopath.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

int main()
{
  // Generate a sinusoidal signal with arc-length parametrization. This is our test signal.
  size_t const num_of_points = 100;  // number of points in the signal.

  // Generate x.
  std::vector<double> xvec = ns_utils::linspace(0.0, 10.0, num_of_points);

  // Generate y = sin(x).
  std::vector<double> yvec;

  std::transform(
    xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [](auto const & x) { return sin(x); });

  // Generate z = cos(x).
  std::vector<double> zvec;
  std::transform(
    xvec.cbegin(), xvec.cend(), std::back_inserter(zvec), [](auto const & x) { return cos(x); });

  // Arc-length parametrization.
  std::vector<double> dx;  // {1, 0.0};
  std::vector<double> dy;  // {1, 0.0};

  std::adjacent_difference(xvec.begin(), xvec.end(), std::back_inserter(dx));
  std::adjacent_difference(yvec.begin(), yvec.end(), std::back_inserter(dy));

  // Define arc-length cumsum()
  std::vector<double> svec;
  std::transform(
    dx.cbegin(), dx.cend(), dy.cbegin(), std::back_inserter(svec), [](auto dxi, auto dyi) {
      static double ds = 0.0;
      ds += std::hypot(dxi, dyi);

      return ds;
    });

  auto log_path = getOutputPath() / "spline_pcg";

  // Create new interpolating coordinates
  auto se_new = ns_utils::linspace(svec[0] - 2., svec.back() + 3., 50);

  //  writeToFile(log_path, xvec, "xe");
  //  writeToFile(log_path, yvec, "ye");
  //  writeToFile(log_path, zvec, "ze");
  //  writeToFile(log_path, svec, "se");
  //  writeToFile(log_path, se_new, "snew");

  /*
   In case we use random noise.
   Generate noise.
      std::random_device rd; std::default_random_engine
      generator{rd()}; std::normal_distribution<double> distribution(0.0, 0.3);
   */

  std::vector<double> yinterp;

  // Default spline type is 'spline'. We can choose 'line' as an additional implementation.
  auto interpolating_spline_aw = ns_splines::InterpolatingSplinePCG(3);

  const double timer_interpolation = tic();

  // Don't keep coefficients.
  auto is_interpolated = interpolating_spline_aw.Interpolate(svec, yvec, se_new, yinterp);
  ns_utils::print("Is interpolated :", is_interpolated);

  fmt::print("{:<{}}{:.2f}ms\n", "Time: PCG spline initialization:", 50, toc(timer_interpolation));

  writeToFile(log_path, yinterp, "yinterp");

  const double timer_interpolation2 = tic();
  std::vector<double> zinterp;

  // Set reuse coeffs true so that we can use the same coefficients for the repeated interpolation.
  // When creating the interpolator first time, we set reuse false so that it computes the coeffs.

  is_interpolated = interpolating_spline_aw.Interpolate(svec, zvec, se_new, zinterp);
  ns_utils::print("Is interpolated :", is_interpolated);

  fmt::print("{:<{}}{:.2f} ms\n", "Time: spline initialization:", 50, toc(timer_interpolation2));
  writeToFile(log_path, zinterp, "zinterp");

  // REUSE EXAMPLE.
  auto se_new2 = ns_utils::linspace(svec[0], svec.back(), 200);
  std::vector<double> zinterp2;

  is_interpolated = interpolating_spline_aw.Interpolate(svec, zvec, se_new2, zinterp2);
  ns_utils::print("Is interpolated :", is_interpolated);

  //  writeToFile(log_path, se_new2, "se_new2");
  //  writeToFile(log_path, zinterp2, "zinterp2");

  // Interpolate a scalar value.
  auto ti = se_new[10];
  double single_interp_point_zval{};

  interpolating_spline_aw.Interpolate(svec, zvec, ti, single_interp_point_zval);

  std::cout << " Expected interpolated value : " << zinterp[10] << std::endl;
  std::cout << " Single interpolated value : " << single_interp_point_zval << std::endl;

  /**
   * Two new interpolation is added, if the interpolator is initialized.
   * */
  interpolating_spline_aw.Initialize(svec, zvec);

  std::vector<double> zinterp_itemwise;
  for (auto const & s : se_new2) {
    double zint{};
    interpolating_spline_aw.Interpolate(s, zint);
    zinterp_itemwise.emplace_back(zint);
  }

  //  writeToFile(log_path, zinterp_itemwise, "zinterp_itemwise");
  // Initialized and vector interpolation
  auto se_new3 = ns_utils::linspace(svec[0] + 0.5, svec.back(), 30);

  std::vector<double> zvectorwise;
  interpolating_spline_aw.Interpolate(se_new3, zvectorwise);

  //  writeToFile(log_path, se_new3, "snew3");
  //  writeToFile(log_path, zvectorwise, "zvectorwise");

  //  LINEAR INTERPOLATION
  std::vector<double> ylinear;
  ns_splines::InterpolatingSplinePCG linear_interp(1);
  is_interpolated = interpolating_spline_aw.Interpolate(svec, yvec, se_new, ylinear);
  ns_utils::print("Is interpolated :", is_interpolated);

  // writeToFile(log_path, ylinear, "ylinear");

  // Constant vector interpolation.
  std::vector<double> yconst(svec.size(), 1.);
  ns_splines::InterpolatingSplinePCG constant_interp(3);
  constant_interp.Initialize(svec, yconst);

  double yconst0{};
  constant_interp.Interpolate(svec[2] + 0.01, yconst0);

  // assert(("Left must equal right : ", yconst[0] == yconst0));
  // ns_utils::print("Is equal : ", yconst[0], yconst0);
  return 0;
}
