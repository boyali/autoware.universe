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

#include "splines/bspline_interpolator_templated.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/timekeep.hpp"
#include "utils_act/writetopath.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

int main()
{
  auto log_path = getOutputPath() / "bspline_interp_templated";

  // <--------------- Dimension Expansion -------------------------------->
  // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
  const size_t Nin = 120;
  const size_t Nout = 80;

  double const & add_noise = 1;

  std::random_device rd;
  std::default_random_engine generator{rd()};
  std::normal_distribution<double> distribution(0.0, 0.3);

  {
    // Generate x.
    double kx = 8;
    std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, Nin);

    // Generate y = sin(x).
    double cy = 10;
    std::vector<double> yvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const & x) {
      return cy * sin(x) + distribution(generator) * add_noise;
    });

    std::vector<double> zvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(zvec), [&](auto const & x) {
      return 2 * cos(x) - 3 * sin(x) + distribution(generator) * add_noise;
    });

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

    writeToFile(log_path, xvec, "xvec");
    writeToFile(log_path, yvec, "yvec");
    writeToFile(log_path, zvec, "zvec");
    writeToFile(log_path, svec, "svec");

    /*
     In case we use random noise.
    // Generate noise.
      std::random_device rd;
      std::default_random_engine generator{rd()};
      std::normal_distribution<double> distribution(0.0, 0.3);
     */

    // Create new interpolating coordinates
    auto snew = ns_utils::linspace(svec[0], svec.back(), 101);

    // Create a spline object from the x
    // Default spline type is 'spline'. We can choose 'line' as an additional implementation.

    // < -------------------------- FULL EIGEN IMPLEMENTATION --------------------------------->

    Eigen::MatrixXd xe;
    xe = Eigen::Map<Eigen::Matrix<double, Nin, 1>>(xvec.data());

    Eigen::MatrixXd ye(
      xe.unaryExpr([&](auto x) { return cy * sin(x) + distribution(generator) * add_noise; }));

    Eigen::MatrixXd ze(xe.unaryExpr(
      [&](auto x) { return 2 * cos(x) - 3 * sin(x) + distribution(generator) * add_noise; }));

    Eigen::MatrixXd se;
    se = Eigen::Map<Eigen::Matrix<double, Nin, 1>>(svec.data());

    // Eigen::Index input_size = Nin;
    auto senew = Eigen::VectorXd::LinSpaced(Nout, 0.0, se(se.rows() - 1));

    //  ns_eigen_utils::printEigenMat(se_new.bottomRows(se_new.rows() - 1));

    writeToFile(log_path, xe, "xe");
    writeToFile(log_path, ye, "ye");
    writeToFile(log_path, ze, "ze");
    writeToFile(log_path, se, "se");
    writeToFile(log_path, senew, "snew");

    // Evaluate Interpolator
    Eigen::MatrixXd yinterp(ye.rows(), ye.cols());

    // Create a new smoothing spline.
    double know_number_ratio = 0.3;
    ns_splines::BSplineInterpolatorTemplated<Nin, Nout> interpolating_bspline(
      know_number_ratio, true);

    // Different size multi-column interpolation. Expanding the data points.
    interpolating_bspline.InterpolateImplicitCoordinates(ye, yinterp);

    writeToFile(log_path, yinterp, "yinterp");

    // Evaluate another variable using the same Interpolator
    Eigen::MatrixXd zinterp(ye.rows(), ye.cols());

    interpolating_bspline.InterpolateImplicitCoordinates(ze, zinterp);
    writeToFile(log_path, zinterp, "zinterp");

    // Multi Dimensional Interpolation
    Eigen::MatrixXd yze(ye.rows(), 2);
    yze.col(0) = ye;
    yze.col(1) = ze;
    writeToFile(log_path, yze, "yze");

    Eigen::MatrixXd yzinterp(ye.rows(), 2);

    interpolating_bspline.InterpolateImplicitCoordinates(yze, yzinterp);
    writeToFile(log_path, yzinterp, "yzinterp");
  }

  // < ------------------- Given X, Y compute curvature by smoothing Bsplines ----------- >
  {
    // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
    // Generate x.
    double kx = 8;
    std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, Nin);

    // Generate y = sin(x).
    double cy = 10;
    std::vector<double> yvec;
    std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const & x) {
      return cy * sin(x) + distribution(generator) * 0;
    });

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

    // EIGEN IMPLEMENTATION.
    Eigen::MatrixXd xe;
    xe = Eigen::Map<Eigen::Matrix<double, Nin, 1>>(xvec.data());

    Eigen::MatrixXd ye(xe.unaryExpr([&](auto x) { return cy * sin(x) + distribution(generator); }));

    Eigen::MatrixXd ze(
      xe.unaryExpr([&](auto x) { return 2 * cos(x) - 3 * sin(x) + distribution(generator); }));
    // Generate arc length
    Eigen::MatrixXd se;
    se = Eigen::Map<Eigen::Matrix<double, Nin, 1>>(svec.data());

    // Compute original curvature.
    Eigen::MatrixXd dxdt(xe.rows(), 1);

    // First derivative
    dxdt.setConstant(kx);  // dxdt = k*x and kx is k of x
    //  std::cout << " dxdt " << std::endl;
    //  ns_eigen_utils::printEigenMat(dxdt);

    Eigen::MatrixXd dydt(xe.unaryExpr([&](auto x) { return cy * kx * cos(x); }));

    // Second derivative
    Eigen::MatrixXd dxdt2(xe.rows(), 1);
    dxdt2.setZero();

    Eigen::MatrixXd dydt2(xe.unaryExpr([&](auto x) { return -cy * kx * kx * sin(x); }));

    // compute r0, r1 as r0 = [dxdt, dydt] and r1[dxdt2, dydt2]
    auto rdt = ns_eigen_utils::hstack<double>(dxdt, dydt);
    auto rdt2 = ns_eigen_utils::hstack<double>(dxdt2, dydt2);

    // Cross product example.
    //  Eigen::MatrixXd cross_product(xe.rows(), 1);
    //  auto cross_product = ns_eigen_utils::crossProduct<double>(rdt, rdt2);

    // Curvature example.
    Eigen::MatrixXd curvature_original;  // (xe.rows(), 1);
    curvature_original = ns_eigen_utils::Curvature(rdt, rdt2);

    //  ns_eigen_utils::printEigenMat(curvature_original);
    writeToFile(log_path, curvature_original, "curvature_original");

    // Create a new smoothing spline.
    double know_number_ratio = 0.3;
    ns_splines::BSplineInterpolatorTemplated<Nin, Nout> interpolating_bspline(
      know_number_ratio, true);

    // Get xdot, ydot
    Eigen::MatrixXd rdot_interp(Nout, 2);  // [xdot, ydot]

    // Get xddot, yddot
    Eigen::MatrixXd rddot_interp(Nout, 2);  // [xddot, yddot]

    auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);
    interpolating_bspline.getFirstDerivative(xy_data, rdot_interp);
    interpolating_bspline.getSecondDerivative(xy_data, rddot_interp);

    writeToFile(log_path, rdot_interp, "rdot_interp");
    writeToFile(log_path, rddot_interp, "rddot_interp");

    // Curvature from the B-spline
    Eigen::MatrixXd curvature_bspline_smoother;
    curvature_bspline_smoother = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);
    writeToFile(log_path, curvature_bspline_smoother, "curvature_bspline_interpolator");
  }

  // < ----- Given X, Y compute curvature by smoothing Bsplines CIRCLE example ----------- >
  {
    // Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
    // Generate x.

    std::vector<double> theta = ns_utils::linspace<double>(-M_PI / 2, M_PI / 2, Nin);

    Eigen::MatrixXd theta_eg;
    theta_eg = Eigen::Map<Eigen::Matrix<double, Nin, 1>>(theta.data());

    // Define a curve radius
    double R = 100;

    // EIGEN IMPLEMENTATION.
    Eigen::MatrixXd xe(theta_eg.unaryExpr([&R](auto x) { return R * cos(x); }));

    Eigen::MatrixXd ye(theta_eg.unaryExpr([&R](auto x) { return R * sin(x); }));

    // Create a new smoothing spline.
    double know_number_ratio = 0.3;
    ns_splines::BSplineInterpolatorTemplated<Nin, Nout> interpolating_bspline(
      know_number_ratio, true);

    // Get xdot, ydot
    Eigen::MatrixXd rdot_interp(Nout, 2);  // [xdot, ydot]

    // Get xddot, yddot
    Eigen::MatrixXd rddot_interp(Nout, 2);  // [xddot, yddot]

    auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);
    interpolating_bspline.getFirstDerivative(xy_data, rdot_interp);
    interpolating_bspline.getSecondDerivative(xy_data, rddot_interp);

    // Curvature from the B-spline
    Eigen::MatrixXd curvature_bspline_smoother;
    curvature_bspline_smoother = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);
    writeToFile(log_path, curvature_bspline_smoother, "curvature_circle");
  }

  return 0;
}
