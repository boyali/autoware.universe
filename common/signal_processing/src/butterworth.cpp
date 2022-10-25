// Copyright 2022 Tier IV, Inc.
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

#include "signal_processing/butterworth.hpp"

#include <rclcpp/logging.hpp>

#include <iomanip>
#include <numeric>
#include <sstream>

/**
 *  @brief Computes the minimum of an analog Butterworth filter order and cut-off frequency give
 *  the pass and stop-band frequencies and ripple magnitude (tolerances).
 *  @param Wp [in] pass-band frequency [rad/sc],
 *  @param Ws [in] stop-band frequency [rad/sc],
 *  @param Ap [in] pass-band ripple [dB]
 *  @param As [in] stop-band ripple [dB]
 * */

void ButterworthFilter::Buttord(
  const double & Wp, const double & Ws, const double & Ap, const double & As)
{
  // N*ln(alpha) > ln(beta)
  auto alpha = Ws / Wp;
  auto beta = std::sqrt((std::pow(10, As / 10.0) - 1.0) / (std::pow(10, Ap / 10.0) - 1.0));
  auto order = static_cast<int>(std::ceil(std::log(beta) / std::log(alpha)));

  setOrder(order);

  // right limit, left limit
  /**
   * The left and right limits of the magnitudes satisfy the specs at the
   * frequencies Ws and Wp Scipy.buttord gives left limit as the cut-off frequency whereas Matlab
   * gives right limit. We keep left_lim as a definition and commented out.
   * */

  double right_lim = Ws * (std::pow((std::pow(10.0, As / 10.0) - 1.0), -1.0 / (2. * order)));
  // double left_lim = Wp * (std::pow((std::pow(10.0, Ap / 10.0) - 1.0), -1.0 / (2. * order)));

  setCuttoffFrequency(right_lim);
}

void ButterworthFilter::setOrder(const int & N) { order_ = N; }

void ButterworthFilter::setCuttoffFrequency(const double & Wc) { cutoff_frequency_rad_sec = Wc; }

/**
 * @brief Sets the cut-off and sampling frequencies.
 * @param fc [in] cut-off frequency in Hz.
 * @param fs [in] sampling frequency in Hz.
 * */
void ButterworthFilter::setCuttoffFrequency(const double & fc, const double & fs)
{
  /*
   * fc is the cut-off frequency in [Hz]
   * fs is the sampling frequency in [Hz]
   * */
  if (fc >= fs / 2) {
    print("Invalid argument : Cut-off frequency  fc must be less than fs/2 \n");
    return;
  }

  cutoff_frequency_rad_sec = fc * 2.0 * M_PI;
  sampling_frequency_hz = fs;
}

sOrderCutoff ButterworthFilter::getOrderCutOff() const
{
  return sOrderCutoff{order_, cutoff_frequency_rad_sec};
}

/**
 * @brief Matlab equivalent : [b, a]  = butter(n, Wn, 's')
 * */
void ButterworthFilter::computeContinuousTimeTF(const bool & use_sampling_frequency)
{
  // First compute  the phase angles of the roots
  computePhaseAngles();
  computeContinuousTimeRoots(use_sampling_frequency);

  continuous_time_denominator_ = poly(continuous_time_roots_);
  continuous_time_numerator_ = std::pow(cutoff_frequency_rad_sec, order_);
}

void ButterworthFilter::computePhaseAngles()
{
  phase_angles_.resize(order_, 0.);

  for (size_t i = 0; i < phase_angles_.size(); ++i) {
    auto & x = phase_angles_.at(i);
    x = M_PI_2 + (M_PI * (2.0 * static_cast<double>((i + 1)) - 1.0) / (2.0 * order_));
  }
}

void ButterworthFilter::computeContinuousTimeRoots(const bool & use_sampling_frequency)
{
  continuous_time_roots_.resize(order_, {0.0, 0.0});

  if (use_sampling_frequency) {
    const double & Fc = (sampling_frequency_hz / M_PI) *
                        tan(cutoff_frequency_rad_sec / (sampling_frequency_hz * 2.0));

    for (size_t i = 0; i < continuous_time_roots_.size(); ++i) {
      auto & x = continuous_time_roots_[i];
      x = {
        std::cos(phase_angles_[i]) * Fc * 2.0 * M_PI, std::sin(phase_angles_[i]) * Fc * 2.0 * M_PI};
    }
    return;
  }

  for (size_t i = 0; i < continuous_time_roots_.size(); ++i) {
    auto & x = continuous_time_roots_[i];
    x = {
      cutoff_frequency_rad_sec * cos(phase_angles_[i]),
      cutoff_frequency_rad_sec * sin(phase_angles_[i])};
  }
}
std::vector<std::complex<double>> ButterworthFilter::poly(
  std::vector<std::complex<double>> const & roots)
{
  std::vector<std::complex<double>> coefficients(roots.size() + 1, {0, 0});

  const int n{static_cast<int>(roots.size())};

  coefficients[0] = {1.0, 0.0};

  for (int i = 0; i < n; i++) {
    for (int j = i; j != -1; j--) {
      coefficients[j + 1] = coefficients[j + 1] - (roots[i] * coefficients[j]);
    }
  }

  return coefficients;
}

/**
 *  @brief Prints the order and cut-off angular frequency (rad/sec) of the filter
 * */

void ButterworthFilter::printFilterContinuousTimeRoots() const
{
  std::stringstream stream;
  stream << "\n The roots of the continuous-time filter Transfer Function's Denominator are : \n";

  for (const auto & x : continuous_time_roots_) {
    stream << std::fixed << std::setprecision(2) << std::real(x) << " j";

    auto txt = std::imag(x) < 0 ? " - j " : " + j ";
    stream << std::fixed << std::setprecision(2) << txt << std::abs(std::imag(x)) << " \n";
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", stream.str().c_str());
}
void ButterworthFilter::printContinuousTimeTF() const
{
  const auto & n = order_;

  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"), "\nThe Continuous Time Transfer Function of the Filter is ;\n");

  std::stringstream stream;
  stream << std::fixed << std::setprecision(2) << continuous_time_numerator_ << " / \n";

  for (int i = n; i > 0; i--) {
    stream << std::fixed << std::setprecision(2) << continuous_time_denominator_[n - i].real()
           << " * s [" << i << "] + ";
  }

  stream << std::fixed << std::setprecision(2) << continuous_time_denominator_[n].real();

  const auto & tf_text = stream.str();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%s]", tf_text.c_str());
}

/**
 * @brief This method assumes the continuous time transfer function of filter has already been
 * computed and stored in the object and uses the bilinear transformation to obtain the discrete
 * time transfer function.
 *
 * Matlab equivalent :
 * Td = 2.
 * [numd, dend] = bilinear(sys_filt.Numerator{1}, sys_filt.Denominator{1}, 1/Td)
 * where sys_filt is the continuous time transfer function.
 * */
void ButterworthFilter::computeDiscreteTimeTF(const bool & use_sampling_frequency)
{
  discrete_time_zeros_.resize(order_, {-1.0, 0.0});  // Butter puts zeros at -1.0 for causality
  discrete_time_roots_.resize(order_, {0.0, 0.0});

  An_.resize(order_ + 1, 0.0);
  Bn_.resize(order_ + 1, 0.0);

  discrete_time_gain_ = {continuous_time_numerator_, 0.0};

  // Bi-linear Transformation of the Roots

  if (use_sampling_frequency) {
    for (size_t i = 0; i < discrete_time_roots_.size(); ++i) {
      auto & dr = discrete_time_roots_[i];

      dr = (1.0 + continuous_time_roots_[i] / (sampling_frequency_hz * 2.0)) /
           (1.0 - continuous_time_roots_[i] / (sampling_frequency_hz * 2.0));
    }

    discrete_time_denominator_ = poly(discrete_time_roots_);

    // Obtain the coefficients of numerator and denominator
    discrete_time_numerator_ = poly(discrete_time_zeros_);

    // Compute Discrete Time Gain
    const auto & sum_num = std::accumulate(
      discrete_time_numerator_.cbegin(), discrete_time_numerator_.cend(), std::complex<double>{});

    const auto & sum_den = std::accumulate(
      discrete_time_denominator_.cbegin(), discrete_time_denominator_.cend(),
      std::complex<double>{});

    discrete_time_gain_ = std::abs(sum_den / sum_num);

    for (size_t i = 0; i < discrete_time_numerator_.size(); ++i) {
      auto & dn = discrete_time_numerator_[i];
      dn = dn * discrete_time_gain_;
      Bn_[i] = dn.real();
    }

    for (size_t i = 0; i < discrete_time_denominator_.size(); ++i) {
      const auto & dd = discrete_time_denominator_[i];
      An_[i] = dd.real();
    }
  } else {
    for (size_t i = 0; i < discrete_time_roots_.size(); ++i) {
      auto & dr = discrete_time_roots_[i];
      dr = (1.0 + Td_ * continuous_time_roots_[i] / 2.0) /
           (1.0 - Td_ * continuous_time_roots_[i] / 2.0);

      discrete_time_gain_ = discrete_time_gain_ / (1.0 - continuous_time_roots_[i]);
    }

    // Obtain the coefficients of numerator and denominator
    discrete_time_denominator_ = poly(discrete_time_roots_);
    discrete_time_numerator_ = poly(discrete_time_zeros_);

    for (size_t i = 0; i < discrete_time_numerator_.size(); ++i) {
      auto & dn = discrete_time_numerator_[i];
      dn = dn * discrete_time_gain_;
      Bn_[i] = dn.real();
    }

    for (size_t i = 0; i < discrete_time_denominator_.size(); ++i) {
      const auto & dd = discrete_time_denominator_[i];
      An_[i] = dd.real();
    }
  }
}
void ButterworthFilter::printDiscreteTimeTF() const
{
  const int & n = order_;

  // *****
  std::stringstream stream;
  stream << "\nThe Discrete Time Transfer Function of the Filter is ;\n";

  for (int i = n; i > 0; i--) {
    stream << std::fixed << std::setprecision(2) << discrete_time_numerator_[n - i].real();
    stream << " z[-" << i << " ] + ";
  }

  stream << std::fixed << std::setprecision(2) << discrete_time_numerator_[n].real() << " / \n";

  for (int i = n; i > 0; i--) {
    stream << std::fixed << std::setprecision(2) << discrete_time_denominator_[n - i].real();
    stream << " z[-" << i << " ] + ";
  }
  stream << std::fixed << std::setprecision(2) << discrete_time_denominator_[n].real() << " \n\n";

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%s]", stream.str().c_str());
}
std::vector<double> ButterworthFilter::getAn() const { return An_; }
std::vector<double> ButterworthFilter::getBn() const { return Bn_; }

sDifferenceAnBn ButterworthFilter::getAnBn() const { return {An_, Bn_}; }

void ButterworthFilter::printFilterSpecs() const
{
  /**
   * @brief Prints the order and cut-off angular frequency (rad/sec) of the filter
   *
   * */

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The order of the filter : %d ", this->order_);

  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"), "Cut-off Frequency : %2.2f rad/sec",
    this->cutoff_frequency_rad_sec);
}
