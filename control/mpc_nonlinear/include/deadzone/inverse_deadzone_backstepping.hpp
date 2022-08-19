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


#ifndef MPC_NONLINEAR_INCLUDE_DEADZONE_INVERSE_DEADZONE_BACKSTEPPING_HPP_
#define MPC_NONLINEAR_INCLUDE_DEADZONE_INVERSE_DEADZONE_BACKSTEPPING_HPP_

#include <cmath>
#include "utils_act/act_utils.hpp"

namespace ns_deadzone
{

constexpr double hertz2radsec(double x)
{
  return 2. * M_PI * x;
}

struct sExtremumSeekerParams
{
  double K{1.}; // Gain
  double wl{1.}; // low-pass filter cut-off frequency
  double wh{1.}; // high-pass filter cut-off frequency
  double wd{1.}; // dither signal frequency
  double dt{0.033};
};

struct sDeadZone
{
  // Constructors
  sDeadZone() = default;
  sDeadZone(double const &mr, /*right slope*/
            double const &br,  /*right threshold*/
            double const &ml,
            double const &bl);

  // Members
  double mr_{1.};
  double mrbr_{0.};

  double ml_{1.};
  double mlbl_{0.};

  double e0_{1e-2}; // param for sigmoid like indicator function for the inverse.

  // Methods
  [[nodiscard]] double get_br() const;
  [[nodiscard]] double get_bl() const;

  [[nodiscard]] double deadzoneOutput(double const &Du) const;
  [[nodiscard]] double invDeadzoneOutputSmooth(double const &desired_Du) const;
  [[nodiscard]] double invDeadzoneOutput(double const &desired_Du) const;

//  [[nodiscard]] double convertInverted_d_minus_u(double const &current_steering,
//                                                 double const &current_steering_cmd,
//                                                 double const &invDu) const;

};

class ExtremumSeeker
{
 public:
  ExtremumSeeker() = default;
  ExtremumSeeker(sExtremumSeekerParams const &es_params);

  void print()
  {
    ns_utils::print("Extremum seeker is prepared...");
  }

 private:
  double K_{1.}; // Gain
  double wl_{1.}; // low-pass filter cut-off frequency
  double wh_{1.}; // high-pass filter cut-off frequency
  double wd_{1.}; // dither signal frequency
  double dt_{0.033};

};

} // namespace ns_deadzone

#endif //MPC_NONLINEAR_INCLUDE_DEADZONE_INVERSE_DEADZONE_BACKSTEPPING_HPP_
