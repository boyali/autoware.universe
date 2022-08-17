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

#include "deadzone/inverse_deadzone_backstepping.hpp"

namespace ns_deadzone
{

sDeadZone::sDeadZone(const double &mr, const double &br, const double &ml, const double &bl)
  : mr_{mr}, mrbr_{mr * br}, ml_{ml}, mlbl_{-ml * bl}
{

}

double sDeadZone::get_br() const
{
  return mrbr_ / mr_;
}
double sDeadZone::get_bl() const
{
  return mlbl_ / ml_;
}

/**
 * @brief given a steering deviation, computes the deadzone function output.
 * @param Du = steering - steering_control = δ - u
 * */
double sDeadZone::deadzoneOutput(const double &Du) const
{
  auto const &br = get_br();
  auto const &bl = get_bl();

  if (Du >= br)
  {
    return mr_ * Du - mrbr_ * 0;
  }
  if (Du <= bl)
  {
    return ml_ * Du - mlbl_ * 0;
  }

  if (Du > bl && Du < br)
  {
    return 0.;
  }

  return 0;
}

/**
 * @brief given a steering deviation, computes the smooth inverse deadzone function output.
 * @param desired_Du = steering - steering_control = δ - u
 * */

double sDeadZone::invDeadzoneOutputSmooth(const double &desired_Du) const
{
  auto const &u = desired_Du;
  auto x = u / e0_;

  auto const &phi_r = exp(x) / (exp(x) + exp(-x));
  auto const &phi_l = exp(-x) / (exp(x) + exp(-x));

  auto const &dz_inv = phi_r * (u + mrbr_) / mr_ + phi_l * (u + mlbl_) / ml_;

  return dz_inv;
}

double sDeadZone::invDeadzoneOutput(const double &desired_Du) const
{
  auto const &u = desired_Du;

  auto const &br = get_br();
  auto const &bl = get_bl();

  double dz_inv{};

  if (desired_Du >= br)
  {

    return mr_ * u - mrbr_;
  }

  if (desired_Du <= bl)
  {
    return ml_ * u - mlbl_;
  }

  return dz_inv;
}

/**
 * @brief computes u from delta_u = δ - u. --> u =  δ - delta_u
 * invDu is the desired deadzone output computed by the inverted deadzone function.
 * */
//double sDeadZone::convertInverted_d_minus_u(double const &current_steering,
//                                            double const &current_steering_cmd,
//                                            const double &invDu) const
//{
//  auto const &br = get_br();
//  auto const &bl = get_bl();
//
//  auto const &Du = current_steering - current_steering_cmd;
//
//  if (Du >= br)
//  {
//    return current_steering + invDu * mr_ + mrbr_;
//  }
//
//  if (Du <= bl)
//  {
//    return current_steering + invDu * ml_ + mlbl_;
//  }
//
//  return current_steering_cmd;
//
//  return current_steering;
//}

} // namespace ns_deadzone