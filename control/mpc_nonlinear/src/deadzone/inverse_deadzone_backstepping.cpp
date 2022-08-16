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
double sDeadZone::deadzoneOutput(const double &u) const
{
  auto const &br = get_br();
  auto const &bl = get_bl();
  if (u >= br)
  {
    return mr_ * u - mrbr_;
  }
  if (u <= bl)
  {
    return ml_ * u - mlbl_;
  }

  if (u > bl && u < br)
  {
    return 0.;
  }

  return 0;
}
double sDeadZone::invDeadzoneOutput(const double &u) const
{
  auto x = u / e0_;

  auto const &phi_r = exp(x) / (exp(x) + exp(-x));
  auto const &phi_l = exp(-x) / (exp(x) + exp(-x));

  auto const &DzInv = phi_r * (u + mrbr_) / mr_ + phi_l * (u + mlbl_) / ml_;

  return DzInv;
}

} // namespace ns_deadzone