// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "utils_act/act_utils.hpp"
#include "utils_act/writetopath.hpp"

int main()
{
  // Create a continuous signal for an angle series.
  auto log_path = getOutputPath() / "helper_funcs";

  double Nx = 512;
  std::vector<double> xvec = ns_utils::linspace<double>(0.0, Nx, static_cast<size_t>(Nx));

  // Generate y = 6*sin(2*pi*n/N).

  std::vector<double> yvec;
  std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const & x) {
    return 6 * sin(2 * M_PI * x / Nx);
  });

  writeToFile(log_path, yvec, "xc");

  /**
   * Wrap the signal into [-pi, pi]
   * */

  std::vector<double> xw;
  std::transform(yvec.cbegin(), yvec.cend(), std::back_inserter(xw), [&](auto const & x) {
    return std::atan2(sin(x), cos(x));
  });

  writeToFile(log_path, xw, "xw");

  /**
   * unWrap the wrapped signal xw.
   * */

  ns_utils::unWrap(xw);
  writeToFile(log_path, xw, "uw_x");

  return 0;
}
