// Copyright 2021 The Autoware Foundation.
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

#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

SimModelInterface::SimModelInterface(int dim_x, int dim_u) : dim_x_(dim_x), dim_u_(dim_u)
{
  state_ = Eigen::VectorXd::Zero(dim_x_);
  input_ = Eigen::VectorXd::Zero(dim_u_);
}

void SimModelInterface::updateRungeKutta(const float64_t & dt, const Eigen::VectorXd & input)
{
  Eigen::VectorXd k1 = calcModel(state_, input);
  Eigen::VectorXd k2 = calcModel(state_ + k1 * 0.5 * dt, input);
  Eigen::VectorXd k3 = calcModel(state_ + k2 * 0.5 * dt, input);
  Eigen::VectorXd k4 = calcModel(state_ + k3 * dt, input);

  state_ += 1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt;
}
void SimModelInterface::updateEuler(const float64_t & dt, const Eigen::VectorXd & input)
{
  state_ += calcModel(state_, input) * dt;
}
void SimModelInterface::getState(Eigen::VectorXd & state) { state = state_; }
void SimModelInterface::getInput(Eigen::VectorXd & input) { input = input_; }
void SimModelInterface::setState(const Eigen::VectorXd & state) { state_ = state; }
void SimModelInterface::setInput(const Eigen::VectorXd & input) { input_ = input; }
void SimModelInterface::setGear(const uint8_t gear) { gear_ = gear; }

std::pair<double, double> SimModelInterface::getSteerTimeDelayDisturbanceInputs()
{
  return disturbance_collection_.steering_inputDisturbance_time_delay_ptr_
    ->getOriginalAndDelayedInputs();
}

double SimModelInterface::getCurrentSteerTimeDelay()
{
  return disturbance_collection_.steering_inputDisturbance_time_delay_ptr_
    ->getCurrentTimeDelayValue();
}

std::pair<double, double> SimModelInterface::getAccTimeDelayDisturbanceInputs()
{
  return disturbance_collection_.acc_inputDisturbance_time_delay_ptr_
    ->getOriginalAndDelayedInputs();
}

double SimModelInterface::getCurrentAccTimeDelay()
{
  return disturbance_collection_.acc_inputDisturbance_time_delay_ptr_->getCurrentTimeDelayValue();
}

double SimModelInterface::getCurrentRoadSlopeAccDisturbance()
{
  return disturbance_collection_.road_slope_outputDisturbance_ptr_->getDisturbedOutput();
}

std::array<double, 4> SimModelInterface::getCurrentDeadzoneParams()
{
  return disturbance_collection_.steering_dedzone_ptr_->getCurrentDeadZoneParameters();
}

std::pair<double, double> SimModelInterface::getCurrentDeadzoneDisturbanceInputs()
{
  return disturbance_collection_.steering_dedzone_ptr_->getOriginalAndDeadzonedInputs();
}
