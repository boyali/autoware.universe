// Copyright 2022 The Autoware Foundation
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

#ifndef COMMUNICATION_DELAY_COMPENSATOR__VEHICLE_KINEMATIC_ERROR_MODEL_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__VEHICLE_KINEMATIC_ERROR_MODEL_HPP

#include "autoware_control_toolbox.hpp"
#include "node_denifitions/node_definitions.hpp"
#include "qfilters.hpp"
#include "utils_delay_observer/delay_compensation_utils.hpp"
#include "vehicle_definitions.hpp"

#include <eigen3/Eigen/Core>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace observers
{

/**
 * @brief Linear kinematic error vehicle model with three states [ey, eyaw, ]
 * */
class LinearKinematicErrorModel
{
public:
  LinearKinematicErrorModel() = default;
  LinearKinematicErrorModel(
    float64_t const & wheelbase, float64_t const & tau_steering, float64_t const & dt);

  void printContinuousSystem();
  void printDiscreteSystem();

  // different types of initial state updates.
  void updateInitialStates(state_vector_vehicle_t const & x0);
  void updateInitialStates(Eigen::MatrixXd const & x0);
  void updateInitialStates(
    float64_t const & ey, float64_t const & eyaw, float64_t const & steering);

  // model update.
  void updateStateSpace(float64_t const & vref, float64_t const & steering_ref);

  void simulateOneStep(
    state_vector_vehicle_t & y0, state_vector_vehicle_t & x0, float64_t const & u);

  [[nodiscard]] bool8_t areInitialStatesSet() const { return are_initial_states_set_; }

  [[nodiscard]] state_vector_vehicle_t getInitialStates() const;

private:
  bool8_t are_initial_states_set_{false};
  float64_t wheelbase_{2.74};
  float64_t tau_steering_{0.3};
  float64_t dt_{0.1};

  state_matrix_vehicle_t A_{};
  input_matrix_vehicle_t B_{};
  input_matrix_vehicle_t Bw_{};
  state_matrix_vehicle_t C_{};
  input_matrix_vehicle_t D_{};

  state_matrix_vehicle_t Ad_{};
  input_matrix_vehicle_t Bd_{};
  input_matrix_vehicle_t Bwd_{};  // curvature disturbance term
  state_matrix_vehicle_t Cd_{};
  input_matrix_vehicle_t Dd_{};

  state_matrix_vehicle_t I_At2_{};  // inv(I - A*ts/2)
  state_vector_vehicle_t x0_;       // keep initial states.

  /**
   * @brief update algebraic solution of inv(I - A*ts/2) required in computing
   * the Tustin form discretization.
   * [ 1, (V*ts)/2, (T*V^2*ts^2)/(2*L*a^2*(2*T + ts))]
   * [ 0,        1,       (T*V*ts)/(L*a^2*(2*T + ts))]
   * [ 0,        0,                  (2*T)/(2*T + ts)]
   *
   * */
  void updateI_Ats2(float64_t const & vref, float64_t const & cos_steer_sqr);
};
}  // namespace observers

/**
 * @brief Implemented to test the current packages and inverted model performance.
 * */
class NonlinearVehicleKinematicModel
{
public:
  // Constructors.
  NonlinearVehicleKinematicModel() = default;

  NonlinearVehicleKinematicModel(
    double const & wheelbase, double const & tau_vel, double const & tau_steer,
    double const & deadtime_vel, double const & deadtime_steer, double const & dt);

  // Public methods.
  std::array<double, 4> simulateNonlinearOneStep(
    const double & desired_velocity, double const & desired_steering);

  std::array<double, 4> simulateLinearOneStep(
    const double & desired_velocity, double const & desired_steering);

  void getInitialStates(std::array<double, 4> & x0);

private:
  double wheelbase_{2.74};
  double tau_steer_{};
  double tau_vel_{};
  double dead_time_steer_{0};
  double dead_time_vel_{0};
  double dt_{0.1};

  // Bool gains static gain discretizaiton.
  bool use_delay_vel{false};
  bool use_delay_steer{false};

  std::vector<std::string> state_names_{"ey", "eyaw", "delta", "V"};        // state names.
  std::vector<std::string> control_names_{"desired_vel", "delta_desired"};  // control names.

  // Deadtime inputs
  ns_control_toolbox::tf2ss deadtime_velocity_model_{};
  ns_control_toolbox::tf2ss deadtime_steering_model_{};

  // Initial state
  std::array<double, 4> x0_{0., 0., 0., 10.};  // this state is updated.

  // delayed input states.
  Eigen::MatrixXd xv0_{Eigen::MatrixXd::Zero(2, 1)};  // delayed speed input states
  Eigen::MatrixXd xs0_{Eigen::MatrixXd::Zero(2, 1)};  // delayed steeering input states.
};

#endif  // COMMUNICATION_DELAY_COMPENSATOR__VEHICLE_KINEMATIC_ERROR_MODEL_HPP
