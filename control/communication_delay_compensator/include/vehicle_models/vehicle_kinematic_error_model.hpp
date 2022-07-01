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
#include "utils_delay_observer/delay_compensation_utils.hpp"
#include "vehicle_definitions.hpp"

#include <eigen3/Eigen/Core>
#include <Eigen/StdVector>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace observers
{

/*
 * @brief : Vehicle model for disturbance observers
 */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
class LinearVehicleModelsBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Atype = mat_type_t<STATE_DIM, STATE_DIM>;
  using Btype = mat_type_t<STATE_DIM, INPUT_DIM>;
  using Ctype = mat_type_t<MEASUREMENT_DIM, STATE_DIM>;
  using Dtype = mat_type_t<MEASUREMENT_DIM, INPUT_DIM>;

  using state_vector_t = mat_type_t<STATE_DIM, 1>;
  using measurement_vector_t = mat_type_t<MEASUREMENT_DIM, 1>;

  // Constructors
  LinearVehicleModelsBase() = default;

  LinearVehicleModelsBase(
      float64_t const &wheelbase, float64_t const &tau_steering, float64_t const &dt);

  // Destructors
  virtual ~LinearVehicleModelsBase() = default;

  virtual void updateStateSpace(float64_t const &vr, float64_t const &steer_r);

  void simulateOneStep(
      measurement_vector_t &y0, state_vector_t &x0, float64_t const &u);

  void updateInitialStates(
      float64_t const &ey, float64_t const &eyaw, float64_t const &steering, float64_t const &vx,
      float64_t const &curvature);

  void discretisize();

  void printContinuousSystem();

  void printDiscreteSystem();

  [[nodiscard]] bool8_t areInitialStatesSet() const
  { return are_initial_states_set_; }

  [[nodiscard]] state_vector_t getInitialStates() const;

  void evaluateNonlinearTermsForLyap(observers::state_vector_observer_t &thetas,
                                     measurement_vector_t const &y0) const;

 protected:
  bool8_t are_initial_states_set_{false};
  float64_t wheelbase_{2.74};
  float64_t tau_steering_{0.3};
  float64_t dt_{0.1};

  // Continuous time state space matrices.
  Atype A_{Atype::Zero()};
  Btype B_{Btype::Zero()};
  Btype Bw_{Btype::Zero()};
  Ctype C_{Ctype::Identity()};
  Dtype D_{Dtype::Zero()};

  // Discrete time state space matrices.
  Atype Ad_{Atype::Zero()};
  Btype Bd_{Btype::Zero()};
  Btype Bwd_{Btype::Zero()};
  Ctype Cd_{Ctype::Zero()};
  Dtype Dd_{Dtype::Zero()};

  Atype I_At2_{Atype::Zero()};  // inv(I - A*ts/2)
  state_vector_t x0_;           // keep initial states.
  float64_t long_velocity_{};
  float64_t curvature_{};

  /**
  * @brief update algebraic solution of inv(I - A*ts/2) required in computing
  * the Tustin form discretization.
  * [ 1, (V*ts)/2, (T*V^2*ts^2)/(2*L*a^2*(2*T + ts))]
  * [ 0,        1,       (T*V*ts)/(L*a^2*(2*T + ts))]
  * [ 0,        0,                  (2*T)/(2*T + ts)]
  *
  * */
  void updateI_Ats2(float64_t const &vref, float64_t const &cos_steer_sqr);

  // Take direct inverse, instead computing analytical inverse.
  virtual void updateI_Ats2();

};

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::printContinuousSystem()
{
  ns_utils::print("Matrix A: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(A_));

  ns_utils::print("Matrix B: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(B_));

  ns_utils::print("Matrix C: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(C_));

  ns_utils::print("Matrix D: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(D_));

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::printDiscreteSystem()
{

  ns_utils::print("Matrix Ad: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Ad_));

  ns_utils::print("Matrix Bd: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Bd_));

  ns_utils::print("Matrix Bwd: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Bwd_));

  ns_utils::print("Matrix Cd: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Cd_));

  ns_utils::print("Matrix Dd: ");
  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Dd_));

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::LinearVehicleModelsBase(const float64_t &wheelbase,
                                                                                        const float64_t &tau_steering,
                                                                                        const float64_t &dt)
    : wheelbase_{wheelbase}, tau_steering_{tau_steering}, dt_{dt}
{


  // Assuming tau does not change.
  A_(2, 2) = -1. / tau_steering_;  // constant terms
  B_(2, 0) = 1. / tau_steering_;

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateStateSpace(const float64_t &vr,
                                                                                      const float64_t &steer_r)
{

  auto const &cos_sqr = std::cos(steer_r) * std::cos(steer_r);
  auto &&L = wheelbase_;
  /**
   * @brief
   * A matrix
   *          [ 0, V,                     0]
   *          [ 0, 0, V/(L*cos(steering)^2)]
   *          [ 0, 0,                  -1/tau]
   * */
  A_(0, 1) = vr;
  A_(1, 2) = vr / (L * cos_sqr);

  /**
   * @brief  B = [0, 0, 1/tau]^T
   * if tau is not constant, we can update here.
   * */
  // B_(2, 0) = 1 / tau_steering_;  // for desired heading rate computations.

  /**
   * @brief  Bw = [0, 1/tau, 0]^T
   *
   * */
//  Bw_(1, 0) = vr * tan(steer_r) / L - vr * curvature_ - vr * steer_r / (L * cos_sqr);
  Bw_(1, 0) = vr * tan(steer_r) / L - vr * curvature_ - steer_r / (L * cos_sqr);

  //  auto IA = I - A_ * dt_ / 2;
  //  auto Ainv = IA.inverse();
  updateI_Ats2();

  // Discretisize.
  discretisize();

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateI_Ats2(float64_t const &vref,
                                                                                  float64_t const &cos_steer_sqr)
{

  auto const &L = wheelbase_;
  auto const asqr = cos_steer_sqr;
  auto const &tau = tau_steering_;
  auto const &var = 2 * tau + dt_;

  I_At2_(0, 1) = vref * dt_ / 2.;
  I_At2_(0, 2) = tau_steering_ * vref * vref * dt_ * dt_ / (2. * L * asqr * var);

  I_At2_(1, 2) = tau * vref * dt_ / (L * asqr * var);
  I_At2_(2, 2) = 2. * tau / var;

  // DEBUG
  if (A_.rows() == 3)
  {
    ns_utils::print("In vehicle model, inverse IA term");
    ns_eigen_utils::printEigenMat(Eigen::MatrixXd(I_At2_));
  } else
  {
    ns_utils::print("In observer model, inverse IA term");
    ns_eigen_utils::printEigenMat(Eigen::MatrixXd(I_At2_));
  }

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateI_Ats2()
{

  auto const &I = Atype::Identity();

  I_At2_ = (I - A_ * dt_ / 2).inverse();

  // Debug
  //ns_utils::print("I - A*dt/2 inverse is updated.");

}

/**
* @brief Simulates the vehicle motion given an input. The resulting states and outputs are
* written into the passed arguments.
* @param y0: Output of the one step integration.
* @param x0: Vehicle initial states, updated after the integration.
* @param u: Steering control signal.
*
* */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::simulateOneStep(measurement_vector_t &y0,
                                                                                     state_vector_t &x0,
                                                                                     const float64_t &u)
{

  // first update the output
  // y0 = x0 + dt_ * (A_ * x0 + B_ * steering_and_ideal_steering);
  y0 = Cd_ * x0.eval() + Dd_ * u;
  x0 = Ad_ * x0.eval() + Bd_ * u + Bwd_;

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void
LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateInitialStates(const float64_t &ey,
                                                                                    const float64_t &eyaw,
                                                                                    const float64_t &steering,
                                                                                    const float64_t &vx,
                                                                                    const float64_t &curvature)
{
  /**
   * @brief The CDOBs use the linear vehicle model to access to its state space. The states of
   * the parallel model is estimated by a state observer using this state-space.
   * */
  x0_.topRows(3) << ey, eyaw, steering;

  long_velocity_ = vx;
  curvature_ = curvature;
  are_initial_states_set_ = true;

}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
typename LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::state_vector_t
LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::getInitialStates() const
{
  return x0_;
}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::evaluateNonlinearTermsForLyap(
    state_vector_observer_t &thetas,
    measurement_vector_t const &y0) const
{

  // Extract values.
  auto const &ey = y0(0);
  auto const &eyaw = y0(1);
  auto const &steering = y0(2);

  auto const &&kappa_expr = curvature_ / (1. - curvature_ * ey);
  auto const &&ke_sqr = kappa_expr * kappa_expr;

  auto const &&t1 = long_velocity_ * std::cos(eyaw);

  auto const &&t2 = -ke_sqr * t1;
  auto const &&t3 = kappa_expr * long_velocity_ * std::sin(eyaw);
  auto const &&t4 = long_velocity_ / (wheelbase_ * std::cos(steering) * std::cos(steering));

  thetas << t1, t2, t3, t4;
}

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::discretisize()
{

  auto const &I = Atype::Identity();

  Ad_ = I_At2_ * (I + A_ * dt_ / 2.);
  Bd_ = I_At2_ * B_ * dt_;
  Cd_ = C_ * I_At2_;
  Dd_ = D_ + C_ * Bd_.eval() / 2.;

  // Disturbance part
  Bwd_ = I_At2_ * Bw_ * dt_;
}

/**
 * @brief Linear vehicle model for state observers
 * */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
class VehicleModelDisturbanceObserver : public LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>
{

  using BASE = LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructors.
  using BASE::LinearVehicleModelsBase::LinearVehicleModelsBase;
  void updateStateSpace(const float64_t &vr, const float64_t &steer_r) override;

};

template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
void VehicleModelDisturbanceObserver<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>::updateStateSpace(const float64_t &vr,
                                                                                              const float64_t &steer_r)
{
  auto const &&cos_sqr = std::cos(steer_r) * std::cos(steer_r);

  /**
    * @brief
    * A matrix
    *          [ 0, V,                     0]
    *          [ 0, 0, V/(L*cos(steering)^2)]
    *          [ 0, 0,                  -1/tau]
    * */

  auto &&L = this->wheelbase_;
  this->A_(0, 1) = vr;
  this->A_(1, 2) = vr / (L * cos_sqr);


  /**
   * @brief  B = [0, 0, 1/tau]^T
   *
   * */
  // this->B_(2, 0) = 1 / this->tau_steering_;  // for desired heading rate computations.

  // In observer model Anew = [A; -B]
  auto col_size = this->A_.cols();
  this->A_.col(col_size - 1) = -this->B_;

  /**
   * @brief  Bw = [0, 1/tau, 0]^T
   *
   * */
  auto &&curvature_ = this->curvature_;
//  this->Bw_(1, 0) = vr * tan(steer_r) / L - vr * curvature_ - vr * steer_r / (L * cos_sqr);
  this->Bw_(1, 0) = vr * tan(steer_r) / L - vr * curvature_ - steer_r / (L * cos_sqr);

  //  auto IA = I - A_ * dt_ / 2;
  //  auto Ainv = IA.inverse();
  this->updateI_Ats2();

  // Discretisize.
  this->discretisize();
}

/**
* @brief Linear kinematic error vehicle model with three states [ey, eyaw, ]
* */
template<int STATE_DIM, int INPUT_DIM, int MEASUREMENT_DIM>
class LinearKinematicErrorModel : public LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>
{
  using BASE = LinearVehicleModelsBase<STATE_DIM, INPUT_DIM, MEASUREMENT_DIM>;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BASE::LinearVehicleModelsBase::LinearVehicleModelsBase;
  using BASE::updateInitialStates;
};

// Observer model types.
using linear_vehicle_model_t = LinearKinematicErrorModel<toUType(KinematicErrorDims::STATE_DIM),
                                                         toUType(KinematicErrorDims::INPUT_DIM),
                                                         toUType(KinematicErrorDims::MEASUREMENT_DIM)>;

using linear_state_observer_model_t = VehicleModelDisturbanceObserver<toUType(
    StateObserverDims::STATE_DIM), toUType(StateObserverDims::INPUT_DIM),
                                                                      toUType(StateObserverDims::MEASUREMENT_DIM)>;

} // namespace observers


/**
 * @brief Implemented to test the current packages and inverted model performance.
 * */
class NonlinearVehicleKinematicModel
{
 public:
  // Constructors.
  NonlinearVehicleKinematicModel() = default;

  NonlinearVehicleKinematicModel(
      double const &wheelbase, double const &tau_vel, double const &tau_steer,
      double const &deadtime_vel, double const &deadtime_steer, double const &dt);

  // Public methods.
  std::array<double, 4> simulateNonlinearOneStep(
      const double &desired_velocity, double const &desired_steering);

  std::array<double, 4> simulateLinearOneStep(
      const double &desired_velocity, double const &desired_steering);

  void getInitialStates(std::array<double, 4> &x0);

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
  std::array<double, 4> x0_{0., 0., 0., 0.};  // this state is updated.

  // delayed input states.
  Eigen::MatrixXd xv0_{Eigen::MatrixXd::Zero(2, 1)};  // delayed speed input states
  Eigen::MatrixXd xs0_{Eigen::MatrixXd::Zero(2, 1)};  // delayed steeering input states.
};

#endif  // COMMUNICATION_DELAY_COMPENSATOR__VEHICLE_KINEMATIC_ERROR_MODEL_HPP
