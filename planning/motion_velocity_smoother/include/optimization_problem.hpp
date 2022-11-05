//
// Created by ali.boyali@tier4.jp on 5/11/22.
//

#ifndef MOTION_VELOCITY_SMOOTHER_INCLUDE_MOTION_VELOCITY_SMOOTHER_OPTIMIZATION_PROBLEM_HPP_
#define MOTION_VELOCITY_SMOOTHER_INCLUDE_MOTION_VELOCITY_SMOOTHER_OPTIMIZATION_PROBLEM_HPP_

#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"

namespace motion_velocity_smoother
{

class OptimizationProblemSQP
{
 public:
  // Type defs.
  using triplet_type = Eigen::Triplet<double>;
  OptimizationProblemSQP() = default;

 private:

};

} // motion_velocity_smoother

#endif //MOTION_VELOCITY_SMOOTHER_INCLUDE_MOTION_VELOCITY_SMOOTHER_OPTIMIZATION_PROBLEM_HPP_
