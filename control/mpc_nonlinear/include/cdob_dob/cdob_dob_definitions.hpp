//
// Created by ali on 11/07/22.
//

#ifndef MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_DOB_DEFINITIONS_HPP_
#define MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_DOB_DEFINITIONS_HPP_

#include "eigen3/Eigen/Core"
#include "utils/nmpc_utils.hpp"

namespace ns_cdob
{
/**
 * @brief Kinematic Vehicle Lateral Error Model state and control definitions.
 * */
enum class CDOB_KinematicErrorDims : int
{
	STATE_DIM = 3,
	INPUT_DIM = 1,
	MEASUREMENT_DIM = 3
};

using state_vector_vehicle_t =
	Eigen::Matrix<double, ns_nmpc_utils::toUType(CDOB_KinematicErrorDims::STATE_DIM), 1>;

// Lyapunov matrix dimension definitions.
constexpr size_t cx_NUMBER_OF_LYAP_MATS = 5;
enum class StateObserverDims : int
{
	STATE_DIM = 4,
	INPUT_DIM = 1,
	MEASUREMENT_DIM = 3,

};

using state_vector_observer_t =
	Eigen::Matrix<double, ns_nmpc_utils::toUType(StateObserverDims::STATE_DIM), 1>;

using input_vector_observer_t =
	Eigen::Matrix<double, ns_nmpc_utils::toUType(StateObserverDims::INPUT_DIM), 1>;

using state_matrix_observer_t = Eigen::Matrix<double, ns_nmpc_utils::toUType(StateObserverDims::STATE_DIM),
																							ns_nmpc_utils::toUType(StateObserverDims::STATE_DIM)>;

using measurement_vector_observer_t = Eigen::Matrix<double, ns_nmpc_utils::toUType(StateObserverDims::STATE_DIM),
																										ns_nmpc_utils::toUType(StateObserverDims::STATE_DIM)>;

using input_matrix_observer_t = Eigen::Matrix<double, ns_nmpc_utils::toUType(StateObserverDims::INPUT_DIM),
																							ns_nmpc_utils::toUType(StateObserverDims::STATE_DIM)>;

using measurement_matrix_observer_t = Eigen::Matrix<double, ns_nmpc_utils::toUType(StateObserverDims::MEASUREMENT_DIM),
																										ns_nmpc_utils::toUType(StateObserverDims::STATE_DIM)>;


// General Template for enum class types.
template<int Nnum_of_states>
using state_vector_qfilter = Eigen::Matrix<double, Nnum_of_states, 1>;

template<typename T>
using func_type = std::function<T(T)>;

template<int nx, int ny>
using mat_type_t = Eigen::Matrix<double, nx, ny>;

} // namespace ns_cdob

#endif //MPC_NONLINEAR_INCLUDE_CDOB_DOB_CDOB_DOB_DEFINITIONS_HPP_
