
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

#include "communication_delay_compensator_core.hpp"

#include <utility>

// -- HELPER METHODS --
observers::tf_t observers::get_nthOrderTF(
        const autoware::common::types::float64_t &w_cut_off_hz, const int &n) {
    auto &&wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

    // float64_t time_constant_of_qfilter{};
    auto tau = 1.0 / wc_rad_sec;

    // --------------- Qfilter Construction --------------------------------------
    // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
    // Calculate the transfer function.
    ns_control_toolbox::tf_factor denominator{std::vector<double>{tau, 1.}};  // (tau*s+1)

    // Take power of the denominator.
    denominator.power(static_cast<unsigned int>(n));

    // Create the transfer function from a numerator an denominator.
    auto const &&q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

    return q_tf;
}

observers::tf_t observers::get_nthOrderTFwithDampedPoles(
        const autoware::common::types::float64_t &w_cut_off_hz, const int &remaining_order,
        const autoware::common::types::float64_t &damping_val) {
    auto &&wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

    // float64_t time_constant_of_qfilter{};
    auto tau = 1.0 / wc_rad_sec;

    // A second order damped transfer function.
    auto q_tf_damped = tf_t({1.}, {tau * tau, 2 * damping_val * tau, 1.});

    if (remaining_order > 0) {
        auto q_remaining_tf = get_nthOrderTF(w_cut_off_hz, remaining_order);
        q_tf_damped = q_tf_damped * q_remaining_tf;
    }

    return q_tf_damped;
}

// ------------------- Communication Delay Compensator using Forward Dynamics. ---------

observers::LateralCommunicationDelayCompensator::LateralCommunicationDelayCompensator(
        observers::LateralCommunicationDelayCompensator::model_ptr_t vehicle_model,
        const observers::tf_t &qfilter_lateral, sLyapMatrixVecs const &lyap_matsXY,
        const float64_t &dt)
        : vehicle_model_ptr_(std::move(vehicle_model)),
          tf_qfilter_lat_{qfilter_lateral},
          xu0_{state_qfilter::Zero()},
          xd0_{state_qfilter::Zero()},
          xhat0_prev_{state_vector_observer_t::Zero()},
          xhat0_next_{state_vector_observer_t::Zero()},
          vXs_{lyap_matsXY.vXs},
          vYs_{lyap_matsXY.vYs},
          Lobs_{input_matrix_observer_t::Zero()},
          theta_params_{state_vector_observer_t::Zero()},
          dt_{dt},
          qfilter_order_{qfilter_lateral.order()} {
    // Compute the state-space model of QGinv(s)
    ss_qfilter_lat_ = ss_t(tf_qfilter_lat_, dt_);  // Do not forget to enter the time step dt.
}

void observers::LateralCommunicationDelayCompensator::printQfilterTFs() const {
    ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
    ns_utils::print("Transfer function of qfilter of lateral error : \n");
    tf_qfilter_lat_.print();
}

void observers::LateralCommunicationDelayCompensator::printQfilterSSs() const {
    ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
    ns_utils::print("State-Space Matrices of qfilter of lateral error : \n");
    ss_qfilter_lat_.print();
}

void observers::LateralCommunicationDelayCompensator::printLyapMatrices() const {
    ns_utils::print("Lyapunov X matrices : ");
    if (!vXs_.empty()) {
        for (auto const &item: vXs_) {
            ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
        }
    }

    ns_utils::print("Lyapunov Y matrices : ");
    if (!vYs_.empty()) {
        for (auto const &item: vYs_) {
            ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
        }
    }
}

void observers::LateralCommunicationDelayCompensator::setInitialStates() {
    if (!is_vehicle_initial_states_set_) {
        if (vehicle_model_ptr_->areInitialStatesSet()) {
            auto const vehicle_states = vehicle_model_ptr_->getInitialStates();

            xhat0_prev_ << vehicle_states(0), vehicle_states(1), vehicle_states(2), 0.;
            is_vehicle_initial_states_set_ = true;
        }
    }
}

void observers::LateralCommunicationDelayCompensator::simulateOneStep(
        const state_vector_vehicle_t &current_measurements,
        const autoware::common::types::float64_t &current_steering_cmd,
        std::shared_ptr<DelayCompensatatorMsg> &msg_compensation_results,
        std::shared_ptr<DelayCompensatorDebugMsg> &msg_debug_results) {
    // If the initial states of the state observer are not set, sets it.
    setInitialStates();

    // Compute the observer gain matrix for the current operating conditions.
    computeObserverGains();

    // Filter the current input and store it in the as the filtered previous input.
    qfilterControlCommand(current_steering_cmd);

    // Final assignment steps.
    msg_debug_results->lat_uf = static_cast<float>(current_qfiltered_control_cmd_);

    // Debug
    ns_utils::print("Current steering command read : ", current_steering_cmd);
    ns_utils::print("Steering command read : ", current_steering_cmd);
    ns_utils::print("Heading error read : ", msg_compensation_results->heading_angle_error_read);
    ns_utils::print("lat_ey_hat : ", msg_debug_results->lat_ey_hat);

    ns_utils::print("Current Measurements : ");
    ns_eigen_utils::printEigenMat(Eigen::MatrixXd(current_measurements));

    ns_utils::print(
            "Previous and current filtered commands :", previous_qfiltered_control_cmd_,
            current_qfiltered_control_cmd_);
}

void observers::LateralCommunicationDelayCompensator::computeObserverGains() {
    theta_params_.setZero();
    vehicle_model_ptr_->evaluateNonlinearTermsForLyap(theta_params_);

    ns_utils::print("Current Thetas : ");
    ns_eigen_utils::printEigenMat(Eigen::MatrixXd(theta_params_));
}

void observers::LateralCommunicationDelayCompensator::qfilterControlCommand(
        const autoware::common::types::float64_t &current_control_cmd) {
    // First give the output, then update the states.
    previous_qfiltered_control_cmd_ = current_qfiltered_control_cmd_;
    current_qfiltered_control_cmd_ = ss_qfilter_lat_.simulateOneStep(xu0_, current_control_cmd);
}

/**
 * @brief Two-steps estimation method is used as we need the current state estimate. To do this,
 * we use the previous values for the vehicle model. Upon estimating the current states, we
 * predict the next states and broadcast it.
 * */
void observers::LateralCommunicationDelayCompensator::estimateVehicleStates() {
    // First step propagate the previous steps.
}
