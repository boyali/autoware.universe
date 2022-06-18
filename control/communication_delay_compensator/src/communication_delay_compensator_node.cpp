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

#include "communication_delay_compensator_node.hpp"

namespace observers {
    CommunicationDelayCompensatorNode::CommunicationDelayCompensatorNode(
            const rclcpp::NodeOptions &node_options)
            : Node("communication_delay_compensator", node_options) {
        using std::placeholders::_1;

        // Vectors that keeps output of the communication delay compensator objects.
        size_t const n_delay_compensator_output = 5;
        cdob_lateral_error_y_outputs_.reserve(n_delay_compensator_output);
        cdob_heading_error_y_outputs_.reserve(n_delay_compensator_output);
        cdob_steering_error_y_outputs_.reserve(n_delay_compensator_output);
        cdob_velocity_error_y_outputs_.reserve(n_delay_compensator_output);
        cdob_acc_error_y_outputs_.reserve(n_delay_compensator_output);

        // Reserve internal results vectors.

        // Read vehicle model parameters
        // Implement Reading Global and Local Variables.
        // 			const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
        // 			params_node_.wheel_base = vehicle_info.wheel_base_m;

        /* Set up ros system timer and read the parameters */
        /* get parameter updates */
        readAndLoadParameters();

        params_node_.wheel_base = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo().wheel_base_m;
        initTimer(params_node_.cdob_ctrl_period);

        // Create Publishers
        pub_delay_compensator_ =
                create_publisher<DelayCompensatatorMsg>("~/output/communication_delay_compensation_refs", 1);

        pub_delay_compensator_debug_ = create_publisher<DelayCompensatorDebugMsg>(
                "~/output/communication_delay_compensation_debug", 1);

        // Create subscriptions
        sub_control_cmds_ = create_subscription<ControlCommand>(
                "~/input/control_cmd", rclcpp::QoS{1},
                std::bind(
                        &observers::CommunicationDelayCompensatorNode::onControlCommands, this,
                        std::placeholders::_1));

        sub_current_velocity_ptr_ = create_subscription<VelocityMsg>(
                "~/input/current_odometry", rclcpp::QoS{1},
                std::bind(
                        &observers::CommunicationDelayCompensatorNode::onCurrentVelocity, this,
                        std::placeholders::_1));

        sub_current_steering_ptr_ = create_subscription<SteeringReport>(
                "~/input/steering_state", rclcpp::QoS{1},
                std::bind(
                        &observers::CommunicationDelayCompensatorNode::onCurrentSteering, this,
                        std::placeholders::_1));

        sub_current_long_error_ptr_ = create_subscription<ControllerErrorReportMsg>(
                "~/input/long_errors", rclcpp::QoS{1},
                std::bind(
                        &observers::CommunicationDelayCompensatorNode::onCurrentLongitudinalError, this,
                        std::placeholders::_1));

        sub_current_lat_errors_ptr_ = create_subscription<ControllerErrorReportMsg>(
                "~/input/lat_errors", rclcpp::QoS{1},
                std::bind(
                        &observers::CommunicationDelayCompensatorNode::onCurrentLateralErrors, this,
                        std::placeholders::_1));

        sub_control_perf_errors_ptr_ = create_subscription<ErrorStampedControlPerfMsg>(
                "~/input/cp_errors", rclcpp::QoS{1},
                std::bind(
                        &observers::CommunicationDelayCompensatorNode::onControlPerfErrors, this,
                        std::placeholders::_1));

        // Dynamic Parameter Update.
        is_parameters_set_res_ = this->add_on_set_parameters_callback(
                std::bind(&CommunicationDelayCompensatorNode::onParameterUpdate, this, _1));

        // Set the delay compensator for each tracking purpose.
        setSteeringCDOBcompensator();
        setHeadingErrorCDOBcompensator();
        setLateralErrorCDOBcompensator();
        setVelocityErrorCDOBcompensator();
        setAccelerationErrorCDOBcompensator();
    }

    void CommunicationDelayCompensatorNode::initTimer(float64_t period_s) {
        const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<float64_t>(period_s));

        timer_ = rclcpp::create_timer(
                this, get_clock(), period_ns, std::bind(&CommunicationDelayCompensatorNode::onTimer, this));
    }

    void CommunicationDelayCompensatorNode::onTimer() {
        // Create compensator messages: For breaking cyclic dependency (controllers wait this package vice
        // versa.).
        DelayCompensatatorMsg compensation_msg{};
        current_delay_references_msg_ = std::make_shared<DelayCompensatatorMsg>(compensation_msg);

        DelayCompensatorDebugMsg compensation_debug_msg{};
        current_delay_debug_msg_ = std::make_shared<DelayCompensatorDebugMsg>(compensation_debug_msg);

        if (!isDataReady()) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000, "Not enough data to compute delay compensation");
            publishCompensationReferences();
            return;
        }

        if (!previous_ctrl_ptr_) {
            ControlCommand zero_cmd{};
            previous_ctrl_ptr_ = std::make_shared<ControlCommand>(zero_cmd);
            publishCompensationReferences();
        }

        // Compute the steering compensation values.
        if (!isVehicleStopping()) {
            computeSteeringCDOBcompensator();
            computeHeadingCDOBcompensator();
            computeLateralCDOBcompensator();
            computeVelocityCDOBcompensator();
            computeAccelerationCDOBcompensator();
        }

        //  computeSteeringCDOBcompensator();
        //  computeHeadingCDOBcompensator();
        //  computeLateralCDOBcompensator();
        //  computeVelocityCDOBcompensator();
        //  computeAccelerationCDOBcompensator();

        // Publish delay compensation reference.
        publishCompensationReferences();

        // Debug
        {
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Hello world");

            // RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Hello World!");
            // RCLCPP_ERROR(get_logger(), "Trajectory is invalid!, stop computing.");
            // RCLCPP_DEBUG(get_logger(), "MPC does not have a QP solver");
            //  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Timer");
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Control frequency  %4.2f ",
            //    params_node_.cdob_ctrl_period);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter ey order %4.2i ",
            //    params_node_.qfilter_lateral_error_order);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter eyaw order %4.2i ",
            //    params_node_.qfilter_heading_error_order);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter steering order %4.2i ",
            //    params_node_.qfilter_steering_order);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter velocity order %4.2i ",
            //    params_node_.qfilter_velocity_error_order);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter ey frq %4.2f ",
            //    params_node_.qfilter_lateral_error_freq);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter eyaw frq %4.2f ",
            //    params_node_.qfilter_heading_error_freq);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter steering frq %4.2f ",
            //    params_node_.qfilter_steering_freq);
            //
            //  RCLCPP_INFO_THROTTLE(
            //    get_logger(), *get_clock(), (1000ms).count(), "Qfilter velocity frq %4.2f ",
            //    params_node_.qfilter_velocity_error_freq);

            // 			if (delay_compensator_steering_error_)
            // 			{
            // 				delay_compensator_steering_error_->print();
            // 			}
            // 			else
            // 			{
            // 				ns_utils::print("Unique pointer is not set ");
            // 			}

            // 			if (delay_compensator_heading_error_)
            // 			{
            // 				delay_compensator_heading_error_->print();
            // 			}
            // 			else
            // 			{
            // 				ns_utils::print("Unique pointer is not set ");
            // 			}

            // 			if (delay_compensator_lat_error_)
            // 			{
            // 				delay_compensator_lat_error_->print();
            // 			}
            // 			else
            // 			{
            // 				ns_utils::print("Unique pointer is not set ");
            // 			}

            // 			if (delay_compensator_acc_error_)
            // 			{
            // 				delay_compensator_acc_error_->print();
            // 			}
            // 			else
            // 			{
            // 				ns_utils::print("Unique pointer is not set ");
            // 			}

            // ns_utils::print("ACT On timer method ");
            // 			ns_utils::print("Params wheelbase ", params_node_.wheel_base);
            // 			ns_utils::print("Params qfilter ey order ",
            // params_node_.qfilter_lateral_error_order); 			ns_utils::print("Params
            // qfilter_velocity_error_freq
            // ", params_node_.qfilter_velocity_error_freq); end of debug
        }
    }

    void CommunicationDelayCompensatorNode::onControlCommands(const ControlCommand::SharedPtr msg) {
        previous_ctrl_ptr_ = current_ctrl_ptr_;
        current_ctrl_ptr_ = std::make_shared<ControlCommand>(*msg);

        // Debug
        // ns_utils::print("ACT On control method ");
        // ns_utils::print("Read parameter control period :", params_node_.cdob_ctrl_period);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "onControlCommands");
        // end of debug
    }

    void CommunicationDelayCompensatorNode::onCurrentVelocity(const VelocityMsg::SharedPtr msg) {
        if (current_velocity_ptr) {
            previous_velocity_ = current_velocity_ptr->twist.twist.linear.x;
        }

        current_velocity_ptr = std::make_shared<VelocityMsg>(*msg);

        // ns_utils::print("ACT On velocity method ");
        // ns_utils::print("Read parameter control period :", params_node_.cdob_ctrl_period);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Velocity");
    }

    void CommunicationDelayCompensatorNode::onCurrentLongitudinalError(
            ControllerErrorReportMsg::SharedPtr const msg) {
        current_longitudinal_errors_ = std::make_shared<ControllerErrorReportMsg>(*msg);

        // Debug
        // auto vel_error = static_cast<double>(current_longitudinal_errors_->velocity_error_read);
        // ns_utils::print("Longitudinal velocity error :", vel_error);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Longitudinal Error");
        // end of debug
    }

    void CommunicationDelayCompensatorNode::onCurrentLateralErrors(
            ControllerErrorReportMsg::SharedPtr const msg) {
        current_lateral_errors_ = std::make_shared<ControllerErrorReportMsg>(*msg);

        // Debug
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Lateral Errors");

        // 			if (current_lateral_errors_)
        // 			{
        // 				auto lat_error =
        // static_cast<double>(current_lateral_errors_->lateral_deviation_read);
        // auto heading_error = static_cast<double>(current_lateral_errors_->heading_angle_error_read);
        // 				ns_utils::print("Current lateral errors : ", lat_error,
        // heading_error);
        // 			}
        // end of debug.
    }

    void CommunicationDelayCompensatorNode::onControlPerfErrors(
            const ErrorStampedControlPerfMsg::SharedPtr msg) {
        current_cont_perf_errors_ = std::make_shared<ErrorStampedControlPerfMsg>(*msg);

        // Debug
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "On Control Perf. Errors");
    }

    void CommunicationDelayCompensatorNode::publishCompensationReferences() {
        current_delay_references_msg_->stamp = this->now();
        current_delay_debug_msg_->stamp = this->now();

        // new_msg.lateral_deviation_error_compensation_ref = 1.0;
        pub_delay_compensator_->publish(*current_delay_references_msg_);
        pub_delay_compensator_debug_->publish(*current_delay_debug_msg_);
    }

    void CommunicationDelayCompensatorNode::onCurrentSteering(const SteeringReport::SharedPtr msg) {
        current_steering_ptr_ = std::make_shared<SteeringReport>(*msg);

        // Debug
        RCLCPP_WARN_SKIPFIRST_THROTTLE(
                get_logger(), *get_clock(), (1000ms).count(), "[communication_delay] On Steering  ...");
        // ns_utils::print("ACT On steering method ");
        // end of debug
    }

    bool8_t CommunicationDelayCompensatorNode::isDataReady() {
        if (!current_velocity_ptr) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(
                    get_logger(), *get_clock(), (1000ms).count(),
                    "[communication_delay] Waiting for the velocity measurement ...");
            return false;
        }

        if (!current_steering_ptr_) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(
                    get_logger(), *get_clock(), (1000ms).count(),
                    "[communication_delay] Waiting for the steering measurement ...");
            return false;
        }

        if (!current_ctrl_ptr_) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(
                    get_logger(), *get_clock(), (1000ms).count(),
                    "[communication_delay] Waiting for the control command ...");
            return false;
        }

        return true;
    }

    void CommunicationDelayCompensatorNode::readAndLoadParameters() {
        try {
            // Read the filter orders.
            params_node_.cdob_ctrl_period = declare_parameter<float64_t>("cdob_ctrl_period");  // reads sec.
            params_node_.qfilter_lateral_error_order =
                    declare_parameter<int>("qfilter_lateral_error_order");
            params_node_.qfilter_heading_error_order =
                    declare_parameter<int>("qfilter_heading_error_order");
            params_node_.qfilter_steering_order = declare_parameter<int>("qfilter_steering_order");
            params_node_.qfilter_velocity_error_order =
                    declare_parameter<int>("qfilter_velocity_error_order");
            params_node_.qfilter_acc_error_order = declare_parameter<int>("qfilter_acc_error_order");

            // Read the filter cut-oof frequencies.
            params_node_.qfilter_lateral_error_freq =
                    declare_parameter<float64_t>("qfilter_lateral_error_freq");
            params_node_.qfilter_heading_error_freq =
                    declare_parameter<float64_t>("qfilter_heading_error_freq");
            params_node_.qfilter_steering_freq = declare_parameter<float64_t>("qfilter_steering_freq");
            params_node_.qfilter_velocity_error_freq =
                    declare_parameter<float64_t>("qfilter_velocity_error_freq");
            params_node_.qfilter_acc_error_freq = declare_parameter<float64_t>("qfilter_acc_error_freq");

            // First order state dynamics parameters.
            params_node_.steering_tau = declare_parameter<float64_t>("steering_time_constant_");
            params_node_.velocity_tau = declare_parameter<float64_t>("velocity_time_constant_");
            params_node_.acc_tau = declare_parameter<float64_t>("acc_time_constant_");

        }

        catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
        }
    }

    rcl_interfaces::msg::SetParametersResult CommunicationDelayCompensatorNode::onParameterUpdate(
            const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        try {
            update_param(parameters, "cdob_ctrl_period", params_node_.cdob_ctrl_period);

            update_param(
                    parameters, "qfilter_lateral_error_order", params_node_.qfilter_lateral_error_order);
            update_param(
                    parameters, "qfilter_heading_error_order", params_node_.qfilter_heading_error_order);
            update_param(parameters, "qfilter_steering_order", params_node_.qfilter_steering_order);
            update_param(
                    parameters, "qfilter_velocity_error_order", params_node_.qfilter_velocity_error_order);
            update_param(parameters, "qfilter_acc_error_order", params_node_.qfilter_acc_error_order);

            update_param(parameters, "qfilter_lateral_error_freq", params_node_.qfilter_lateral_error_freq);
            update_param(parameters, "qfilter_heading_error_freq", params_node_.qfilter_heading_error_freq);
            update_param(parameters, "qfilter_steering_freq", params_node_.qfilter_steering_freq);
            update_param(
                    parameters, "qfilter_velocity_error_freq", params_node_.qfilter_velocity_error_freq);
            update_param(parameters, "qfilter_acc_error_freq", params_node_.qfilter_acc_error_freq);

            update_param(parameters, "steering_time_constant_", params_node_.steering_tau);
            update_param(parameters, "velocity_time_constant_", params_node_.velocity_tau);
            update_param(parameters, "acc_time_constant_", params_node_.acc_tau);

        }

            // transaction succeeds, now assign values
        catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
            result.successful = false;
            result.reason = e.what();
        }

        for (const auto &param: parameters) {
            RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
        }

        return result;
    }

    /**
     * @brief Creates a qfilter Q(s), a transfer function G(s) from steering input to steering state and
     * Q(s)/G(s). The transfer function from steering->heading is G(s) = 1 / (tau_steer*s + 1)
     * */
    void CommunicationDelayCompensatorNode::setSteeringCDOBcompensator() {
        // Create a qfilter from the given order for the steering system.
        auto const &order_of_q = params_node_.qfilter_steering_order;
        auto const &cut_off_frq_in_hz_q = params_node_.qfilter_steering_freq;

        // --------------- Qfilter Construction --------------------------------------
        // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n

        // Create the transfer function from a numerator an denominator.
        auto q_tf = get_nthOrderTF(cut_off_frq_in_hz_q, order_of_q);

        // --------------- System Model Construction --------------------------------------
        // There is no dynamical parameter but tau might be changing if we use the adaptive control
        // approach.

        auto g_tf = tf_t({1.}, {params_node_.steering_tau, 1.});
        CommunicationDelayCompensatorCore delay_compensator_steering(
                q_tf, g_tf, params_node_.cdob_ctrl_period);

        // Store as an unique ptr.
        delay_compensator_steering_error_ =
                std::make_unique<CommunicationDelayCompensatorCore>(delay_compensator_steering);
    }

/**
 * @brief Computes a corrective reference signal to subtract from the current steering error
 * reference in the form of  r_steering_corrected = r_steering - cdob_corrrection. Additional
 * variables are computed for debugging purpose.
 * */
    void CommunicationDelayCompensatorNode::computeSteeringCDOBcompensator() {
        // Get the previous steering control value sent to the vehicle.
        auto &u_prev = previous_ctrl_ptr_->lateral.steering_tire_angle;

        // Get the current measured steering value.
        auto &current_steering = current_steering_ptr_->steering_tire_angle;

        // Compute current steering error.
        auto &curvature = current_cont_perf_errors_->error.curvature_estimate;

        // Ackerman Ideal Steering
        auto const &&ideal_ackerman_steering = std::atan(curvature * params_node_.wheel_base);

        // Error: current_val - target (ref)_val
        auto const &&steering_error = current_steering - ideal_ackerman_steering;

        // reset the stored outputs to zero.
        // std::fill(cdob_steering_error_y_outputs_.begin(), cdob_steering_error_y_outputs_.end(), 0.);

        // Input is steering_input -> G(s) steering model --> next steering state.
        delay_compensator_steering_error_->simulateOneStep(u_prev, steering_error);
        cdob_steering_error_y_outputs_ = delay_compensator_steering_error_->getOutputs();

        /**
         * @brief Outputs of the delay compensator.
         * y0: u_filtered,Q(s)*u where u is the input sent to the system.
         * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
         * y2: du = y0 - y1 where du is the estimated disturbance input
         * y3: ydu = G(s)*du where ydu is the response of the system to du.
         * */

        // Set delay_compensation_reference for the steering.
        current_delay_references_msg_->steering_error_read = static_cast<float>(steering_error);
        current_delay_references_msg_->steering_error_compensation_ref =
                static_cast<float>(cdob_steering_error_y_outputs_[4]);

        // Set debug message.
        current_delay_debug_msg_->steering_uf = static_cast<float>(cdob_steering_error_y_outputs_[0]);
        current_delay_debug_msg_->steering_u_du = static_cast<float>(cdob_steering_error_y_outputs_[1]);
        current_delay_debug_msg_->steering_du = static_cast<float>(cdob_steering_error_y_outputs_[2]);
        current_delay_debug_msg_->steering_ydu = static_cast<float>(cdob_steering_error_y_outputs_[3]);
        current_delay_debug_msg_->steering_yu =
                static_cast<float>(cdob_steering_error_y_outputs_[4]);  // to sum or subtract from ref.

        current_delay_debug_msg_->steering_nondelay_u_estimated =
                static_cast<float>(cdob_steering_error_y_outputs_[1] + cdob_steering_error_y_outputs_[2]);

        // Debug
        // ns_utils::print("previous input : ", u_prev, current_steering);
    }

/**
 * @brief Creates a qfilter Q(s), a transfer function G(s) from steering to heading angle and
 * Q(s)/G(s). The transfer function from steering->heading is G(s) = V / (cos(delta)**2 * wheelbase
 * (tau_steer*s + 1))
 * */
    void CommunicationDelayCompensatorNode::setHeadingErrorCDOBcompensator() {
        // Create a qfilter for he steering to heading transfer function.
        // Compute the cut-off frequency in rad/sec.
        auto const &order_of_q = params_node_.qfilter_heading_error_order;
        auto const &cut_off_frq_in_hz_q = params_node_.qfilter_heading_error_freq;
        auto &&w_c_of_q = 2.0 * M_PI * cut_off_frq_in_hz_q;  // in [rad/sec]

        // float64_t time_constant_of_qfilter{};
        auto const &time_constant_of_qfilter = 1.0 / w_c_of_q;

        // --------------- Qfilter Construction --------------------------------------
        // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
        // Calculate the transfer function.
        ns_control_toolbox::tf_factor denominator{
                std::vector<double>{time_constant_of_qfilter, 1.}};  // (tau*s+1)

        // Take power of the denominator.
        denominator.power(static_cast<unsigned int>(order_of_q));

        // Create the transfer function from a numerator an denominator.
        auto q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

        // --------------- System Model Construction --------------------------------------
        // There are dynamically changing numerator and denominator coefficients.
        // We store the factored num and denominators:  a(var1) * num / b(var1)*den where num-den are
        // constants.
        std::pair<std::string_view, std::string_view> param_names{"v", "delta"};

        // Functions of v, and delta.
        std::unordered_map<std::string_view, func_type<double>> f_variable_num_den_funcs{};

        f_variable_num_den_funcs["v"] = [](auto const &x) -> double {
            return std::fabs(x) < 1. ? 1. : x;
        };  // to prevent zero division.

        f_variable_num_den_funcs["delta"] = [](auto const &x) -> double {
            return std::cos(x) * std::cos(x);
        };

        // Create G(s) without varying parameters using only the constant parts.
        ns_control_toolbox::tf_factor m_den1{{params_node_.wheel_base, 0}};    // L*s
        ns_control_toolbox::tf_factor m_den2{{params_node_.steering_tau, 1}};  // (tau*s + 1)
        auto den_tf_factor = m_den1 * m_den2;                                  // Ls*(tau*s + 1)

        // 1. / 1.*Ls*(tau*s + 1) where 1., 1. are replaced by the functions.
        auto g_tf = tf_t({1.}, den_tf_factor(), 1., 1.);  // num, den, num_constant, den_constant

        CommunicationDelayCompensatorCore delay_compensator_heading(
                q_tf, g_tf, params_node_.cdob_ctrl_period);

        // Set the mapping functions of the delay compensator.
        delay_compensator_heading.setDynamicParams_num_den(param_names, f_variable_num_den_funcs);

        // Store as an unique ptr.
        delay_compensator_heading_error_ =
                std::make_unique<CommunicationDelayCompensatorCore>(delay_compensator_heading);
    }

/**
 * @brief Computes a corrective reference signal to subtract from the current heading error
 * reference in the form of  r_heading_corrected = r_heading - cdob_corrrection. Additional
 * variables are computed for debugging purpose.
 * */
    void CommunicationDelayCompensatorNode::computeHeadingCDOBcompensator() {
        // Get the previous steering control value sent to the vehicle.
        auto &u_prev = previous_ctrl_ptr_->lateral.steering_tire_angle;

        // Get the current measured steering value.
        auto &current_steering = current_steering_ptr_->steering_tire_angle;

        // Get the current longitudinal speed.
        auto &current_speed_v = current_velocity_ptr->twist.twist.linear.x;

        // Send the current states [v, delta] to the delay compensator.
        std::pair<float64_t, float64_t> varying_params({current_speed_v, current_steering});

        // Get the current heading error computed by the controllers.
        auto &current_heading_error = current_lateral_errors_->heading_angle_error_read;

        // Input is steering_input -> G(s) heading error model --> next heading state.
        delay_compensator_heading_error_->simulateOneStep(u_prev, current_heading_error, varying_params);
        cdob_heading_error_y_outputs_ = delay_compensator_heading_error_->getOutputs();

        /**
         * @brief Outputs of the delay compensator.
         * y0: u_filtered,Q(s)*u where u is the input sent to the system.
         * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
         * y2: du = y0 - y1 where du is the estimated disturbance input
         * y3: ydu = G(s)*du where ydu is the response of the system to du.
         * */
        current_delay_references_msg_->heading_angle_error_read = current_heading_error;
        current_delay_references_msg_->heading_angle_error_compensation_ref =
                static_cast<float>(cdob_heading_error_y_outputs_[4]);

        // Set debug message.
        current_delay_debug_msg_->heading_uf = static_cast<float>(cdob_heading_error_y_outputs_[0]);
        current_delay_debug_msg_->heading_u_du = static_cast<float>(cdob_heading_error_y_outputs_[1]);
        current_delay_debug_msg_->heading_du = static_cast<float>(cdob_heading_error_y_outputs_[2]);
        current_delay_debug_msg_->heading_ydu = static_cast<float>(cdob_heading_error_y_outputs_[3]);
        current_delay_debug_msg_->heading_yu =
                static_cast<float>(cdob_heading_error_y_outputs_[4]);  // to sum or
        // subtract from ref.

        current_delay_debug_msg_->heading_nondelay_u_estimated =
                static_cast<float>(cdob_heading_error_y_outputs_[1] + cdob_heading_error_y_outputs_[2]);

        // Debug
        // ns_utils::print("previous input : ", u_prev, current_steering);
    }

    void CommunicationDelayCompensatorNode::setLateralErrorCDOBcompensator() {
        // Create a qfilter for he steering to heading transfer function.
        // Compute the cut-off frequency in rad/sec.
        auto const &order_of_q = params_node_.qfilter_lateral_error_order;
        auto const &cut_off_frq_in_hz_q = params_node_.qfilter_lateral_error_freq;
        auto &&w_c_of_q = 2.0 * M_PI * cut_off_frq_in_hz_q;  // in [rad/sec]

        auto const &time_constant_of_qfilter = 1.0 / w_c_of_q;

        // --------------- Qfilter Construction --------------------------------------
        // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
        // Calculate the transfer function.

        // We use 1 /( tau*s + 1)&^n
        ns_control_toolbox::tf_factor denominator{
                std::vector<double>{time_constant_of_qfilter, 1.}};  // (tau*s+1)

        // Take power of the denominator.
        denominator.power(static_cast<unsigned int>(order_of_q));

        // if we use damping formulation
        ns_control_toolbox::tf_factor den1{
                std::vector<double>{time_constant_of_qfilter, 1.}};  // (tau*s+1)


        // Create the transfer function from a numerator an denominator.
        auto q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

        // --------------- System Model Construction --------------------------------------
        // There are dynamically changing numerator and denominator coefficients.
        // We store the factored num and denominators:  a(var1) * num / b(var1)*den where num-den are
        // constants.
        std::pair<std::string_view, std::string_view> param_names{"v", "delta"};

        // Functions of v, and delta.
        std::unordered_map<std::string_view, func_type<double>> f_variable_num_den_funcs{};

        f_variable_num_den_funcs["v"] = [](auto const &x) -> double {
            return std::fabs(x) < 1. ? 1. : x * x;
        };  // to prevent zero division.

        f_variable_num_den_funcs["delta"] = [](auto const &x) -> double {
            return std::cos(x) * std::cos(x);
        };

        // Create G(s) without varying parameters using only the constant parts.
        ns_control_toolbox::tf_factor m_den1{{params_node_.wheel_base, 0., 0.}};  // L*s^2
        ns_control_toolbox::tf_factor m_den2{{params_node_.steering_tau, 1}};     // (tau*s + 1)
        auto den_tf_factor = m_den1 * m_den2;                                     // Ls^2*(tau*s + 1)

        // 1. / 1.*Ls^2*(tau*s + 1) where 1., 1. are replaced by the functions.
        auto g_tf = tf_t({1.}, den_tf_factor(), 1., 1.);  // num, den, num_constant, den_constant

        CommunicationDelayCompensatorCore delay_compensator_lateral(
                q_tf, g_tf, params_node_.cdob_ctrl_period);

        // Set the mapping functions of the delay compensator.
        delay_compensator_lateral.setDynamicParams_num_den(param_names, f_variable_num_den_funcs);

        // Store as an unique ptr.
        delay_compensator_lat_error_ =
                std::make_unique<CommunicationDelayCompensatorCore>(delay_compensator_lateral);
    }

    void CommunicationDelayCompensatorNode::computeLateralCDOBcompensator() {
        // Get the previous steering control value sent to the vehicle.
        auto &u_prev = previous_ctrl_ptr_->lateral.steering_tire_angle;

        // Get the current measured steering value.
        auto &current_steering = current_steering_ptr_->steering_tire_angle;

        // Get the current longitudinal speed.
        auto &current_speed_v = current_velocity_ptr->twist.twist.linear.x;

        // Send the current states [v, delta] to the delay compensator.
        std::pair<float64_t, float64_t> varying_params({current_speed_v, current_steering});

        // Get the current heading error computed by the controllers.
        auto &current_lateral_error = current_lateral_errors_->lateral_deviation_read;

        // Input is steering_input -> G(s) heading error model --> next heading state.
        delay_compensator_lat_error_->simulateOneStep(u_prev, current_lateral_error, varying_params);
        cdob_lateral_error_y_outputs_ = delay_compensator_lat_error_->getOutputs();

        /**
         * @brief Outputs of the delay compensator.
         * y0: u_filtered,Q(s)*u where u is the input sent to the system.
         * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
         * y2: du = y0 - y1 where du is the estimated disturbance input
         * y3: ydu = G(s)*du where ydu is the response of the system to du.
         * */
        current_delay_references_msg_->lateral_deviation_read = current_lateral_error;
        current_delay_references_msg_->lateral_deviation_error_compensation_ref =
                static_cast<float>(cdob_lateral_error_y_outputs_[4]);

        // Set debug message.
        current_delay_debug_msg_->lat_uf = static_cast<float>(cdob_lateral_error_y_outputs_[0]);
        current_delay_debug_msg_->lat_u_du = static_cast<float>(cdob_lateral_error_y_outputs_[1]);
        current_delay_debug_msg_->lat_du = static_cast<float>(cdob_lateral_error_y_outputs_[2]);
        current_delay_debug_msg_->lat_ydu = static_cast<float>(cdob_lateral_error_y_outputs_[3]);
        current_delay_debug_msg_->lat_yu =
                static_cast<float>(cdob_lateral_error_y_outputs_[4]);  // to sum or
        // subtract from ref.

        current_delay_debug_msg_->lat_u_nondelay_u_estimated =
                static_cast<float>(cdob_lateral_error_y_outputs_[1] + cdob_lateral_error_y_outputs_[2]);

        // Debug
        // ns_utils::print("previous input : ", u_prev, current_steering);
    }

    void CommunicationDelayCompensatorNode::setVelocityErrorCDOBcompensator() {
        // Create a qfilter from the given order for the steering system.
        auto const &order_of_q = params_node_.qfilter_velocity_error_order;
        auto const &cut_off_frq_in_hz_q = params_node_.qfilter_velocity_error_freq;
        auto &&w_c_of_q = 2.0 * M_PI * cut_off_frq_in_hz_q;  // in [rad/sec]

        // float64_t time_constant_of_qfilter{};
        auto time_constant_of_qfilter = 1.0 / w_c_of_q;

        // --------------- Qfilter Construction --------------------------------------
        // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
        // Calculate the transfer function.
        ns_control_toolbox::tf_factor denominator{
                std::vector<double>{time_constant_of_qfilter, 1.}};  // (tau*s+1)

        // Take power of the denominator.
        denominator.power(static_cast<unsigned int>(order_of_q));

        // Create the transfer function from a numerator an denominator.
        auto q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

        // --------------- System Model Construction --------------------------------------
        // There is no dynamical parameter but tau might be changing if we use the adaptive control
        // approach.

        auto g_tf = tf_t({1.}, {params_node_.velocity_tau, 1.});
        CommunicationDelayCompensatorCore delay_compensator_velocity(
                q_tf, g_tf, params_node_.cdob_ctrl_period);

        // Store as an unique ptr.
        delay_compensator_velocity_error_ =
                std::make_unique<CommunicationDelayCompensatorCore>(delay_compensator_velocity);
    }

    void CommunicationDelayCompensatorNode::computeVelocityCDOBcompensator() {
        // Get the previous steering control value sent to the vehicle.
        auto &u_prev = previous_ctrl_ptr_->longitudinal.speed;

        // Get the current measured steering value.
        auto &current_velocity = current_velocity_ptr->twist.twist.linear.x;

        // reset the stored outputs to zero.
        // std::fill(cdob_steering_error_y_outputs_.begin(), cdob_steering_error_y_outputs_.end(), 0.);

        // Input is steering_input -> G(s) steering model --> next steering state.
        delay_compensator_velocity_error_->simulateOneStep(u_prev, current_velocity);
        cdob_velocity_error_y_outputs_ = delay_compensator_velocity_error_->getOutputs();

        /**
         * @brief Outputs of the delay compensator.
         * y0: u_filtered,Q(s)*u where u is the input sent to the system.
         * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
         * y2: du = y0 - y1 where du is the estimated disturbance input
         * y3: ydu = G(s)*du where ydu is the response of the system to du.
         * */

        // Set delay_compensation_reference for the steering.
        current_delay_references_msg_->velocity_error_read =
                current_longitudinal_errors_->velocity_error_read;
        current_delay_references_msg_->velocity_error_compensation_ref =
                static_cast<float>(cdob_velocity_error_y_outputs_[4]);

        // Set debug message.
        current_delay_debug_msg_->vel_uf = static_cast<float>(cdob_velocity_error_y_outputs_[0]);
        current_delay_debug_msg_->vel_u_du = static_cast<float>(cdob_velocity_error_y_outputs_[1]);
        current_delay_debug_msg_->vel_du = static_cast<float>(cdob_velocity_error_y_outputs_[2]);
        current_delay_debug_msg_->vel_ydu = static_cast<float>(cdob_velocity_error_y_outputs_[3]);
        current_delay_debug_msg_->vel_yu = static_cast<float>(cdob_velocity_error_y_outputs_[4]);

        current_delay_debug_msg_->vel_u_nondelay_u_estimated =
                static_cast<float>(cdob_velocity_error_y_outputs_[1] + cdob_velocity_error_y_outputs_[2]);

        // Debug
        // ns_utils::print("previous input : ", u_prev, current_steering);
    }

    void CommunicationDelayCompensatorNode::setAccelerationErrorCDOBcompensator() {
        // Create a qfilter from the given order for the steering system.
        auto const &order_of_q = params_node_.qfilter_acc_error_order;
        auto const &cut_off_frq_in_hz_q = params_node_.qfilter_acc_error_freq;
        auto &&w_c_of_q = 2.0 * M_PI * cut_off_frq_in_hz_q;  // in [rad/sec]

        // float64_t time_constant_of_qfilter{};
        auto time_constant_of_qfilter = 1.0 / w_c_of_q;

        // --------------- Qfilter Construction --------------------------------------
        // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
        // Calculate the transfer function.
        ns_control_toolbox::tf_factor denominator{
                std::vector<double>{time_constant_of_qfilter, 1.}};  // (tau*s+1)

        // Take power of the denominator.
        denominator.power(static_cast<unsigned int>(order_of_q));

        // Create the transfer function from a numerator an denominator.
        auto q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

        // --------------- System Model Construction --------------------------------------
        // There is no dynamical parameter but tau might be changing if we use the adaptive control
        // approach.

        auto g_tf = tf_t({1.}, {params_node_.acc_tau, 1.});
        CommunicationDelayCompensatorCore delay_compensator_acc(
                q_tf, g_tf, params_node_.cdob_ctrl_period);

        // Store as an unique ptr.
        delay_compensator_acc_error_ =
                std::make_unique<CommunicationDelayCompensatorCore>(delay_compensator_acc);
    }

    void CommunicationDelayCompensatorNode::computeAccelerationCDOBcompensator() {
        // Get the previous steering control value sent to the vehicle.
        auto &u_prev = previous_ctrl_ptr_->longitudinal.acceleration;

        // Get the current measured steering value.
        auto current_acceleration = (current_velocity_ptr->twist.twist.linear.x - previous_velocity_) /
                                    params_node_.cdob_ctrl_period;

        // reset the stored outputs to zero.
        // std::fill(cdob_steering_error_y_outputs_.begin(), cdob_steering_error_y_outputs_.end(), 0.);

        // Input is steering_input -> G(s) steering model --> next steering state.
        delay_compensator_acc_error_->simulateOneStep(u_prev, current_acceleration);
        cdob_acc_error_y_outputs_ = delay_compensator_acc_error_->getOutputs();

        /**
         * @brief Outputs of the delay compensator.
         * y0: u_filtered,Q(s)*u where u is the input sent to the system.
         * y1: u-d_u = (Q(s)/G(s))*y_system where y_system is the measured system response.
         * y2: du = y0 - y1 where du is the estimated disturbance input
         * y3: ydu = G(s)*du where ydu is the response of the system to du.
         * */

        // Set delay_compensation_reference for the steering.
        current_delay_references_msg_->acceleration_error_read =
                current_longitudinal_errors_->acceleration_error_read;
        current_delay_references_msg_->acceleration_error_compensation_ref =
                static_cast<float>(cdob_acc_error_y_outputs_[4]);

        // Set debug message.
        current_delay_debug_msg_->acc_uf = static_cast<float>(cdob_acc_error_y_outputs_[0]);
        current_delay_debug_msg_->acc_u_du = static_cast<float>(cdob_acc_error_y_outputs_[1]);
        current_delay_debug_msg_->acc_du = static_cast<float>(cdob_acc_error_y_outputs_[2]);
        current_delay_debug_msg_->acc_ydu = static_cast<float>(cdob_acc_error_y_outputs_[3]);
        current_delay_debug_msg_->acc_yu = static_cast<float>(cdob_acc_error_y_outputs_[4]);

        current_delay_debug_msg_->acc_u_nondelay_u_estimated =
                static_cast<float>(cdob_acc_error_y_outputs_[1] + cdob_acc_error_y_outputs_[2]);

        // Debug
        // ns_utils::print("previous input : ", u_prev, current_steering);
    }

    bool8_t CommunicationDelayCompensatorNode::isVehicleStopping() {
        auto current_vel = current_velocity_ptr->twist.twist.linear.x;
        return std::fabs(current_vel) <= 0.5;
    }

}  // namespace observers
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(observers::CommunicationDelayCompensatorNode)