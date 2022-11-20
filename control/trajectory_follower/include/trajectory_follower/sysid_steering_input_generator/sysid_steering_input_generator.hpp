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

#ifndef TRAJECTORY_FOLLOWER_INCLUDE_TRAJECTORY_FOLLOWER_SYSID_STEERING_INPUT_GENERATOR_SYSID_STEERING_INPUT_GENERATOR_HPP_
#define TRAJECTORY_FOLLOWER_INCLUDE_TRAJECTORY_FOLLOWER_SYSID_STEERING_INPUT_GENERATOR_SYSID_STEERING_INPUT_GENERATOR_HPP_

#include "common/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "trajectory_follower/lateral_controller_base.hpp"

#include "trajectory_follower/visibility_control.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_system_msgs/msg/float32_multi_array_diagnostic.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"
#include "signal_processing/lowpass_filter.hpp"

namespace autoware::motion::control::trajectory_follower
{

class TRAJECTORY_FOLLOWER_PUBLIC SysIDLateralController : public LateralControllerBase
{
 public:
  /**
 * @brief constructor
 */
  explicit SysIDLateralController(rclcpp::Node &node);

  /**
   * @brief destructor
   */
  virtual ~SysIDLateralController();

 private:
  rclcpp::Node *node_;

  // DATA MEMBERS
  //!< @brief measured pose
  geometry_msgs::msg::PoseStamped::SharedPtr m_current_pose_ptr_;

  //!< @brief measured velocity
  nav_msgs::msg::Odometry::SharedPtr m_current_velocity_ptr_;

  //!< @brief measured steering
  autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr m_current_steering_ptr_;

  //!< @brief reference trajectory
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr m_current_trajectory_ptr_;

  //!< @brief buffer for transforms
  tf2::BufferCore m_tf_buffer_{tf2::BUFFER_CORE_DEFAULT_CACHE_TIME};
  tf2_ros::TransformListener m_tf_listener_{m_tf_buffer_};

  double signal_mag_{};  // maximum magnitude of the control signal
  double min_speed_{};   // the speed over which the signal is generated
  double max_speed_{};   // the speed guard to make the input zero.

  // INTERFACE Methods
  /**
   * @brief compute control command for path follow with a constant control period
   */
  boost::optional<LateralOutput> run() override;

  /**
   * @brief set input data like current odometry, trajectory and steering.
   */
  void setInputData(InputData const &input_data) override;

  /**
   * @brief create control command
   * @param [in] ctrl_cmd published control command
   */
  autoware_auto_control_msgs::msg::AckermannLateralCommand createCtrlCmdMsg(autoware_auto_control_msgs::msg::AckermannLateralCommand ctrl_cmd);

  bool updateCurrentPose();

  void loadParams();
  bool checkData() const;
};

} // namespace autoware::motion::control::trajectory_follower

#endif //TRAJECTORY_FOLLOWER_INCLUDE_TRAJECTORY_FOLLOWER_SYSID_STEERING_INPUT_GENERATOR_SYSID_STEERING_INPUT_GENERATOR_HPP_
