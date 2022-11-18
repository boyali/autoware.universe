//
// Created by ali on 18/11/22.
//

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
};

} // namespace autoware::motion::control::trajectory_follower

#endif //TRAJECTORY_FOLLOWER_INCLUDE_TRAJECTORY_FOLLOWER_SYSID_STEERING_INPUT_GENERATOR_SYSID_STEERING_INPUT_GENERATOR_HPP_
