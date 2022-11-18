//
// Created by ali on 18/11/22.
//

#include "trajectory_follower/sysid_steering_input_generator/sysid_steering_input_generator.hpp"

namespace autoware::motion::control::trajectory_follower
{

SysIDLateralController::SysIDLateralController(rclcpp::Node &node) : node_{&node}
{

  using std::placeholders::_1;

}

SysIDLateralController::~SysIDLateralController() = default;
} // namespace autoware::motion::control::trajectory_follower
