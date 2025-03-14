// Copyright 2020 Tier IV, Inc.
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

#include "scenario_selector/scenario_selector_node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <class T>
void onData(const T & data, T * buffer)
{
  *buffer = data;
}

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & search_point)
{
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, 1);

  if (nearest_lanelets.empty()) {
    return {};
  }

  const auto nearest_lanelet = nearest_lanelets.front().second;
  const auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);

  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    nearest_lanelet, all_parking_lots, linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

geometry_msgs::msg::PoseStamped::ConstSharedPtr getCurrentPose(
  const tf2_ros::Buffer & tf_buffer, const rclcpp::Logger & logger)
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "%s", ex.what());
    return nullptr;
  }

  geometry_msgs::msg::PoseStamped::SharedPtr p(new geometry_msgs::msg::PoseStamped());
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return geometry_msgs::msg::PoseStamped::ConstSharedPtr(p);
}

bool isInLane(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Point & current_pos)
{
  const auto & p = current_pos;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point.basicPoint2d(), 1);

  if (nearest_lanelets.empty()) {
    return false;
  }

  const auto nearest_lanelet = nearest_lanelets.front().second;

  return lanelet::geometry::within(search_point, nearest_lanelet.polygon3d());
}

bool isInParkingLot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
    findNearestParkinglot(lanelet_map_ptr, search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

bool isNearTrajectoryEnd(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory,
  const geometry_msgs::msg::Pose & current_pose, const double th_dist)
{
  if (!trajectory || trajectory->points.empty()) {
    return false;
  }

  const auto & p1 = current_pose.position;
  const auto & p2 = trajectory->points.back().pose.position;

  const auto dist = std::hypot(p1.x - p2.x, p1.y - p2.y);

  return dist < th_dist;
}

bool isStopped(
  const std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> & twist_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & twist : twist_buffer) {
    if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

}  // namespace

autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr
ScenarioSelectorNode::getScenarioTrajectory(const std::string & scenario)
{
  if (scenario == tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    return lane_driving_trajectory_;
  }
  if (scenario == tier4_planning_msgs::msg::Scenario::PARKING) {
    return parking_trajectory_;
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "invalid scenario argument: " << scenario);
  return lane_driving_trajectory_;
}

std::string ScenarioSelectorNode::selectScenarioByPosition()
{
  const auto is_in_lane = isInLane(lanelet_map_ptr_, current_pose_->pose.position);
  const auto is_goal_in_lane = isInLane(lanelet_map_ptr_, route_->goal_pose.position);
  const auto is_in_parking_lot = isInParkingLot(lanelet_map_ptr_, current_pose_->pose);

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::EMPTY) {
    if (is_in_lane && is_goal_in_lane) {
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    } else if (is_in_parking_lot) {
      return tier4_planning_msgs::msg::Scenario::PARKING;
    } else {
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    }
  }

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    if (is_in_parking_lot && !is_goal_in_lane) {
      return tier4_planning_msgs::msg::Scenario::PARKING;
    }
  }

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::PARKING) {
    bool is_parking_completed{};
    this->get_parameter<bool>("is_parking_completed", is_parking_completed);
    if (is_parking_completed && is_in_lane) {
      this->set_parameter(rclcpp::Parameter("is_parking_completed", false));
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    }
  }

  return current_scenario_;
}

void ScenarioSelectorNode::updateCurrentScenario()
{
  const auto prev_scenario = current_scenario_;

  const auto scenario_trajectory = getScenarioTrajectory(current_scenario_);
  const auto is_near_trajectory_end =
    isNearTrajectoryEnd(scenario_trajectory, current_pose_->pose, th_arrived_distance_m_);

  const auto is_stopped = isStopped(twist_buffer_, th_stopped_velocity_mps_);

  if (is_near_trajectory_end && is_stopped) {
    current_scenario_ = selectScenarioByPosition();
  }

  if (current_scenario_ != prev_scenario) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "scenario changed: " << prev_scenario << " -> " << current_scenario_);
  }
}

void ScenarioSelectorNode::onMap(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void ScenarioSelectorNode::onRoute(
  const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr msg)
{
  route_ = msg;
  current_scenario_ = tier4_planning_msgs::msg::Scenario::EMPTY;
}

void ScenarioSelectorNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist->header = msg->header;
  twist->twist = msg->twist.twist;

  twist_ = twist;
  twist_buffer_.push_back(twist);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(twist_buffer_.front()->header.stamp);

    if (time_diff.seconds() < th_stopped_time_sec_) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

void ScenarioSelectorNode::onTimer()
{
  current_pose_ = getCurrentPose(tf_buffer_, this->get_logger());

  // Check all inputs are ready
  if (!current_pose_ || !lanelet_map_ptr_ || !route_ || !twist_) {
    return;
  }

  // Initialize Scenario
  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::EMPTY) {
    current_scenario_ = selectScenarioByPosition();
  }

  updateCurrentScenario();
  tier4_planning_msgs::msg::Scenario scenario;
  scenario.current_scenario = current_scenario_;

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::PARKING) {
    scenario.activating_scenarios.push_back(current_scenario_);
  }

  pub_scenario_->publish(scenario);
}

void ScenarioSelectorNode::onLaneDrivingTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  lane_driving_trajectory_ = msg;

  if (current_scenario_ != tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    return;
  }

  publishTrajectory(msg);
}

void ScenarioSelectorNode::onParkingTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  parking_trajectory_ = msg;

  if (current_scenario_ != tier4_planning_msgs::msg::Scenario::PARKING) {
    return;
  }

  publishTrajectory(msg);
}

void ScenarioSelectorNode::publishTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  const auto now = this->now();
  const auto delay_sec = (now - msg->header.stamp).seconds();
  if (delay_sec <= th_max_message_delay_sec_) {
    pub_trajectory_->publish(*msg);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "trajectory is delayed: scenario = %s, delay = %f, th_max_message_delay = %f",
      current_scenario_.c_str(), delay_sec, th_max_message_delay_sec_);
  }
}

ScenarioSelectorNode::ScenarioSelectorNode(const rclcpp::NodeOptions & node_options)
: Node("scenario_selector", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  current_scenario_(tier4_planning_msgs::msg::Scenario::EMPTY),
  update_rate_(this->declare_parameter<double>("update_rate", 10.0)),
  th_max_message_delay_sec_(this->declare_parameter<double>("th_max_message_delay_sec", 1.0)),
  th_arrived_distance_m_(this->declare_parameter<double>("th_arrived_distance_m", 1.0)),
  th_stopped_time_sec_(this->declare_parameter<double>("th_stopped_time_sec", 1.0)),
  th_stopped_velocity_mps_(this->declare_parameter<double>("th_stopped_velocity_mps", 0.01))
{
  // Parameters
  this->declare_parameter<bool>("is_parking_completed", false);

  // Input
  sub_lane_driving_trajectory_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "input/lane_driving/trajectory", rclcpp::QoS{1},
      std::bind(&ScenarioSelectorNode::onLaneDrivingTrajectory, this, std::placeholders::_1));

  sub_parking_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/parking/trajectory", rclcpp::QoS{1},
    std::bind(&ScenarioSelectorNode::onParkingTrajectory, this, std::placeholders::_1));

  sub_lanelet_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/lanelet_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ScenarioSelectorNode::onMap, this, std::placeholders::_1));
  sub_route_ = this->create_subscription<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&ScenarioSelectorNode::onRoute, this, std::placeholders::_1));
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", rclcpp::QoS{100},
    std::bind(&ScenarioSelectorNode::onOdom, this, std::placeholders::_1));

  // Output
  pub_scenario_ =
    this->create_publisher<tier4_planning_msgs::msg::Scenario>("output/scenario", rclcpp::QoS{1});
  pub_trajectory_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "output/trajectory", rclcpp::QoS{1});

  // Timer Callback
  const auto period_ns = rclcpp::Rate(static_cast<double>(update_rate_)).period();

  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ScenarioSelectorNode::onTimer, this));

  // Wait for first tf
  while (rclcpp::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(this->get_logger(), "waiting for initial pose...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ScenarioSelectorNode)
