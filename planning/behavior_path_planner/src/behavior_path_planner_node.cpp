// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/behavior_path_planner_node.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"
#include "behavior_path_planner/scene_module/pull_over/pull_over_module.hpp"
#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}
}  // namespace

namespace behavior_path_planner
{
using tier4_planning_msgs::msg::PathChangeModuleId;
using vehicle_info_util::VehicleInfoUtil;

BehaviorPathPlannerNode::BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_path_planner", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  // data_manager
  {
    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->parameters = getCommonParam();
  }

  // publisher
  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);
  path_candidate_publisher_ = create_publisher<Path>("~/output/path_candidate", 1);
  turn_signal_publisher_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  hazard_signal_publisher_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);
  debug_drivable_area_publisher_ = create_publisher<OccupancyGrid>("~/debug/drivable_area", 1);
  debug_path_publisher_ = create_publisher<Path>("~/debug/path_for_visualize", 1);
  debug_avoidance_msg_array_publisher_ =
    create_publisher<AvoidanceDebugMsgArray>("~/debug/avoidance_debug_message_array", 1);

  // For remote operation
  plan_ready_publisher_ = create_publisher<PathChangeModule>("~/output/ready", 1);
  plan_running_publisher_ = create_publisher<PathChangeModuleArray>("~/output/running", 1);
  force_available_publisher_ =
    create_publisher<PathChangeModuleArray>("~/output/force_available", 1);

  // Debug
  debug_marker_publisher_ = create_publisher<MarkerArray>("~/debug/markers", 1);

  if (planner_data_->parameters.visualize_drivable_area_for_shared_linestrings_lanelet) {
    debug_drivable_area_lanelets_publisher_ =
      create_publisher<MarkerArray>("~/drivable_area_boundary", 1);
  }

  // subscriber
  velocity_subscriber_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&BehaviorPathPlannerNode::onVelocity, this, _1),
    createSubscriptionOptions(this));
  perception_subscriber_ = create_subscription<PredictedObjects>(
    "~/input/perception", 1, std::bind(&BehaviorPathPlannerNode::onPerception, this, _1),
    createSubscriptionOptions(this));
  scenario_subscriber_ = create_subscription<Scenario>(
    "~/input/scenario", 1,
    [this](const Scenario::ConstSharedPtr msg) {
      current_scenario_ = std::make_shared<Scenario>(*msg);
    },
    createSubscriptionOptions(this));
  external_approval_subscriber_ = create_subscription<ApprovalMsg>(
    "~/input/external_approval", 1,
    std::bind(&BehaviorPathPlannerNode::onExternalApproval, this, _1),
    createSubscriptionOptions(this));
  force_approval_subscriber_ = create_subscription<PathChangeModule>(
    "~/input/force_approval", 1, std::bind(&BehaviorPathPlannerNode::onForceApproval, this, _1),
    createSubscriptionOptions(this));

  // route_handler
  auto qos_transient_local = rclcpp::QoS{1}.transient_local();
  vector_map_subscriber_ = create_subscription<HADMapBin>(
    "~/input/vector_map", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onMap, this, _1),
    createSubscriptionOptions(this));
  route_subscriber_ = create_subscription<HADMapRoute>(
    "~/input/route", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onRoute, this, _1),
    createSubscriptionOptions(this));

  // behavior tree manager
  {
    mutex_bt_.lock();

    bt_manager_ = std::make_shared<BehaviorTreeManager>(*this, getBehaviorTreeManagerParam());

    auto side_shift_module =
      std::make_shared<SideShiftModule>("SideShift", *this, getSideShiftParam());
    bt_manager_->registerSceneModule(side_shift_module);

    auto avoidance_module =
      std::make_shared<AvoidanceModule>("Avoidance", *this, getAvoidanceParam());
    bt_manager_->registerSceneModule(avoidance_module);

    auto lane_following_module =
      std::make_shared<LaneFollowingModule>("LaneFollowing", *this, getLaneFollowingParam());
    bt_manager_->registerSceneModule(lane_following_module);

    const auto lane_change_param = getLaneChangeParam();

    auto lane_change_module =
      std::make_shared<LaneChangeModule>("LaneChange", *this, lane_change_param);
    bt_manager_->registerSceneModule(lane_change_module);

    auto force_lane_change_module =
      std::make_shared<LaneChangeModule>("ForceLaneChange", *this, lane_change_param);
    bt_manager_->registerSceneModule(force_lane_change_module);

    bt_manager_->registerForceApproval("ForceLaneChange");

    auto pull_over_module = std::make_shared<PullOverModule>("PullOver", *this, getPullOverParam());
    bt_manager_->registerSceneModule(pull_over_module);

    auto pull_out_module = std::make_shared<PullOutModule>("PullOut", *this, getPullOutParam());
    bt_manager_->registerSceneModule(pull_out_module);

    bt_manager_->createBehaviorTree();

    mutex_bt_.unlock();
  }

  // turn signal decider
  {
    double intersection_search_distance{declare_parameter("intersection_search_distance", 30.0)};
    turn_signal_decider_.setParameters(
      planner_data_->parameters.base_link2front, intersection_search_distance);
  }

  waitForData();

  // Start timer. This must be done after all data (e.g. vehicle pose, velocity) are ready.
  {
    const auto planning_hz = declare_parameter("planning_hz", 10.0);
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&BehaviorPathPlannerNode::run, this));
  }
}

BehaviorPathPlannerParameters BehaviorPathPlannerNode::getCommonParam()
{
  // ROS parameters
  BehaviorPathPlannerParameters p{};
  p.backward_path_length = declare_parameter("backward_path_length", 5.0);
  p.forward_path_length = declare_parameter("forward_path_length", 100.0);
  p.backward_length_buffer_for_end_of_lane =
    declare_parameter("backward_length_buffer_for_end_of_lane", 5.0);
  p.backward_length_buffer_for_end_of_pull_over =
    declare_parameter("backward_length_buffer_for_end_of_pull_over", 5.0);
  p.backward_length_buffer_for_end_of_pull_out =
    declare_parameter("backward_length_buffer_for_end_of_pull_out", 5.0);
  p.minimum_lane_change_length = declare_parameter("minimum_lane_change_length", 8.0);
  p.minimum_pull_over_length = declare_parameter("minimum_pull_over_length", 15.0);
  p.drivable_area_resolution = declare_parameter<double>("drivable_area_resolution");
  p.drivable_lane_forward_length = declare_parameter<double>("drivable_lane_forward_length");
  p.drivable_lane_backward_length = declare_parameter<double>("drivable_lane_backward_length");
  p.drivable_lane_margin = declare_parameter<double>("drivable_lane_margin");
  p.drivable_area_margin = declare_parameter<double>("drivable_area_margin");
  p.refine_goal_search_radius_range = declare_parameter("refine_goal_search_radius_range", 7.5);
  p.turn_light_on_threshold_dis_lat = declare_parameter("turn_light_on_threshold_dis_lat", 0.3);
  p.turn_light_on_threshold_dis_long = declare_parameter("turn_light_on_threshold_dis_long", 10.0);
  p.turn_light_on_threshold_time = declare_parameter("turn_light_on_threshold_time", 3.0);
  p.visualize_drivable_area_for_shared_linestrings_lanelet =
    declare_parameter("visualize_drivable_area_for_shared_linestrings_lanelet", true);

  // vehicle info
  const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  p.vehicle_width = vehicle_info.vehicle_width_m;
  p.vehicle_length = vehicle_info.vehicle_length_m;
  p.wheel_tread = vehicle_info.wheel_tread_m;
  p.wheel_base = vehicle_info.wheel_base_m;
  p.front_overhang = vehicle_info.front_overhang_m;
  p.rear_overhang = vehicle_info.rear_overhang_m;
  p.left_over_hang = vehicle_info.left_overhang_m;
  p.right_over_hang = vehicle_info.right_overhang_m;
  p.base_link2front = vehicle_info.max_longitudinal_offset_m;
  p.base_link2rear = p.rear_overhang;

  return p;
}

SideShiftParameters BehaviorPathPlannerNode::getSideShiftParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
    std::string name = "side_shift." + str;
    return this->declare_parameter(name, def_val);
  };

  SideShiftParameters p{};
  p.min_distance_to_start_shifting = dp("min_distance_to_start_shifting", 5.0);
  p.time_to_start_shifting = dp("time_to_start_shifting", 1.0);
  p.shifting_lateral_jerk = dp("shifting_lateral_jerk", 0.2);
  p.min_shifting_distance = dp("min_shifting_distance", 5.0);
  p.min_shifting_speed = dp("min_shifting_speed", 5.56);
  p.shift_request_time_limit = dp("shift_request_time_limit", 1.0);

  return p;
}

AvoidanceParameters BehaviorPathPlannerNode::getAvoidanceParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
    std::string name = "avoidance." + str;
    return this->declare_parameter(name, def_val);
  };

  AvoidanceParameters p{};
  p.resample_interval_for_planning = dp("resample_interval_for_planning", 0.3);
  p.resample_interval_for_output = dp("resample_interval_for_output", 3.0);
  p.detection_area_right_expand_dist = dp("detection_area_right_expand_dist", 0.0);
  p.detection_area_left_expand_dist = dp("detection_area_left_expand_dist", 1.0);
  p.enable_avoidance_over_same_direction = dp("enable_avoidance_over_same_direction", true);
  p.enable_avoidance_over_opposite_direction = dp("enable_avoidance_over_opposite_direction", true);

  p.threshold_distance_object_is_on_center = dp("threshold_distance_object_is_on_center", 1.0);
  p.threshold_speed_object_is_stopped = dp("threshold_speed_object_is_stopped", 1.0);
  p.object_check_forward_distance = dp("object_check_forward_distance", 150.0);
  p.object_check_backward_distance = dp("object_check_backward_distance", 2.0);
  p.lateral_collision_margin = dp("lateral_collision_margin", 2.0);
  p.lateral_collision_safety_buffer = dp("lateral_collision_safety_buffer", 0.5);

  p.prepare_time = dp("prepare_time", 3.0);
  p.min_prepare_distance = dp("min_prepare_distance", 10.0);
  p.min_avoidance_distance = dp("min_avoidance_distance", 10.0);

  p.min_nominal_avoidance_speed = dp("min_nominal_avoidance_speed", 5.0);
  p.min_sharp_avoidance_speed = dp("min_sharp_avoidance_speed", 1.0);

  p.road_shoulder_safety_margin = dp("road_shoulder_safety_margin", 0.5);

  p.max_right_shift_length = dp("max_right_shift_length", 1.5);
  p.max_left_shift_length = dp("max_left_shift_length", 1.5);

  p.nominal_lateral_jerk = dp("nominal_lateral_jerk", 0.3);
  p.max_lateral_jerk = dp("max_lateral_jerk", 2.0);

  p.longitudinal_collision_margin_min_distance =
    dp("longitudinal_collision_margin_min_distance", 0.0);
  p.longitudinal_collision_margin_time = dp("longitudinal_collision_margin_time", 0.0);

  p.object_hold_max_count = dp("object_hold_max_count", 0);

  p.min_avoidance_speed_for_acc_prevention = dp("min_avoidance_speed_for_acc_prevention", 3.0);
  p.max_avoidance_acceleration = dp("max_avoidance_acceleration", 0.5);
  p.avoidance_search_distance = dp("avoidance_search_distance", 30.0);

  p.publish_debug_marker = dp("publish_debug_marker", false);
  p.print_debug_info = dp("print_debug_info", false);

  p.avoid_car = dp("target_object.car", true);
  p.avoid_truck = dp("target_object.truck", true);
  p.avoid_bus = dp("target_object.bus", true);
  p.avoid_trailer = dp("target_object.trailer", true);
  p.avoid_unknown = dp("target_object.unknown", false);
  p.avoid_bicycle = dp("target_object.bicycle", false);
  p.avoid_motorcycle = dp("target_object.motorcycle", false);
  p.avoid_pedestrian = dp("target_object.pedestrian", false);

  p.avoidance_execution_lateral_threshold = dp("avoidance_execution_lateral_threshold", 0.499);

  return p;
}

LaneFollowingParameters BehaviorPathPlannerNode::getLaneFollowingParam()
{
  LaneFollowingParameters p{};
  p.expand_drivable_area = declare_parameter("lane_following.expand_drivable_area", false);
  p.right_bound_offset = declare_parameter("lane_following.right_bound_offset", 0.5);
  p.left_bound_offset = declare_parameter("lane_following.left_bound_offset", 0.5);
  p.lane_change_prepare_duration =
    declare_parameter("lane_following.lane_change_prepare_duration", 2.0);
  return p;
}

LaneChangeParameters BehaviorPathPlannerNode::getLaneChangeParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
    std::string name = "lane_change." + str;
    return this->declare_parameter(name, def_val);
  };

  LaneChangeParameters p{};
  p.min_stop_distance = dp("min_stop_distance", 5.0);
  p.stop_time = dp("stop_time", 2.0);
  p.hysteresis_buffer_distance = dp("hysteresis_buffer_distance", 2.0);
  p.lane_change_prepare_duration = dp("lane_change_prepare_duration", 2.0);
  p.lane_changing_duration = dp("lane_changing_duration", 4.0);
  p.lane_change_finish_judge_buffer = dp("lane_change_finish_judge_buffer", 3.0);
  p.minimum_lane_change_velocity = dp("minimum_lane_change_velocity", 8.3);
  p.prediction_duration = dp("prediction_duration", 8.0);
  p.prediction_time_resolution = dp("prediction_time_resolution", 0.5);
  p.static_obstacle_velocity_thresh = dp("static_obstacle_velocity_thresh", 0.1);
  p.maximum_deceleration = dp("maximum_deceleration", 1.0);
  p.lane_change_sampling_num = dp("lane_change_sampling_num", 10);
  p.enable_abort_lane_change = dp("enable_abort_lane_change", true);
  p.enable_collision_check_at_prepare_phase = dp("enable_collision_check_at_prepare_phase", true);
  p.use_predicted_path_outside_lanelet = dp("use_predicted_path_outside_lanelet", true);
  p.use_all_predicted_path = dp("use_all_predicted_path", false);
  p.abort_lane_change_velocity_thresh = dp("abort_lane_change_velocity_thresh", 0.5);
  p.abort_lane_change_angle_thresh =
    dp("abort_lane_change_angle_thresh", tier4_autoware_utils::deg2rad(10.0));
  p.abort_lane_change_distance_thresh = dp("abort_lane_change_distance_thresh", 0.3);
  p.enable_blocked_by_obstacle = dp("enable_blocked_by_obstacle", false);
  p.lane_change_search_distance = dp("lane_change_search_distance", 30.0);

  // validation of parameters
  if (p.lane_change_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "lane_change_sampling_num must be positive integer. Given parameter: "
                      << p.lane_change_sampling_num << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "maximum_deceleration cannot be negative value. Given parameter: "
                      << p.maximum_deceleration << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  return p;
}

PullOverParameters BehaviorPathPlannerNode::getPullOverParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
    std::string name = "pull_over." + str;
    return this->declare_parameter(name, def_val);
  };

  PullOverParameters p;

  p.min_stop_distance = dp("min_stop_distance", 5.0);
  p.stop_time = dp("stop_time", 2.0);
  p.hysteresis_buffer_distance = dp("hysteresis_buffer_distance", 2.0);
  p.pull_over_prepare_duration = dp("pull_over_prepare_duration", 2.0);
  p.pull_over_duration = dp("pull_over_duration", 4.0);
  p.pull_over_finish_judge_buffer = dp("pull_over_finish_judge_buffer", 3.0);
  p.minimum_pull_over_velocity = dp("minimum_pull_over_velocity", 8.3);
  p.prediction_duration = dp("prediction_duration", 8.0);
  p.prediction_time_resolution = dp("prediction_time_resolution", 0.5);
  p.static_obstacle_velocity_thresh = dp("static_obstacle_velocity_thresh", 0.1);
  p.maximum_deceleration = dp("maximum_deceleration", 1.0);
  p.pull_over_sampling_num = dp("pull_over_sampling_num", 4);
  p.enable_collision_check_at_prepare_phase = dp("enable_collision_check_at_prepare_phase", true);
  p.use_predicted_path_outside_lanelet = dp("use_predicted_path_outside_lanelet", true);
  p.use_all_predicted_path = dp("use_all_predicted_path", false);
  p.enable_blocked_by_obstacle = dp("enable_blocked_by_obstacle", false);
  p.pull_over_search_distance = dp("pull_over_search_distance", 30.0);
  p.after_pull_over_straight_distance = dp("after_pull_over_straight_distance", 3.0);
  p.before_pull_over_straight_distance = dp("before_pull_over_straight_distance", 3.0);
  p.margin_from_boundary = dp("margin_from_boundary", 0.3);
  p.maximum_lateral_jerk = dp("maximum_lateral_jerk", 3.0);
  p.minimum_lateral_jerk = dp("minimum_lateral_jerk", 1.0);
  p.deceleration_interval = dp("deceleration_interval", 10.0);
  p.hazard_on_threshold_dis = dp("hazard_on_threshold_dis", 1.0);
  p.hazard_on_threshold_vel = dp("hazard_on_threshold_vel", 0.5);

  // validation of parameters
  if (p.pull_over_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "pull_over_sampling_num must be positive integer. Given parameter: "
                      << p.pull_over_sampling_num << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "maximum_deceleration cannot be negative value. Given parameter: "
                      << p.maximum_deceleration << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  return p;
}

PullOutParameters BehaviorPathPlannerNode::getPullOutParam()
{
  const auto dp = [this](const std::string & str, auto def_val) {
    std::string name = "pull_out." + str;
    return this->declare_parameter(name, def_val);
  };

  PullOutParameters p;

  p.min_stop_distance = dp("min_stop_distance", 5.0);
  p.stop_time = dp("stop_time", 2.0);
  p.hysteresis_buffer_distance = dp("hysteresis_buffer_distance", 2.0);
  p.pull_out_prepare_duration = dp("pull_out_prepare_duration", 2.0);
  p.pull_out_duration = dp("pull_out_duration", 4.0);
  p.pull_out_finish_judge_buffer = dp("pull_out_finish_judge_buffer", 1.0);
  p.minimum_pull_out_velocity = dp("minimum_pull_out_velocity", 8.3);
  p.prediction_duration = dp("prediction_duration", 8.0);
  p.prediction_time_resolution = dp("prediction_time_resolution", 0.5);
  p.static_obstacle_velocity_thresh = dp("static_obstacle_velocity_thresh", 0.1);
  p.maximum_deceleration = dp("maximum_deceleration", 1.0);
  p.pull_out_sampling_num = dp("pull_out_sampling_num", 4);
  p.enable_collision_check_at_prepare_phase = dp("enable_collision_check_at_prepare_phase", true);
  p.use_predicted_path_outside_lanelet = dp("use_predicted_path_outside_lanelet", true);
  p.use_all_predicted_path = dp("use_all_predicted_path", false);
  p.use_dynamic_object = dp("use_dynamic_object", false);
  p.enable_blocked_by_obstacle = dp("enable_blocked_by_obstacle", false);
  p.pull_out_search_distance = dp("pull_out_search_distance", 30.0);
  p.after_pull_out_straight_distance = dp("after_pull_out_straight_distance", 3.0);
  p.before_pull_out_straight_distance = dp("before_pull_out_straight_distance", 3.0);
  p.maximum_lateral_jerk = dp("maximum_lateral_jerk", 3.0);
  p.minimum_lateral_jerk = dp("minimum_lateral_jerk", 1.0);
  p.deceleration_interval = dp("deceleration_interval", 10.0);

  // validation of parameters
  if (p.pull_out_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "pull_out_sampling_num must be positive integer. Given parameter: "
                      << p.pull_out_sampling_num << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "maximum_deceleration cannot be negative value. Given parameter: "
                      << p.maximum_deceleration << std::endl
                      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  return p;
}

BehaviorTreeManagerParam BehaviorPathPlannerNode::getBehaviorTreeManagerParam()
{
  BehaviorTreeManagerParam p{};
  p.bt_tree_config_path = declare_parameter("bt_tree_config_path", "default");
  p.groot_zmq_publisher_port = declare_parameter("groot_zmq_publisher_port", 1666);
  p.groot_zmq_server_port = declare_parameter("groot_zmq_server_port", 1667);
  return p;
}

void BehaviorPathPlannerNode::waitForData()
{
  // wait until mandatory data is ready
  while (!current_scenario_ && rclcpp::ok()) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for scenario topic");
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::Rate(100).sleep();
  }

  mutex_pd_.lock();  // for planner_data_
  while (!planner_data_->route_handler->isHandlerReady() && rclcpp::ok()) {
    mutex_pd_.unlock();
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for route to be ready");
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::Rate(100).sleep();
    mutex_pd_.lock();
  }

  while (rclcpp::ok()) {
    if (planner_data_->dynamic_object && planner_data_->self_odometry) {
      break;
    }

    mutex_pd_.unlock();
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "waiting for vehicle pose, vehicle_velocity, and obstacles");
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::Rate(100).sleep();
    mutex_pd_.lock();
  }

  self_pose_listener_.waitForFirstPose();
  planner_data_->self_pose = self_pose_listener_.getCurrentPose();
  mutex_pd_.unlock();
}

void BehaviorPathPlannerNode::run()
{
  RCLCPP_DEBUG(get_logger(), "----- BehaviorPathPlannerNode start -----");
  mutex_bt_.lock();  // for bt_manager_
  mutex_pd_.lock();  // for planner_data_

  // behavior_path_planner runs only in LANE DRIVING scenario.
  if (current_scenario_->current_scenario != Scenario::LANEDRIVING) {
    return;
  }

  // update planner data
  planner_data_->self_pose = self_pose_listener_.getCurrentPose();

  // NOTE: planner_data must not be referenced for multithreading
  const auto planner_data = planner_data_;
  mutex_pd_.unlock();

  // run behavior planner
  const auto output = bt_manager_->run(planner_data);

  // path handling
  const auto path = getPath(output, planner_data);
  const auto path_candidate = getPathCandidate(output, planner_data);

  // update planner data
  mutex_pd_.lock();  // for planner_data_
  planner_data_->prev_output_path = path;
  mutex_pd_.unlock();

  auto clipped_path = modifyPathForSmoothGoalConnection(*path);
  clipPathLength(clipped_path);

  if (!clipped_path.points.empty()) {
    path_publisher_->publish(clipped_path);
  } else {
    RCLCPP_ERROR(get_logger(), "behavior path output is empty! Stop publish.");
  }
  path_candidate_publisher_->publish(util::toPath(*path_candidate));

  // debug_path_publisher_->publish(util::toPath(path));
  debug_drivable_area_publisher_->publish(path->drivable_area);

  // for turn signal
  {
    TurnIndicatorsCommand turn_signal;
    HazardLightsCommand hazard_signal;
    if (output.turn_signal_info.hazard_signal.command == HazardLightsCommand::ENABLE) {
      turn_signal.command = TurnIndicatorsCommand::DISABLE;
      hazard_signal.command = output.turn_signal_info.hazard_signal.command;
    } else {
      turn_signal = turn_signal_decider_.getTurnSignal(
        *path, planner_data->self_pose->pose, *(planner_data->route_handler),
        output.turn_signal_info.turn_signal, output.turn_signal_info.signal_distance);
      hazard_signal.command = HazardLightsCommand::DISABLE;
    }
    turn_signal.stamp = get_clock()->now();
    hazard_signal.stamp = get_clock()->now();
    turn_signal_publisher_->publish(turn_signal);
    hazard_signal_publisher_->publish(hazard_signal);
  }

  // for remote operation
  publishModuleStatus(bt_manager_->getModulesStatus(), planner_data);
  debug_avoidance_msg_array_publisher_->publish(bt_manager_->getAvoidanceDebugMsgArray());

  publishDebugMarker(bt_manager_->getDebugMarkers());

  if (planner_data->parameters.visualize_drivable_area_for_shared_linestrings_lanelet) {
    const auto drivable_area_lines = marker_utils::createFurthestLineStringMarkerArray(
      util::getDrivableAreaForAllSharedLinestringLanelets(planner_data));
    debug_drivable_area_lanelets_publisher_->publish(drivable_area_lines);
  }

  mutex_bt_.unlock();
  RCLCPP_DEBUG(get_logger(), "----- behavior path planner end -----\n\n");
}

PathWithLaneId::SharedPtr BehaviorPathPlannerNode::getPath(
  const BehaviorModuleOutput & bt_output, const std::shared_ptr<PlannerData> planner_data)
{
  // TODO(Horibe) do some error handling when path is not available.

  auto path = bt_output.path ? bt_output.path : planner_data->prev_output_path;
  path->header = planner_data->route_handler->getRouteHeader();
  path->header.stamp = this->now();
  RCLCPP_DEBUG(
    get_logger(), "BehaviorTreeManager: output is %s.", bt_output.path ? "FOUND" : "NOT FOUND");
  return path;
}

PathWithLaneId::SharedPtr BehaviorPathPlannerNode::getPathCandidate(
  const BehaviorModuleOutput & bt_output, const std::shared_ptr<PlannerData> planner_data)
{
  auto path_candidate =
    bt_output.path_candidate ? bt_output.path_candidate : std::make_shared<PathWithLaneId>();
  path_candidate->header = planner_data->route_handler->getRouteHeader();
  path_candidate->header.stamp = this->now();
  RCLCPP_DEBUG(
    get_logger(), "BehaviorTreeManager: path candidate is %s.",
    bt_output.path_candidate ? "FOUND" : "NOT FOUND");
  return path_candidate;
}

void BehaviorPathPlannerNode::publishModuleStatus(
  const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses,
  const std::shared_ptr<PlannerData> planner_data)
{
  auto getModuleType = [](std::string name) {
    if (name == "LaneChange") {
      return PathChangeModuleId::LANE_CHANGE;
    } else if (name == "Avoidance") {
      return PathChangeModuleId::AVOIDANCE;
    } else if (name == "ForceLaneChange") {
      return PathChangeModuleId::FORCE_LANE_CHANGE;
    } else if (name == "PullOver") {
      return PathChangeModuleId::PULL_OVER;
    } else if (name == "PullOut") {
      return PathChangeModuleId::PULL_OUT;
    } else {
      return PathChangeModuleId::NONE;
    }
  };

  const auto now = this->now();

  PathChangeModule ready_module{};
  PathChangeModuleArray running_modules{};
  PathChangeModuleArray force_available{};

  bool is_ready{false};
  for (auto & status : statuses) {
    if (status->status == BT::NodeStatus::RUNNING) {
      PathChangeModuleId module{};
      module.type = getModuleType(status->module_name);
      running_modules.modules.push_back(module);
    }
    if (status->module_name == "LaneChange") {
      const auto force_approval = planner_data->approval.is_force_approved;
      if (
        force_approval.module_name == "ForceLaneChange" &&
        (now - force_approval.stamp).seconds() < 0.5) {
        is_ready = true;
        ready_module.module.type = getModuleType("ForceLaneChange");
      }
      if (status->is_requested && !status->is_ready) {
        PathChangeModuleId module;
        module.type = getModuleType("ForceLaneChange");
        force_available.modules.push_back(module);
        break;
      }
    }
    if (status->is_ready && status->is_waiting_approval) {
      if (status->module_name == "LaneFollowing" || status->module_name == "SideShift") {
        continue;
      }
      is_ready = true;
      RCLCPP_DEBUG(
        get_logger(), "%s is Ready : ready = %s, is_approved = %s", status->module_name.c_str(),
        status->is_ready ? "true" : "false", status->is_waiting_approval ? "true" : "false");
      ready_module.module.type = getModuleType(status->module_name);
    }
  }

  if (!is_ready) {
    prev_ready_module_name_ = "NONE";
    ready_module.module.type = PathChangeModuleId::NONE;
  }

  ready_module.header.stamp = now;
  plan_ready_publisher_->publish(ready_module);

  running_modules.header.stamp = now;
  plan_running_publisher_->publish(running_modules);

  force_available.header.stamp = now;
  force_available_publisher_->publish(force_available);
}

void BehaviorPathPlannerNode::publishDebugMarker(const std::vector<MarkerArray> & debug_markers)
{
  MarkerArray msg{};
  for (const auto & markers : debug_markers) {
    tier4_autoware_utils::appendMarkerArray(markers, &msg);
  }
  debug_marker_publisher_->publish(msg);
}

void BehaviorPathPlannerNode::onVelocity(const Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->self_odometry = msg;
}
void BehaviorPathPlannerNode::onPerception(const PredictedObjects::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->dynamic_object = msg;
}
void BehaviorPathPlannerNode::onExternalApproval(const ApprovalMsg::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->approval.is_approved.data = msg->approval;
  // TODO(wep21): Replace msg stamp after {stamp: now} is implemented in ros2 topic pub
  planner_data_->approval.is_approved.stamp = this->now();
}
void BehaviorPathPlannerNode::onForceApproval(const PathChangeModule::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);
  auto getModuleName = [](PathChangeModuleId module) {
    if (module.type == PathChangeModuleId::FORCE_LANE_CHANGE) {
      return "ForceLaneChange";
    } else {
      return "NONE";
    }
  };
  planner_data_->approval.is_force_approved.module_name = getModuleName(msg->module);
  planner_data_->approval.is_force_approved.stamp = msg->header.stamp;
}
void BehaviorPathPlannerNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->route_handler->setMap(*msg);
}
void BehaviorPathPlannerNode::onRoute(const HADMapRoute::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);
  const bool is_first_time = !(planner_data_->route_handler->isHandlerReady());

  planner_data_->route_handler->setRoute(*msg);

  // Reset behavior tree when new route is received,
  // so that the each modules do not have to care about the "route jump".
  if (!is_first_time) {
    RCLCPP_DEBUG(get_logger(), "new route is received. reset behavior tree.");
    bt_manager_->resetBehaviorTree();
  }
}

void BehaviorPathPlannerNode::clipPathLength(PathWithLaneId & path) const
{
  const auto ego_pose = planner_data_->self_pose->pose;
  const double forward = planner_data_->parameters.forward_path_length;
  const double backward = planner_data_->parameters.backward_path_length;

  util::clipPathLength(path, ego_pose, forward, backward);
}

PathWithLaneId BehaviorPathPlannerNode::modifyPathForSmoothGoalConnection(
  const PathWithLaneId & path) const
{
  const auto goal = planner_data_->route_handler->getGoalPose();
  const auto is_approved = planner_data_->approval.is_approved.data;
  auto goal_lane_id = planner_data_->route_handler->getGoalLaneId();

  Pose refined_goal{};
  {
    lanelet::ConstLanelet goal_lanelet;
    lanelet::ConstLanelet pull_over_lane;
    geometry_msgs::msg::Pose pull_over_goal;
    if (
      is_approved && planner_data_->route_handler->getPullOverTarget(
                       planner_data_->route_handler->getShoulderLanelets(), &pull_over_lane)) {
      refined_goal = planner_data_->route_handler->getPullOverGoalPose();
      goal_lane_id = pull_over_lane.id();
    } else if (planner_data_->route_handler->getGoalLanelet(&goal_lanelet)) {
      refined_goal = util::refineGoal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }

  auto refined_path = util::refinePathForGoal(
    planner_data_->parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal,
    goal_lane_id);
  refined_path.header.frame_id = "map";
  refined_path.header.stamp = this->now();

  return refined_path;
}
}  // namespace behavior_path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_path_planner::BehaviorPathPlannerNode)
