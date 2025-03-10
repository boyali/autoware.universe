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

#include "behavior_path_planner/scene_module/avoidance/debug.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace marker_utils
{
using behavior_path_planner::util::calcPathArcLengthArray;
using behavior_path_planner::util::shiftPose;
using visualization_msgs::msg::Marker;

inline int64_t bitShift(int64_t original_id) { return original_id << (sizeof(int32_t) * 8 / 2); }

MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> shift_distance,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & reference, const std::string & ns,
  const double r, const double g, const double b)
{
  MarkerArray ma;

  if (shift_distance.size() != reference.points.size()) {
    return ma;
  }

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.action = Marker::ADD;
  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
  marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.9);
  marker.type = Marker::LINE_STRIP;

  for (size_t i = 0; i < shift_distance.size(); ++i) {
    auto p = reference.points.at(i).point.pose;
    shiftPose(&p, shift_distance.at(i));
    marker.points.push_back(p.position);
  }

  ma.markers.push_back(marker);
  return ma;
}

MarkerArray createAvoidPointMarkerArray(
  const AvoidPointArray & shift_points, const std::string & ns, const double r, const double g,
  const double b, const double w)
{
  int32_t id = 0;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  AvoidPointArray shift_points_local = shift_points;
  if (shift_points_local.empty()) {
    shift_points_local.push_back(AvoidPoint());
  }

  for (const auto & sp : shift_points_local) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.action = Marker::ADD;
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = tier4_autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.9);
    {
      marker.type = Marker::CUBE;

      // start point
      auto marker_s = marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      // shiftPose(&marker_s.pose, current_shift);  // old
      shiftPose(&marker_s.pose, sp.start_length);
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = marker;
      marker_e.id = id++;
      marker_e.pose = sp.end;
      shiftPose(&marker_e.pose, sp.length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = marker;
      marker_l.id = id++;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = tier4_autoware_utils::createMarkerScale(w, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    // current_shift = sp.length;
  }

  return msg;
}

MarkerArray createShiftPointMarkerArray(
  const ShiftPointArray & shift_points, const double base_shift, const std::string & ns,
  const double r, const double g, const double b, const double w)
{
  int32_t id = 0;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  ShiftPointArray shift_points_local = shift_points;
  if (shift_points_local.empty()) {
    shift_points_local.push_back(ShiftPoint());
  }

  // TODO(Horibe) now assuming the shift point is aligned in longitudinal distance order
  double current_shift = base_shift;
  for (const auto & sp : shift_points_local) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.action = Marker::ADD;
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = tier4_autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.5);
    {
      marker.type = Marker::CUBE;

      // start point
      auto marker_s = marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      shiftPose(&marker_s.pose, current_shift);  // old
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = marker;
      marker_e.id = id++;
      marker_e.pose = sp.end;
      shiftPose(&marker_e.pose, sp.length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = marker;
      marker_l.id = id++;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = tier4_autoware_utils::createMarkerScale(w, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    current_shift = sp.length;
  }

  return msg;
}

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & lanelet : lanelets) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = lanelet.id();
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.999);
    for (const auto & p : lanelet.polygon3d()) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  int32_t i = 0;
  int32_t uid = bitShift(lane_id);
  for (const auto & polygon : polygons) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
    for (const auto & p : polygon) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;

  marker.ns = ns;
  marker.id = lane_id;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = tier4_autoware_utils::createMarkerScale(0.3, 0.0, 0.0);
  marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.8);
  for (const auto & p : polygon.points) {
    Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray createObjectsMarkerArray(
  const PredictedObjects & objects, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  for (const auto & object : objects.objects) {
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(3.0, 1.0, 1.0);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.8);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createAvoidanceObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, const std::string & ns)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  const auto normal_color = tier4_autoware_utils::createMarkerColor(0.9, 0.0, 0.0, 0.8);
  const auto disappearing_color = tier4_autoware_utils::createMarkerColor(0.9, 0.5, 0.9, 0.6);

  int32_t i = 0;
  for (const auto & object : objects) {
    marker.id = i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(3.0, 1.5, 1.5);
    marker.color = object.lost_count == 0 ? normal_color : disappearing_color;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b)
{
  const auto arclength = calcPathArcLengthArray(path);
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;
  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  int32_t idx = 0;
  for (const auto & p : path.points) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose = p.point.pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(0.2, 0.1, 0.3);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.999);
    msg.markers.push_back(marker);
    if (idx % 10 == 0) {
      auto marker_text = marker;
      marker_text.id = uid + i++;
      marker_text.type = Marker::TEXT_VIEW_FACING;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << "i=" << idx << "\ns=" << arclength.at(idx);
      marker_text.text = ss.str();
      marker_text.color = tier4_autoware_utils::createMarkerColor(1, 1, 1, 0.999);
      msg.markers.push_back(marker_text);
    }
    ++idx;
  }

  return msg;
}

MarkerArray createPoseLineMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const double r, const double g,
  const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker_line{};
  marker_line.header.frame_id = "map";
  marker_line.header.stamp = current_time;
  marker_line.ns = ns + "_line";
  marker_line.id = id;
  marker_line.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_line.type = Marker::LINE_STRIP;
  marker_line.action = Marker::ADD;
  marker_line.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker_line.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
  marker_line.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.999);

  const double yaw = tf2::getYaw(pose.orientation);

  const double a = 3.0;
  Point p0;
  p0.x = pose.position.x - a * std::sin(yaw);
  p0.y = pose.position.y + a * std::cos(yaw);
  p0.z = pose.position.z;
  marker_line.points.push_back(p0);

  Point p1;
  p1.x = pose.position.x + a * std::sin(yaw);
  p1.y = pose.position.y - a * std::cos(yaw);
  p1.z = pose.position.z;
  marker_line.points.push_back(p1);

  msg.markers.push_back(marker_line);

  return msg;
}

MarkerArray createPoseMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const double r, const double g,
  const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.id = id;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::ARROW;
  marker.action = Marker::ADD;
  marker.pose = pose;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.7, 0.3, 0.3);
  marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.999);
  msg.markers.push_back(marker);

  return msg;
}
MarkerArray makeOverhangToRoadShoulderMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = "overhang";

  const auto normal_color = tier4_autoware_utils::createMarkerColor(1.0, 1.0, 0.0, 1.0);

  int32_t i = 0;
  for (const auto & object : objects) {
    marker.id = i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::TEXT_VIEW_FACING;
    // marker.action = Marker::ADD;
    marker.pose = object.overhang_pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(1.0, 1.0, 1.0);
    marker.color = normal_color;
    std::ostringstream string_stream;
    string_stream << "(to_road_shoulder_distance = " << object.to_road_shoulder_distance << " [m])";
    marker.text = string_stream.str();
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createOverhangFurthestLineStringMarkerArray(
  const lanelet::ConstLineStrings3d & linestrings, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & linestring : linestrings) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = linestring.id();
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = tier4_autoware_utils::createMarkerScale(0.4, 0.0, 0.0);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.999);
    for (const auto & p : linestring.basicLineString()) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
    marker.ns = "linestring id";
    marker.type = Marker::TEXT_VIEW_FACING;
    Pose text_id_pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(1.5, 1.5, 1.5);
    marker.color = tier4_autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.8);
    text_id_pose.position.x = linestring.front().x();
    text_id_pose.position.y = linestring.front().y();
    text_id_pose.position.z = linestring.front().z();
    marker.pose = text_id_pose;
    std::ostringstream ss;
    ss << "(ID : " << linestring.id() << ") ";
    marker.text = ss.str();
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createFurthestLineStringMarkerArray(const lanelet::ConstLineStrings3d & linestrings)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  MarkerArray msg;
  if (linestrings.empty()) {
    return msg;
  }

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = "shared_linestring_lanelets";
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = tier4_autoware_utils::createMarkerScale(0.3, 0.0, 0.0);
  marker.color = tier4_autoware_utils::createMarkerColor(0.996, 0.658, 0.466, 0.999);

  const auto reserve_size = linestrings.size() / 2;
  lanelet::ConstLineStrings3d lefts;
  lanelet::ConstLineStrings3d rights;
  lefts.reserve(reserve_size);
  rights.reserve(reserve_size);
  for (size_t idx = 1; idx < linestrings.size(); idx += 2) {
    rights.emplace_back(linestrings.at(idx - 1));
    lefts.emplace_back(linestrings.at(idx));
  }

  const auto & first_ls = lefts.front().basicLineString();
  for (const auto & ls : first_ls) {
    Point p;
    p.x = ls.x();
    p.y = ls.y();
    p.z = ls.z();
    marker.points.push_back(p);
  }

  for (auto idx = lefts.cbegin() + 1; idx != lefts.cend(); ++idx) {
    const auto & marker_back = marker.points.back();
    Point front;
    front.x = idx->basicLineString().front().x();
    front.y = idx->basicLineString().front().y();
    front.z = idx->basicLineString().front().z();
    Point front_inverted;
    front_inverted.x = idx->invert().basicLineString().front().x();
    front_inverted.y = idx->invert().basicLineString().front().y();
    front_inverted.z = idx->invert().basicLineString().front().z();
    const bool isFrontNear = tier4_autoware_utils::calcDistance2d(marker_back, front) <
                             tier4_autoware_utils::calcDistance2d(marker_back, front_inverted);
    const auto & left_ls = (isFrontNear) ? idx->basicLineString() : idx->invert().basicLineString();
    for (auto ls = left_ls.cbegin(); ls != left_ls.cend(); ++ls) {
      Point p;
      p.x = ls->x();
      p.y = ls->y();
      p.z = ls->z();
      marker.points.push_back(p);
    }
  }

  for (auto idx = rights.crbegin(); idx != rights.crend(); ++idx) {
    const auto & marker_back = marker.points.back();
    Point front;
    front.x = idx->basicLineString().front().x();
    front.y = idx->basicLineString().front().y();
    front.z = idx->basicLineString().front().z();
    Point front_inverted;
    front_inverted.x = idx->invert().basicLineString().front().x();
    front_inverted.y = idx->invert().basicLineString().front().y();
    front_inverted.z = idx->invert().basicLineString().front().z();
    const bool isFrontFurther = tier4_autoware_utils::calcDistance2d(marker_back, front) >
                                tier4_autoware_utils::calcDistance2d(marker_back, front_inverted);
    const auto & right_ls =
      (isFrontFurther) ? idx->basicLineString() : idx->invert().basicLineString();
    for (auto ls = right_ls.crbegin(); ls != right_ls.crend(); ++ls) {
      Point p;
      p.x = ls->x();
      p.y = ls->y();
      p.z = ls->z();
      marker.points.push_back(p);
    }
  }

  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  msg.markers.push_back(marker);
  return msg;
}
}  // namespace marker_utils

std::string toStrInfo(const behavior_path_planner::ShiftPointArray & sp_arr)
{
  if (sp_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & sp : sp_arr) {
    ss << std::endl << toStrInfo(sp);
  }
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::ShiftPoint & sp)
{
  const auto & ps = sp.start.position;
  const auto & pe = sp.end.position;
  std::stringstream ss;
  ss << "shift length: " << sp.length << ", start_idx: " << sp.start_idx
     << ", end_idx: " << sp.end_idx << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x
     << ", " << pe.y << ")";
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::AvoidPointArray & ap_arr)
{
  if (ap_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & ap : ap_arr) {
    ss << std::endl << toStrInfo(ap);
  }
  return ss.str();
}
std::string toStrInfo(const behavior_path_planner::AvoidPoint & ap)
{
  std::stringstream pids;
  for (const auto pid : ap.parent_ids) {
    pids << pid << ", ";
  }
  const auto & ps = ap.start.position;
  const auto & pe = ap.end.position;
  std::stringstream ss;
  ss << "id = " << ap.id << ", shift length: " << ap.length << ", start_idx: " << ap.start_idx
     << ", end_idx: " << ap.end_idx << ", start_dist = " << ap.start_longitudinal
     << ", end_dist = " << ap.end_longitudinal << ", start_length: " << ap.start_length
     << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x << ", " << pe.y
     << "), relative_length: " << ap.getRelativeLength() << ", grad = " << ap.getGradient()
     << ", parent_ids = [" << pids.str() << "]";
  return ss.str();
}
