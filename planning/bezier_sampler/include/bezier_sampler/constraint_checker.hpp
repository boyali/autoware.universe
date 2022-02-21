/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <bezier_sampler/bezier.hpp>
#include <vector>

BOOST_GEOMETRY_REGISTER_POINT_2D(Eigen::Vector2d, double, cs::cartesian, x(), y())
namespace motion_planning::bezier_sampler
{
namespace bg = boost::geometry;
using polygon_t = bg::model::polygon<Eigen::Vector2d>;

struct ConstraintParameters
{
  double maximum_curvature;   // [m⁻¹]
  double ego_width;           // [m] total width of the vehicle
  double ego_rear_length;     // [m] distance from baselink to the rear bumper
  double ego_front_length;    // [m] distance from baselink to the front bumber
  double hard_safety_margin;  // [m] distance from obstacles below which a path becomes invalid
  int nb_points;  // number of points to sample along Bezier curves for constraint checks
};
//@brief Constraint checker class
class ConstraintChecker
{
  // Drivable area occupancy grid
  nav_msgs::msg::OccupancyGrid _drivable_area;
  ConstraintParameters _params;
  Eigen::Matrix2d _drivable_area_rotation;
  // Corners of the footprint relative to baselink
  Eigen::Vector2d _left_rear;
  Eigen::Vector2d _right_rear;
  Eigen::Vector2d _left_front;
  Eigen::Vector2d _right_front;

public:
  ConstraintChecker() = delete;
  ConstraintChecker(const nav_msgs::msg::OccupancyGrid & drivable_area, ConstraintParameters parameters);
  //@brief build a polygon representing the footprint of the ego vehicle along the path
  [[nodiscard]] polygon_t buildFootprintPolygon(const Bezier & path) const;
  //@brief return true if the given path is drivable (w.r.t maximum curvature)
  [[nodiscard]] bool isDrivable(const Bezier & path) const;
  //@brief return true if the given path is collision free
  [[nodiscard]] bool isCollisionFree(const Bezier & path) const;
  //@brief return true if the given map position is collision free
  [[nodiscard]] int8_t isCollisionFree(const Eigen::Vector2d & position) const;
};
}  // namespace bezier_sampler::motion_planning
