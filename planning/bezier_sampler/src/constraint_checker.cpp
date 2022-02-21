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

#include <bezier_sampler/constraint_checker.hpp>

namespace motion_planning
{
namespace bezier_sampler
{
ConstraintChecker::ConstraintChecker(
  const nav_msgs::msg::OccupancyGrid & drivable_area, ConstraintParameters parameters)
: _drivable_area(drivable_area), _params(parameters)
{
  // Prepare rotation map -> OccupancyGrid cell
  tf2::Quaternion drivable_area_quaternion;
  tf2::convert(_drivable_area.info.origin.orientation, drivable_area_quaternion);
  tf2::Matrix3x3 m(drivable_area_quaternion);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  _drivable_area_rotation << std::cos(-yaw), -std::sin(-yaw),
                             std::sin(-yaw),  std::cos(-yaw);
  //TODO
  // build polygon from the OccupancyGrid
  // Convert to opencv image
  // Extract contour
  // Rotate and translate

  // Prepare vectors to the footprint corners;
  _left_front = {parameters.ego_front_length, parameters.ego_width / 2.0};
  _right_front = {parameters.ego_front_length, -parameters.ego_width / 2.0};
  _left_rear = {-parameters.ego_rear_length, parameters.ego_width / 2.0};
  _right_rear = {-parameters.ego_rear_length, -parameters.ego_width / 2.0};
}
polygon_t ConstraintChecker::buildFootprintPolygon(const Bezier & path) const
{
  // Using the method from Section IV.A of A. Artuñedoet al.: Real-Time Motion Planning Approach for Automated Driving in Urban Environments
  polygon_t footprint;
  // sample points
  // we store the right bound as it needs to be added to the polygon after the left bound
  std::vector<Eigen::Vector2d> right_bound;
  const std::vector<Eigen::Vector3d> points = path.cartesianWithHeading(50);
  // first point: use the left and right point on the rear
  {
    const Eigen::Vector2d first_point = points.front().head<2>();
    double heading = points.front().z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading),
                std::sin(heading), std::cos(heading);
    const Eigen::Vector2d left_rear_point = first_point + rotation * _left_rear;
    bg::append(footprint, left_rear_point);
    right_bound.push_back(first_point + rotation * _right_rear);
  }
  // For each points (except 1st and last)
  for(auto it = std::next(points.begin()); it != std::prev(points.end()); ++it)
  {
    const Eigen::Vector2d point = it->head<2>();
    const double prev_heading = std::prev(it)->z();
    const double heading = it->z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading),
                std::sin(heading), std::cos(heading);
    // We use the change in the heading (restricted in [-pi;pi]) to determine if the path is turning left or right
    const bool turning_right = (std::fmod(heading - prev_heading + M_PI, 2*M_PI) - M_PI) < 0;
    if(turning_right)
    {
      const Eigen::Vector2d left_front_point = point + rotation * _left_front;
      bg::append(footprint, left_front_point);
      right_bound.push_back(point + rotation * _right_rear);
    }
    else {
      const Eigen::Vector2d left_rear_point = point + rotation * _left_rear;
      bg::append(footprint, left_rear_point);
      right_bound.push_back(point + rotation * _right_front);
    }
  }
  // last point: use the left and right point on the front
  {
    Eigen::Vector2d last_point = points.back().head<2>();
    double heading = points.back().z();
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading),
                std::sin(heading), std::cos(heading);
    Eigen::Vector2d left_front_point = last_point + rotation * _left_front;
    bg::append(footprint, left_front_point);
    right_bound.push_back(last_point + rotation * _right_front);
  }
  for(auto it = right_bound.rbegin(); it != right_bound.rend(); ++it)
    bg::append(footprint, *it);
  bg::correct(footprint);
  return footprint;
}
bool ConstraintChecker::isDrivable(const Bezier & path) const {
  const double step = 1.0/(_params.nb_points-1);
  for(double t = 0.0; t <= 1.0; t+=step) {
    const double curvature = std::abs(path.curvature(t));
    if(curvature >= _params.maximum_curvature)
      return false;
  }
  return true;
}
bool ConstraintChecker::isCollisionFree(const Bezier & path) const {
  for(const Eigen::Vector2d & position: buildFootprintPolygon(path).outer()) {
    if(not isCollisionFree(position))
    {
      // std::printf("Collision @ (%2.2f, %2.2f)\n", position.x(), position.y());
      return false;
    }
  }
  return true;
}
int8_t ConstraintChecker::isCollisionFree(const Eigen::Vector2d & position) const {
  const Eigen::Vector2d local_pose(position.x() - _drivable_area.info.origin.position.x, position.y() - _drivable_area.info.origin.position.y);
  // const Eigen::Vector2d local_pose_rot = _drivable_area_rotation * local_pose;
  const Eigen::Vector2i index = (_drivable_area_rotation * local_pose / _drivable_area.info.resolution).cast<int>();
  const int flat_index = index.x() + index.y() * _drivable_area.info.width;
  // std::printf("\tPosition (%2.2f,%2.2f)\n", position.x(), position.y());
  // std::printf("\tIndex (%d,%d)\n", index.x(), index.y());
  // std::printf("\tFlat index (%d)\n", flat_index);
  // std::printf("\tArea info (%d x %d)\n", _drivable_area.info.width, _drivable_area.info.height);
  // std::cout << std::endl;
  // Return true if the position is outside of the grid OR if the corresponding cell value is 0
  return index.x() < 0 || index.y() < 0 || index.x() >= static_cast<int>(_drivable_area.info.width) ||
         index.y() >= static_cast<int>(_drivable_area.info.height) || static_cast<int>(_drivable_area.data[flat_index]) == 0;
}
}  // namespace bezier_sampler
} // namespace motion_planning
