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

#include <bezier_sampler/bezier.hpp>

namespace motion_planning
{
namespace bezier_sampler
{
  const Eigen::Matrix<double, 6, 2> Bezier::getControlPoints() const {
    return control_points;
  }

  Bezier::Bezier(const Eigen::Matrix<double, 6, 2> & _control_points): control_points(_control_points)
  {
  }
  Bezier::Bezier(const std::vector<Eigen::Vector2d> & _control_points)
  {
    if(_control_points.size() != 6)
    {
      //TODO exception
      std::cerr << "Trying to initialize a quintic bezier curve with " << _control_points.size()
                << " control points." << std::endl;
    }
    control_points << _control_points[0], _control_points[1], _control_points[2],
      _control_points[3], _control_points[4], _control_points[5];
  }

  Eigen::Vector2d Bezier::value(double t) const{
      Eigen::Vector2d point = {0.0, 0.0};
      // sum( binomial(i in 5) * (1 - t)^(5-i) * t^i * control_point[i] )
      point += std::pow((1 - t), 5) * control_points.row(0);
      point += 5 * std::pow((1 - t), 4) * t * control_points.row(1);
      point += 10 * std::pow((1 - t), 3) * t*t * control_points.row(2);
      point += 10 * std::pow((1 - t), 2) * t*t*t * control_points.row(3);
      point += 5 * (1 - t) * t*t*t*t * control_points.row(4); 
      point += t*t*t*t*t * control_points.row(5);
      return point;
  }

  Eigen::Vector2d Bezier::valueM(double t) const{
    Eigen::Matrix<double, 1, 6> ts;
    ts << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
    return ts * quintic_bezier_coefficients * control_points;
  }

  // TODO ensure points are separated by fixed arc-length (rather than fixed t-parameter)
  std::vector<Eigen::Vector2d> Bezier::cartesian(int nb_points) const{
    std::vector<Eigen::Vector2d> points;
    points.reserve(nb_points);
    double step = 1.0/(nb_points-1);
    for (double t = 0.0; t <= 1.0; t += step)
      points.push_back(valueM(t));
    return points;
  }

  // TODO ensure points are separated by fixed arc-length (rather than fixed t-parameter)
  std::vector<Eigen::Vector2d> Bezier::cartesian(double resolution) const{
    std::vector<Eigen::Vector2d> points;
    points.reserve((int)(1/resolution));
    for (double t = 0.0; t <= 1.0; t += resolution)
      points.push_back(valueM(t));
    return points;
  }

  // TODO ensure points are separated by fixed arc-length (rather than fixed t-parameter)
  std::vector<Eigen::Vector3d> Bezier::cartesianWithHeading(int nb_points) const{
    std::vector<Eigen::Vector3d> points;
    points.reserve(nb_points);
    double step = 1.0/(nb_points-1);
    for (double t = 0.0; t <= 1.0; t += step) {
      Eigen::Vector2d point = valueM(t);
      points.emplace_back(point.x(), point.y(), heading(t));
    }
    return points;
  }

  Eigen::Vector2d Bezier::velocity(double t) const{
    Eigen::Matrix<double, 1, 5> ts;
    ts << 1, t, t*t, t*t*t, t*t*t*t;
    return ts * quintic_bezier_velocity_coefficients * control_points;
  }

  Eigen::Vector2d Bezier::acceleration(double t) const{
    Eigen::Matrix<double, 1, 4> ts;
    ts << 1, t, t*t, t*t*t;
    return ts * quintic_bezier_acceleration_coefficients * control_points;
  }

  double Bezier::curvature(double t) const {
    Eigen::Vector2d vel = velocity(t);
    Eigen::Vector2d accel = acceleration(t);
    double denominator = std::pow(vel.x() * vel.x() + vel.y() * vel.y(), 3.0/2.0);
    if(denominator)
      return (vel.x() * accel.y() - accel.x() * vel.y()) / denominator;
    else
      return std::numeric_limits<double>::infinity();
  }

  double Bezier::heading(double t) const {
    Eigen::Vector2d vel = velocity(t);
    return std::atan2(vel.y(), vel.x());
  }

} // namespace bezier_sampler
} // namespace motion_planning
