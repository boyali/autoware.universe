/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
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

#include "nonlinear_mpc_core/nmpc_data_trajectory.hpp"
#include <algorithm>
#include <limits>
#include <vector>

ns_data::MPCdataTrajectoryVectors::MPCdataTrajectoryVectors(size_t const &traj_size) {
    s.reserve(traj_size);
    t.reserve(traj_size);
    acc.reserve(traj_size);
    acc.reserve(traj_size);
    x.reserve(traj_size);
    y.reserve(traj_size);
    z.reserve(traj_size);
    yaw.reserve(traj_size);
    vx.reserve(traj_size);
    curvature.reserve(traj_size);
}

void ns_data::MPCdataTrajectoryVectors::emplace_back(
        autoware_planning_msgs::msg::Trajectory const &msg) {
    auto const &&EPS = std::numeric_limits<double>::epsilon();
    // Insert the first elements.
    s.emplace_back(0.0);
    t.emplace_back(0.0);

    // we append acc at the end by repeating the same acc.
    auto const &xw0 = msg.points.at(0).pose.position.x;
    auto const &yw0 = msg.points.at(0).pose.position.y;
    auto const &zw0 = msg.points.at(0).pose.position.z;

    //  x.emplace_back(0.0);
    // y.emplace_back(0.0);
    // z.emplace_back(0.0);

    x.emplace_back(xw0);
    y.emplace_back(yw0);
    z.emplace_back(zw0);

    yaw.emplace_back(tf2::getYaw(msg.points.at(0).pose.orientation));
    vx.emplace_back(msg.points.at(0).twist.linear.x);

    // copy x,y, psi, vx of raw trajectory into their corresponding vectors.
    for (size_t k = 1; k < msg.points.size(); ++k) {
        // compute ds and dt.
        auto const &point0 = msg.points.at(k - 1);
        auto const &point1 = msg.points.at(k);

        double const &dx = point1.pose.position.x - point0.pose.position.x;
        double const &dy = point1.pose.position.y - point0.pose.position.y;
        double const &dz = point1.pose.position.z - point0.pose.position.z;

        double const &ds = std::sqrt(dx * dx + dy * dy + dz * dz);
        double const &mean_v =
                (point0.twist.linear.x + point1.twist.linear.x) / 2;  // used for trapezoidal integration

        double dv = point1.twist.linear.x - point0.twist.linear.x;  // dv = v[k]-v[k-1],

        double &&dt = ds / ns_utils::clamp(mean_v, EPS, mean_v);  // !<@brief to prevent zero division.
        double &&acc_computed =
                dv / (EPS + dt);  // !<@brief this acceleration is implied by x,y,z and vx in the planner.

        // Insert s, t and acc,
        s.emplace_back(s[k - 1] + ds);
        t.emplace_back(t[k - 1] + dt);
        acc.emplace_back(acc_computed);

        // Insert the rest of x, y, psi, v.
        // x.emplace_back(point1.pose.position.x - xw0);
        // y.emplace_back(point1.pose.position.y - yw0);
        // z.emplace_back(point1.pose.position.z - zw0);

        x.emplace_back(point1.pose.position.x);
        y.emplace_back(point1.pose.position.y);
        z.emplace_back(point1.pose.position.z);

        yaw.emplace_back(tf2::getYaw(point1.pose.orientation));
        vx.emplace_back(point1.twist.linear.x);
    }

    // Repeat acc last value to make all the same size.
    acc.emplace_back(acc.back());

    // Convert heading angle to a monotonic series
    ns_utils::convertEulerAngleToMonotonic(&yaw);

    // Set curvature to zero for the raw curvature.
    curvature = std::vector<double>(s.size(), 0.0);

    //  Add end-points as an extra points.
    //    double &&ds = 1.0e-1;             // !<@brief for guaranteeing the monotonicity condition in s.
    //    s.emplace_back(s.back() + ds);  // !<@brief zero velocity no motion.
    //
    //    double &&t_ext = 1.0;  // !<@brief extra time for MPC
    //    t.emplace_back(t.back() + t_ext);

    // Add x, y points on the current direction line.
    //    auto xtemp = x.back();
    //    auto ytemp = y.back();
    //    auto yaw_temp = yaw.back();

    //    auto &&tangent_vector = ns_utils::getTangentVector(yaw_temp);
    //    xtemp += ds * tangent_vector[0];
    //    ytemp += ds * tangent_vector[1];

    //    x.emplace_back(x.back());
    //    y.emplace_back(y.back());
    //    x.emplace_back(xtemp);
    //    y.emplace_back(ytemp);
    //    z.emplace_back(z.back());
    //    yaw.emplace_back(yaw.back());
    //
    //    vx.emplace_back(vx.back());  // !<@brief vend = 0.0.
    //    acc.emplace_back(acc.back());
    //    curvature.emplace_back(0.0);
}

void ns_data::MPCdataTrajectoryVectors::clear() {
    s.clear();  // !<@brief arc-length.
    t.clear();  // !<@brief relative-time.
    curvature.clear();

    acc.clear();  // !<@brief implied acceleration.

    x.clear();
    y.clear();
    z.clear();
    yaw.clear();
    vx.clear();
}

size_t ns_data::MPCdataTrajectoryVectors::size() const {
    if (
            x.size() == y.size() && x.size() == z.size() && x.size() == yaw.size() &&
            x.size() == vx.size() && x.size() == curvature.size() && x.size() == t.size() &&
            x.size() == s.size() && x.size() == acc.size()) {
        return x.size();
    } else {
        ns_utils::print("s size : ", s.size());
        ns_utils::print("t size : ", t.size());
        ns_utils::print("curv size : ", curvature.size());
        ns_utils::print("x size : ", x.size());
        ns_utils::print("y size : ", y.size());
        ns_utils::print("z size : ", z.size());
        ns_utils::print("yaw size : ", yaw.size());
        ns_utils::print("vx size : ", vx.size());
        ns_utils::print("ax size : ", acc.size());
        std::cerr << "[MPC trajectory] trajectory size is inappropriate" << std::endl;
        return 0;
    }
}

void ns_data::MPCdataTrajectoryVectors::setTrajectoryCoordinate(
        char const &coord_name, std::vector<double> const &data_vect) {
    switch (coord_name) {
        case 's':
            s = data_vect;
            break;

        case 't':
            t = data_vect;
            break;

        case 'a':
            acc = data_vect;
            break;

        case 'x':
            x = data_vect;
            break;

        case 'y':
            y = data_vect;
            break;

        case 'z':
            z = data_vect;
            break;

        case 'v':
            vx = data_vect;
            break;

        case 'w':
            yaw = data_vect;
            break;

        case 'c':
            curvature = data_vect;
            break;

        default:
            break;
    }
}

void ns_data::MPCdataTrajectoryVectors::addExtraEndPoints(double const &avg_mpc_compute_time) {
    //  Add end-points as an extra points.
    double const ds = 2.0e-1;          // for guaranteeing the monotonicity condition in s.
    s.emplace_back(s.back() + ds);  // zero velocity no motion.

    double &&t_ext = 1.0 + avg_mpc_compute_time;
    t.emplace_back(t.back() + t_ext);

    // Add x, y points on the current direction line.
    auto xtemp = x.back();
    auto ytemp = y.back();
    auto yaw_temp = yaw.back();
    // auto dyaw = yaw.rbegin()[0] - yaw.rbegin()[1];

    auto &&tangent_vector = ns_utils::getTangentVector(yaw_temp);
    xtemp += ds * tangent_vector[0];
    ytemp += ds * tangent_vector[1];

    x.emplace_back(xtemp);
    y.emplace_back(ytemp);

    z.emplace_back(z.back());
    yaw.emplace_back(yaw.back());

    vx.emplace_back(vx.back());  // vend = 0.0.
    acc.emplace_back(acc.back());
    curvature.emplace_back(0.0);
}