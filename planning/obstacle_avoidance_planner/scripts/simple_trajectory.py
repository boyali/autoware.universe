#!/usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import time

from autoware_auto_vehicle_msgs.msg import Engage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class SimpleTrajectoryNode(Node):
    def __init__(self, map_name, route=0, no_engage=False):
        super().__init__("simple_trajectory")
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            # '/sensing/gnss/pose_with_covariance', 1)
            "/initialpose",
            1,
        )
        self.goal_pub = self.create_publisher(PoseStamped, "/planning/mission_planning/goal", 1)
        self.checkpoint_pub = self.create_publisher(
            PoseStamped, "/planning/mission_planning/checkpoint", 1
        )
        self.engage_pub = self.create_publisher(Engage, "/autoware/engage", 1)
        time.sleep(1)

        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = "map"
        self.initial_pose.pose.covariance = [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.06853892326654787,
        ]

        self.checkpoint_pose = PoseStamped()
        self.checkpoint_pose.header.frame_id = "map"

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"

    def loop(self):
        if map_name == "kashiwanoha":
            if route == 0:  # 0 curve
                self.initial_pose.pose.pose.position.x = 3776.375244140625
                self.initial_pose.pose.pose.position.y = 73718.6953125
                self.initial_pose.pose.pose.orientation.z = 0.8500498956431319
                self.initial_pose.pose.pose.orientation.w = 0.526702169083345

                self.goal_pose.pose.position.x = 3819.2685546875
                self.goal_pose.pose.position.y = 73720.8203125
                self.goal_pose.pose.orientation.z = -0.9651853898167073
                self.goal_pose.pose.orientation.w = 0.26156674728330975
            elif route == 1:  # 1 curve
                self.initial_pose.pose.pose.position.x = 3766.744873046875
                self.initial_pose.pose.pose.position.y = 73740.4375
                self.initial_pose.pose.pose.orientation.z = 0.6552989486556383
                self.initial_pose.pose.pose.orientation.w = 0.7553696365957631

                self.goal_pose.pose.position.x = 3819.2685546875
                self.goal_pose.pose.position.y = 73720.8203125
                self.goal_pose.pose.orientation.z = -0.9651853898167073
                self.goal_pose.pose.orientation.w = 0.26156674728330975
            elif route == 2:  # 1 curve
                self.initial_pose.pose.pose.position.x = 3774.8994140625
                self.initial_pose.pose.pose.position.y = 73721.9140625
                self.initial_pose.pose.pose.orientation.z = 0.862067220244219
                self.initial_pose.pose.pose.orientation.w = 0.506793946738701

                self.goal_pose.pose.position.x = 3819.2685546875
                self.goal_pose.pose.position.y = 73720.8203125
                self.goal_pose.pose.orientation.z = -0.9651853898167073
                self.goal_pose.pose.orientation.w = 0.26156674728330975
            elif route == 3:  # 2 curves
                pass
            elif route == 4:  # straight line with cars
                pass
        elif map_name == "jari":
            if route == 0:  # L curve
                self.initial_pose.pose.pose.position.x = 17055.1484375
                self.initial_pose.pose.pose.position.y = 93148.4453125
                self.initial_pose.pose.pose.orientation.z = 0.7140408257289201
                self.initial_pose.pose.pose.orientation.w = 0.7001040631165927

                self.goal_pose.pose.position.x = 17006.634765625
                self.goal_pose.pose.position.y = 93234.4375
                self.goal_pose.pose.orientation.z = 0.6942983765788977
                self.goal_pose.pose.orientation.w = 0.7196872683880876
            elif route == 1:  # S curve
                self.initial_pose.pose.pose.position.x = 17008.623046875
                self.initial_pose.pose.pose.position.y = 93215.9453125
                self.initial_pose.pose.pose.orientation.z = -0.7299569291325098
                self.initial_pose.pose.pose.orientation.w = 0.6834931467187042

                self.goal_pose.pose.position.x = 17058.244140625
                self.goal_pose.pose.position.y = 93138.3046875
                self.goal_pose.pose.orientation.z = -0.6827896769751527
                self.goal_pose.pose.orientation.w = 0.7306149854856295
        elif map_name == "odaiba":
            if route == 0:  # left: first curve
                self.initial_pose.pose.pose.position.x = 88984.2265625
                self.initial_pose.pose.pose.position.y = 42579.71484375
                self.initial_pose.pose.pose.orientation.z = -0.9567246316280491
                self.initial_pose.pose.pose.orientation.w = 0.29099480963785884

                # self.checkpoint_pose.pose.position.x = 89107.1640625
                # self.checkpoint_pose.pose.position.y = 42325.5859375
                # self.checkpoint_pose.pose.orientation.z = -0.48011764444185606
                # self.checkpoint_pose.pose.orientation.w = 0.8772041082300079
                self.checkpoint_pose.pose.position.x = 89178.78125
                self.checkpoint_pose.pose.position.y = 42220.265625
                self.checkpoint_pose.pose.orientation.z = -0.47287394959909756
                self.checkpoint_pose.pose.orientation.w = 0.8811300856233149

                self.goal_pose.pose.position.x = 89298.625
                self.goal_pose.pose.position.y = 42784.3515625
                self.goal_pose.pose.orientation.z = -0.9551960841794407
                self.goal_pose.pose.orientation.w = 0.29597371634701447
            if route == 1:  # left: third curve
                self.initial_pose.pose.pose.position.x = 89571.1953125
                self.initial_pose.pose.pose.position.y = 42301.1171875
                self.initial_pose.pose.pose.orientation.z = 0.2879980505988256
                self.initial_pose.pose.pose.orientation.w = 0.9576309951391905

                self.goal_pose.pose.position.x = 89562.2265625
                self.goal_pose.pose.position.y = 42358.265625
                self.goal_pose.pose.orientation.z = 0.8778442651657494
                self.goal_pose.pose.orientation.w = 0.4789461829011745
            if route == 2:  # long trajectory
                self.initial_pose.pose.pose.position.x = 88979.9921875
                self.initial_pose.pose.pose.position.y = 42577.0078125
                self.initial_pose.pose.pose.orientation.z = -0.9624462541741448
                self.initial_pose.pose.pose.orientation.w = 0.2714722966089863

                self.goal_pose.pose.position.x = 89562.2265625
                self.goal_pose.pose.position.y = 42358.265625
                self.goal_pose.pose.orientation.z = 0.8778442651657494
                self.goal_pose.pose.orientation.w = 0.4789461829011745

        time.sleep(0.5)
        self.initial_pub.publish(self.initial_pose)

        time.sleep(0.5)
        self.goal_pub.publish(self.goal_pose)

        time.sleep(0.5)
        self.checkpoint_pub.publish(self.checkpoint_pose)

        if not no_engage:
            time.sleep(8)
            msg = Engage()
            msg.engage = True
            self.engage_pub.publish(msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map-name", default="kashiwanoha", type=str)
    parser.add_argument("-r", "--route", default=0, type=int)
    parser.add_argument("-n", "--no-engage", action="store_true")

    args = parser.parse_args()
    map_name = args.map_name
    route = args.route
    no_engage = args.no_engage

    rclpy.init()

    node = SimpleTrajectoryNode(map_name, route, no_engage)
    node.loop()

    node.destroy_node()
    rclpy.shutdown()
