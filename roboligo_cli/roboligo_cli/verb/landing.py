# Copyright 2026 Juan S. Cely G.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from ros2cli.node.direct import DirectNode

from roboligo_interfaces.srv import RoboligoString

from ros2cli.verb import VerbExtension


class LandingVerb(VerbExtension):
    """roboligo landing command."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
                  '--value', '-v', action='store_true',
                  help="Landing the robot, just work for Drones and Rovers.")

    def main(self, *, args):
        with DirectNode(args) as node:
            client = node.create_client(RoboligoString, '/roboligo/input')
            while not client.wait_for_service(timeout_sec=2.0):
                node.get_logger().info('Service not available, try again')
            req = RoboligoString.Request()
            req.data = 'landing'
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future)