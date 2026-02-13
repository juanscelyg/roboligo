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

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    params_file = join(get_package_share_directory('roboligo'), 
                         'params', 'drone.params.yaml')
    
    roboligo = Node(
            package="roboligo_system",
            executable="roboligo_main",
            output="screen",
            parameters=[
                params_file,
            ],
        )

    ld = LaunchDescription()
    ld.add_action(roboligo)
    return ld


