"""
Targets converter app launch file.

September 12, 2025
"""

# Copyright 2025 dotX Automation s.r.l.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    cov_thr = LaunchConfiguration('cov_threshold', default='0.5')
    it = LaunchConfiguration('in_topic', default='/targets_in')
    nm = LaunchConfiguration('name', default='targets_converter')
    ns = LaunchConfiguration('namespace', default='')
    of = LaunchConfiguration('out_frame', default='map')
    ort = LaunchConfiguration('orientation', default='true')
    ot = LaunchConfiguration('out_topic', default='/targets_out')
    tf_service = LaunchConfiguration('tf_service', default='/dua_tf_server/transform_pose')
    ust = LaunchConfiguration('use_sim_time', default='false')
    ws = LaunchConfiguration('wait_servers', default='false')
    cov_thr_launch_arg = DeclareLaunchArgument(
        'cov_threshold',
        default_value='0.5'
    )
    it_launch_arg = DeclareLaunchArgument(
        'in_topic',
        default_value='/targets_in'
    )
    nm_launch_arg = DeclareLaunchArgument(
        'name',
        default_value='targets_converter'
    )
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )
    of_launch_arg = DeclareLaunchArgument(
        'out_frame',
        default_value='map'
    )
    ort_launch_arg = DeclareLaunchArgument(
        'orientation',
        default_value='true'
    )
    ot_launch_arg = DeclareLaunchArgument(
        'out_topic',
        default_value='/targets_out'
    )
    tf_service_launch_arg = DeclareLaunchArgument(
        'tf_service',
        default_value='/dua_tf_server/transform_pose'
    )
    ust_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )
    ws_launch_arg = DeclareLaunchArgument(
        'wait_servers',
        default_value='false'
    )
    ld.add_action(cov_thr_launch_arg)
    ld.add_action(it_launch_arg)
    ld.add_action(nm_launch_arg)
    ld.add_action(ns_launch_arg)
    ld.add_action(of_launch_arg)
    ld.add_action(ort_launch_arg)
    ld.add_action(ot_launch_arg)
    ld.add_action(tf_service_launch_arg)
    ld.add_action(ust_launch_arg)
    ld.add_action(ws_launch_arg)

    # Create node launch description
    node = Node(
        package='targets_converter',
        executable='targets_converter',
        name=nm,
        namespace=ns,
        emulate_tty=True,
        shell=False,
        output='both',
        log_cmd=True,
        parameters=[
            {
                'cov_threshold': cov_thr,
                'orientation': ort,
                'out_frame_id': of,
                'use_sim_time': ust,
                'wait_servers': ws
            },
        ],
        remappings={
            ('/targets_in',     it),
            ('/targets_out',    ot),
            ('/transform_pose', tf_service)
        }
    )
    ld.add_action(node)

    return ld
