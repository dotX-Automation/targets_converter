"""
ROS 2 utility to handle mission target information.

dotX Automation s.r.l. <info@dotxautomation.com>

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

import rclpy

import dua_qos_py.dua_qos_reliable as dua_qos
from dua_node_py.dua_node import NodeBase

from dua_common_interfaces.msg import CommandResultStamped
from dua_mission_interfaces.msg import Target2D, VisualTargets
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D, ObjectHypothesisWithPose


class TargetsConverter(NodeBase):
    """Node that converts target information from a given reference frame to another."""

    _PARAMS_FILE_PATH = "/opt/ros/dua-utils/install/share/targets_converter/targets_converter_params.yaml"

    def __init__(self):
        """Constructor."""
        super().__init__("targets_converter", True)
        self.get_logger().info("Node initialized")

    def init_cgroups(self):
        """Initializes callback groups."""
        self._in_cgroup = self.dua_create_exclusive_cgroup()

    def init_subscribers(self):
        """Initializes subscribers."""
        # targets_in
        self._in_sub = self.dua_create_subscription(
            VisualTargets,
            "/targets_in",
            self._in_clbk,
            dua_qos.get_datum_qos(),
            self._in_cgroup
        )

    def init_publishers(self):
        """Initializes publishers."""
        # targets_out
        self._out_pub = self.dua_create_publisher(
            VisualTargets,
            "/targets_out",
            dua_qos.get_datum_qos()
        )

    def _in_clbk(self, msg: VisualTargets):
        """Converts and republishes target data."""
        # Prepare output message with invariant information
        out_msg = VisualTargets()
        out_msg.targets.header = msg.targets.header
        out_msg.targets.header.frame_id = self._out_frame_id
        out_msg.targets.camera_info = msg.targets.camera_info
        out_msg.image = msg.image

        # Process detected targets data
        src_frame_id = msg.targets.header.frame_id
        src_stamp = msg.targets.header.stamp
        for tgt in msg.targets.targets:
            tgt: Target2D
            out_tgt = Target2D()
            out_tgt.bbox = tgt.bbox
            out_tgt.corners = tgt.corners
            out_tgt.id = tgt.id
            for res in tgt.results:
                res: ObjectHypothesisWithPose
                # Check if pose is valid
                if res.pose.covariance[0] >= self._cov_threshold:
                    self.get_logger().warn(
                        f"Skipping target '{res.hypothesis.class_id}' with invalid covariance from '{src_frame_id}'",
                        throttle_duration_sec=0.5
                    )
                    continue

                # Convert pose in desired frame
                out_res = res
                source_pose = PoseStamped()
                source_pose.header.stamp = src_stamp
                source_pose.header.frame_id = src_frame_id
                source_pose.pose = res.pose.pose
                target_header = Header()
                target_header.stamp = src_stamp
                target_header.frame_id = self._out_frame_id
                self.get_logger().info(
                    f"Transforming pose of '{res.hypothesis.class_id}': '{source_pose.header.frame_id}' -> '{self._out_frame_id}'",
                    throttle_duration_sec=0.5
                )
                ret_code, ret_pose = self.transform_pose(source_pose, target_header, 1.0, False, 1.0)
                if ret_code == CommandResultStamped.TIMEOUT or ret_code == CommandResultStamped.ERROR:
                    continue
                out_res.pose.pose = ret_pose.pose
                if not self._orientation:
                    # If orientation data is not required/consistent, reset it to identity
                    out_res.pose.pose.orientation.w = 1.0
                    out_res.pose.pose.orientation.x = 0.0
                    out_res.pose.pose.orientation.y = 0.0
                    out_res.pose.pose.orientation.z = 0.0
                out_tgt.results.append(out_res)
            out_msg.targets.targets.append(out_tgt)

        # Publish converted message
        self._out_pub.publish(out_msg)
