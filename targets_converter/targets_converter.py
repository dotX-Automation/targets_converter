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
from dua_mission_interfaces.msg import VisualTargets
from geometry_msgs.msg import PoseWithCovariance, PoseStamped
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, BoundingBox2D

from dua_geometry_interfaces.srv import TransformPose


class TargetsConverter(NodeBase):
    """Node that converts target information from a given reference frame to another."""

    # _PARAMS_FILE_PATH = "/opt/ros/dua-utils/install/share/targets_converter/targets_converter_params.yaml"
    _PARAMS_FILE_PATH = "/home/neo/workspace/src/targets_converter/targets_converter/targets_converter_params.yaml"

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

    def init_service_clients(self):
        """Initializes service clients."""
        # transform_pose
        self._transform_pose_cln = self.dua_create_service_client(
            TransformPose,
            "/transform_pose",
            self._wait_servers
        )

    def _in_clbk(self, msg: VisualTargets):
        """Converts and republishes target data."""
        # Prepare output message with invariant information
        out_msg = VisualTargets()
        out_msg.camera_info = msg.camera_info
        out_msg.image = msg.image
        out_msg.targets.header = msg.targets.header

        # Process detections data
        for det in msg.targets.detections:
            det: Detection2D
            out_det = Detection2D()
            out_det.header = det.header
            out_det.header.frame_id = self._out_frame_id
            out_det.bbox = det.bbox
            out_det.id = det.id
            for res in det.results:
                res: ObjectHypothesisWithPose
                out_res = res
                req = TransformPose.Request()
                req.source_pose.header = det.header
                req.source_pose.pose = res.pose.pose
                req.target.stamp = det.header.stamp
                req.target.frame_id = self._out_frame_id
                resp: TransformPose.Response = self._transform_pose_cln.call_sync(req)
                if resp.result.result != CommandResultStamped.SUCCESS:
                    self.get_logger().error(
                        f"Error transforming target pose '{det.header.frame_id}' -> '{self._out_frame_id}': {resp.result.error_msg}"
                    )
                    continue
                out_res.pose.pose = resp.target_pose.pose
                out_det.results.append(out_res)
            out_msg.targets.detections.append(out_det)

        # Publish converted message
        self._out_pub.publish(out_msg)
