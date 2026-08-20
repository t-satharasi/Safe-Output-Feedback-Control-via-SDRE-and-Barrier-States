#!/usr/bin/env python3
"""
LyLA Visualization Node
Publishes desired trajectory and drone position as RViz markers
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import torch
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from lyla_controller import LyLA_forROS as LyAT

mavros_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10)

class LyLAViz(Node):
    def __init__(self):
        super().__init__('lyla_viz')

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/lyla/markers', 10)
        self.desired_path_pub = self.create_publisher(Path, '/lyla/desired_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/lyla/actual_path', 10)

        # Subscriber for drone position
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'autonomy_park/pose',
            self.pose_callback,
            mavros_qos)

        self.drone_position = None
        self.actual_path = Path()
        self.actual_path.header.frame_id = "map"

        # Pre-compute full desired trajectory for display
        self.desired_path = self.compute_desired_path()

        # Timer to publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_viz)
        self.t = 0.0
        self.get_logger().info("LyLA Visualization Node Started")

    def compute_desired_path(self):
        path = Path()
        path.header.frame_id = "map"
        T_final = 42.0  # one full cycle of the 8-figure
        dt = 0.1
        t = 0.0
        while t <= T_final:
            t_tensor = torch.tensor(t, dtype=torch.float32)
            xd, _ = LyAT.Dynamics.desired_trajectory(t_tensor)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = xd[0].item()
            pose.pose.position.y = xd[1].item()
            pose.pose.position.z = xd[2].item()
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            t += dt
        return path

    def pose_callback(self, msg):
        self.drone_position = msg

        # Append to actual path
        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.actual_path.poses.append(msg)
        # Keep last 500 poses
        if len(self.actual_path.poses) > 500:
            self.actual_path.poses.pop(0)

    def publish_viz(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # Update desired path timestamp
        self.desired_path.header.stamp = now
        self.desired_path_pub.publish(self.desired_path)

        # Publish actual path
        self.actual_path_pub.publish(self.actual_path)

        # Moving desired position marker (sphere)
        t_tensor = torch.tensor(self.t, dtype=torch.float32)
        xd, _ = LyAT.Dynamics.desired_trajectory(t_tensor)

        desired_marker = Marker()
        desired_marker.header.frame_id = "map"
        desired_marker.header.stamp = now
        desired_marker.ns = "desired"
        desired_marker.id = 0
        desired_marker.type = Marker.SPHERE
        desired_marker.action = Marker.ADD
        desired_marker.pose.position.x = xd[0].item()
        desired_marker.pose.position.y = xd[1].item()
        desired_marker.pose.position.z = xd[2].item()
        desired_marker.pose.orientation.w = 1.0
        desired_marker.scale.x = 0.4
        desired_marker.scale.y = 0.4
        desired_marker.scale.z = 0.4
        desired_marker.color.r = 0.0
        desired_marker.color.g = 0.0
        desired_marker.color.b = 1.0
        desired_marker.color.a = 1.0
        markers.markers.append(desired_marker)

        # Drone position marker (sphere)
        if self.drone_position is not None:
            drone_marker = Marker()
            drone_marker.header.frame_id = "map"
            drone_marker.header.stamp = now
            drone_marker.ns = "drone"
            drone_marker.id = 1
            drone_marker.type = Marker.SPHERE
            drone_marker.action = Marker.ADD
            drone_marker.pose = self.drone_position.pose
            drone_marker.scale.x = 0.4
            drone_marker.scale.y = 0.4
            drone_marker.scale.z = 0.4
            drone_marker.color.r = 1.0
            drone_marker.color.g = 0.0
            drone_marker.color.b = 0.0
            drone_marker.color.a = 1.0
            markers.markers.append(drone_marker)

        self.marker_pub.publish(markers)
        self.t += 0.1
        if self.t > 42.0:
            self.t = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = LyLAViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
