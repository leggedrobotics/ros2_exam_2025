#!/usr/bin/env python3

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")
        self.waypoints = self.load_waypoints()
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(MarkerArray, "waypoint_markers", qos_profile)
        self.marker_publisher.publish(self.create_waypoint_markers())
        self.get_logger().info("Published waypoint markers")

    def load_waypoints(self):
        config_path = os.path.join(
            get_package_share_directory("robot_navigation"),
            "config",
            "waypoints.yaml",
        )
        with open(config_path, "r") as f:
            return yaml.safe_load(f)["waypoints"]

    def create_waypoint_markers(self):
        marker_array = MarkerArray()

        for idx, waypoint in enumerate(self.waypoints):
            # Create arrow marker for waypoint
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = waypoint["position"]["x"]
            marker.pose.position.y = waypoint["position"]["y"]
            marker.pose.position.z = 0.0

            # Set orientation from yaw
            quaternion = yaw_to_quaternion(waypoint["yaw"])
            marker.pose.orientation = quaternion

            # Set scale
            marker.scale.x = 1.0  # arrow length
            marker.scale.y = 0.2  # arrow width
            marker.scale.z = 0.2  # arrow height

            # Set color (blue)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        return marker_array


def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
