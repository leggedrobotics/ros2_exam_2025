#!/usr/bin/env python3

import re
import socket
import sys

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String


class StudentIDFormatError(Exception):
    """Exception raised when student ID format is invalid."""

    pass


class EnvironmentInfoPublisher(Node):
    def __init__(self):
        super().__init__("environment_info_publisher")

        # Declare the student_id parameter with type checking and empty default
        self.declare_parameter(
            "student_id",
            "",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING,
                description="Student ID number (format: XX-XXX-XX)",
            ),
        )

        # Initial parameter validation
        student_id = self.get_parameter("student_id").value
        if not student_id or not self._validate_student_id(student_id):
            msg = f"Invalid student ID format: {student_id}, expected format: XX-XXX-XXX"
            self.get_logger().error(msg)
            raise StudentIDFormatError(msg)

        self.student_id = student_id

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        # Create publishers
        self.hostname_publisher = self.create_publisher(
            String,
            "/my_hostname",
            qos_profile,
        )
        self.student_id_publisher = self.create_publisher(
            String,
            "/my_student_id",
            qos_profile,
        )

        # Publish hostname (once)
        self.hostname = socket.gethostname()
        hostname_msg = String()
        hostname_msg.data = self.hostname
        self.hostname_publisher.publish(hostname_msg)

        # Create timer for student ID publishing (once)
        student_id_msg = String()
        student_id_msg.data = self.student_id
        self.student_id_publisher.publish(student_id_msg)

    def _validate_student_id(self, student_id):
        pattern = r"^\d{2}-\d{3}-\d{3}$"
        return bool(re.match(pattern, student_id))


def main(args=None):
    try:
        rclpy.init(args=args)
        node = EnvironmentInfoPublisher()
        rclpy.spin(node)
    except StudentIDFormatError as e:
        print(str(e), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
