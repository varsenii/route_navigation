#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistConverter(Node):
    def __init__(self):
        super().__init__("twist_converter")
        self.sub = self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.remove_timestamp_callback,
            10,
        )
        # Republish as an unstamped Twist
        self.pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)

    def remove_timestamp_callback(self, msg: TwistStamped):
        self.pub.publish(msg.twist)


def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
