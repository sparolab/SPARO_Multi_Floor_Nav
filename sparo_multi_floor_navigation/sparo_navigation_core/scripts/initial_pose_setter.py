#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('floor', 'L1')

        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.floor = self.get_parameter('floor').get_parameter_value().string_value

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Wait for AMCL to be ready, then publish
        self.timer = self.create_timer(5.0, self.publish_initial_pose)
        self.get_logger().info(
            f'[InitialPoseSetter] Will set initial pose for {self.floor}: '
            f'x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}')

    def publish_initial_pose(self):
        self.timer.cancel()

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        for i in range(5):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.pose.covariance[0] = 0.25   # x
            msg.pose.covariance[7] = 0.25   # y
            msg.pose.covariance[35] = 0.07  # yaw

            self.pose_pub.publish(msg)
            self.get_logger().info(
                f'[InitialPoseSetter] Published initial pose ({i+1}/5)')
            time.sleep(0.5)

        self.get_logger().info('[InitialPoseSetter] Done. Shutting down.')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
