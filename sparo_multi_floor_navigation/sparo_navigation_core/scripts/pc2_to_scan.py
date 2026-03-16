#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
import sensor_msgs_py.point_cloud2 as pc2
import math


class PC2ToScan(Node):
    def __init__(self):
        super().__init__('pc2_to_scan')

        # GO2 로봇 파라미터
        # base_link→지면: ~0.3m, base_link→LiDAR: z=0.139m → LiDAR→지면: ~0.44m
        self.declare_parameter('lidar_height', 0.44)
        self.declare_parameter('ground_margin', 0.10)  # 바닥 위 마진
        self.declare_parameter('z_max', 0.8)

        self.lidar_height = self.get_parameter('lidar_height').value
        self.ground_margin = self.get_parameter('ground_margin').value
        self.z_max = self.get_parameter('z_max').value

        # IMU에서 pitch/roll 저장
        self.pitch = 0.0
        self.roll = 0.0

        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        self.pc_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.pc_callback, 10)

        self.publisher = self.create_publisher(
            LaserScan, '/scan', 10)

        self.get_logger().info(
            f'PC2 to Scan started (lidar_height={self.lidar_height}, '
            f'ground_margin={self.ground_margin}, z_max={self.z_max})')

    def imu_callback(self, msg):
        # 쿼터니언 → pitch, roll 변환
        q = msg.orientation
        # roll (x축 회전)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr, cosr)
        # pitch (y축 회전)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.asin(sinp)

    def pc_callback(self, cloud_msg):
        sin_pitch = math.sin(self.pitch)
        sin_roll = math.sin(self.roll)
        margin = self.ground_margin

        points_data = []

        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            x, y, z = point

            # 현재 기울기에서 이 (x,y) 위치의 바닥 z 계산 (라이다 프레임 기준)
            z_ground = -self.lidar_height + x * sin_pitch - y * sin_roll

            # 바닥 제거: z_ground + margin 이하는 바닥
            if z <= z_ground + margin:
                continue

            # 천장 제거
            if z > self.z_max:
                continue

            range_val = math.sqrt(x * x + y * y)

            if range_val < 0.1 or range_val > 100.0:
                continue

            angle = math.atan2(y, x)
            points_data.append((angle, range_val))

        if not points_data:
            return

        scan = LaserScan()
        scan.header = cloud_msg.header

        scan.angle_min = -math.pi
        scan.angle_max = math.pi

        desired_angular_resolution = 0.25 * math.pi / 180.0
        num_bins = int((scan.angle_max - scan.angle_min) / desired_angular_resolution)

        scan.angle_increment = (scan.angle_max - scan.angle_min) / num_bins
        scan.range_min = 0.1
        scan.range_max = 100.0
        scan.scan_time = 0.1
        scan.time_increment = scan.scan_time / num_bins

        scan.ranges = [float('inf')] * num_bins

        for angle, range_val in points_data:
            bin_idx = int((angle - scan.angle_min) / scan.angle_increment)
            bin_idx = max(0, min(num_bins - 1, bin_idx))

            if scan.ranges[bin_idx] == float('inf') or range_val < scan.ranges[bin_idx]:
                scan.ranges[bin_idx] = range_val

        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PC2ToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
