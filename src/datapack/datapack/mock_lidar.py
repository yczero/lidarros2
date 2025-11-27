import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json
import os
import random
import math

# DATASET_DIR = os.path.join(os.path.dirname(__file__), "lds02_dataset")
DATASET_DIR = "/home/zero/ros2data/lds02_dataset"
class MockLidarPublisher(Node):
    def __init__(self):
        super().__init__('mock_lidar_publisher')
        self.pub = self.create_publisher(LaserScan, 'scan', 10)

        # JSON 파일 목록 읽기
        self.files = [
            os.path.join(DATASET_DIR, f)
            for f in os.listdir(DATASET_DIR)
            if f.endswith(".json")
        ]
        if not self.files:
            self.get_logger().error(f"No JSON files found in {DATASET_DIR}")
            return

        # 2초마다 퍼블리시
        self.timer = self.create_timer(2.0, self.publish_random_scan)

    def publish_random_scan(self):
        file_path = random.choice(self.files)
        with open(file_path, "r", encoding="utf-8") as f:
            scan_data = json.load(f)

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = scan_data["angle_min"]
        scan_msg.angle_max = scan_data["angle_max"]
        scan_msg.angle_increment = scan_data["angle_increment"]
        scan_msg.range_min = scan_data["range_min"]
        scan_msg.range_max = scan_data["range_max"]
        scan_msg.ranges = scan_data["ranges"]
        scan_msg.intensities = scan_data["intensities"]

        self.pub.publish(scan_msg)
        pattern = scan_data.get("meta", {}).get("pattern", "unknown")
        self.get_logger().info(f"Published pattern: {pattern}")

def main(args=None):
    rclpy.init(args=args)
    node = MockLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()