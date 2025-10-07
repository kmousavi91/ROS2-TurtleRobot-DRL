#!/usr/bin/env python3
"""
Minimal trainer node for turtlebot_nav.

Subscribes to LaserScan and Odometry, processes data, and publishes Twist commands
for simple anti-collision navigation in Gazebo.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

# Import local helpers
from sensor_helpers import process_scan, pose_to_state


class TrainerNode(Node):
    def __init__(self):
        super().__init__('trainer_node')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/gazebo_ros_laser_controller/out', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Internal states
        self.latest_scan = None
        self.latest_odom = None

        # Timers
        self.timer = self.create_timer(0.1, self.tick)        # 10 Hz control loop
        self.status_timer = self.create_timer(2.0, self.log_status)  # slower status updates

        self.get_logger().info('Trainer Node initialized. Waiting for sensor data...')

    def log_status(self):
        """Logs current readiness of sensor topics."""
        ready_scan = self.latest_scan is not None
        ready_odom = self.latest_odom is not None
        if not (ready_scan and ready_odom):
            self.get_logger().warn(f'Waiting for data: SCAN={ready_scan}, ODOM={ready_odom}')

    def scan_callback(self, msg: LaserScan):
        """Handles incoming LaserScan messages."""
        try:
            self.latest_scan = process_scan(msg, max_range=6.0, downsample=2)
        except Exception as e:
            self.get_logger().error(f'Scan processing error: {e}')

    def odom_callback(self, msg: Odometry):
        """Handles incoming Odometry messages."""
        try:
            self.latest_odom = pose_to_state(msg)
        except Exception as e:
            self.get_logger().error(f'Odom processing error: {e}')

    def tick(self):
        """Main control loop executed at 10 Hz."""
        if self.latest_scan is None or self.latest_odom is None:
            self.get_logger().debug('Still waiting for Scan/Odom data...')
            return

        scan = self.latest_scan
        scan_len = len(scan)
        front_region = scan[scan_len // 3 : 2 * scan_len // 3]

        # Use *real distance in meters*
        min_front = float(np.min(front_region)) if len(front_region) > 0 else 6.0

        cmd = Twist()

        # --- Anti-collision behavior ---
        if min_front < 0.25:  # < 25 cm — collision imminent
            cmd.linear.x = 0.0
            cmd.angular.z = 0.6
            self.get_logger().warn(f'Collision imminent! Obstacle {min_front:.2f} m ahead.')
        elif min_front < 0.6:  # 25–60 cm — slow down and turn slightly
            cmd.linear.x = 0.05
            cmd.angular.z = 0.3
            self.get_logger().info(f'Obstacle near ({min_front:.2f} m). Turning slowly.')
        else:  # path clear
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            self.get_logger().info(f'Path clear ({min_front:.2f} m). Moving forward.', throttle_duration_sec=2.0)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrainerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

