#!/usr/bin/env python3
"""
Sensor helper functions for turtlebot_nav.

Provides lightweight processing for LaserScan and Odometry messages.
Now returns *real distances in meters* instead of normalized values.
"""

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def process_scan(scan_msg: LaserScan, max_range: float = 6.0, downsample: int = 1):
    """
    Convert a LaserScan message into a 1D NumPy array of float distances in meters.

    Args:
        scan_msg: sensor_msgs.msg.LaserScan
        max_range: float, cap distance readings beyond this value
        downsample: int, keep every Nth measurement (1 = keep all)

    Returns:
        np.ndarray: 1D float array of distances in meters, clipped to [0, max_range]
    """
    # Replace inf/nan with max_range for stability
    ranges = np.array(
        [r if np.isfinite(r) else max_range for r in scan_msg.ranges],
        dtype=np.float32
    )

    if downsample > 1:
        ranges = ranges[::downsample]

    # Clip to valid distance range
    ranges = np.clip(ranges, 0.0, max_range)
    return ranges


def normalize_scan(scan_data, max_range: float = 6.0):
    """
    Normalize scan data to [0, 1] range (optional utility).

    Args:
        scan_data: sequence or np.ndarray of distances in meters
        max_range: float normalization denominator

    Returns:
        np.ndarray normalized to [0, 1]
    """
    arr = np.array(scan_data, dtype=np.float32)
    arr = np.clip(arr, 0.0, max_range)
    return arr / max_range


def pose_to_state(odom_msg: Odometry):
    """
    Convert an Odometry message into a (x, y, theta) state vector.

    Args:
        odom_msg: nav_msgs.msg.Odometry

    Returns:
        np.ndarray: (x, y, theta)
    """
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    q = odom_msg.pose.pose.orientation

    # Compute yaw (theta) from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    theta = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([x, y, theta], dtype=np.float32)

