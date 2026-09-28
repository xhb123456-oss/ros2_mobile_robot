#!/usr/bin/env python3
"""Read /scan without moving the robot; run after loading lidar_test.world."""
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node('check_scan')
    scans = []
    subscription = node.create_subscription(
        LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    deadline = time.monotonic() + 20.0
    print('Waiting for 11 scans (20 s timeout)...', flush=True)
    try:
        while rclpy.ok() and len(scans) < 11 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if len(scans) < 11:
            print(f'INCOMPLETE: received {len(scans)} scans; check simulation and /scan.')
            return 1
        scan = scans[-1]
        print(f'frame: {scan.header.frame_id}')
        print(f'samples: {len(scan.ranges)}')
        print(f'range limits: {scan.range_min:.3f} .. {scan.range_max:.3f} m')
        if not scan.ranges or scan.angle_increment <= 0:
            print('ERROR: empty ranges or invalid angle increment.')
            return 1
        front = min(range(len(scan.ranges)),
                    key=lambda i: abs(scan.angle_min + i * scan.angle_increment))
        angle = scan.angle_min + front * scan.angle_increment
        print(f'front ray: angle={math.degrees(angle):.3f} deg, '
              f'distance={scan.ranges[front]:.3f} m')
        finite = [r for r in scan.ranges if math.isfinite(r)]
        print(f'finite rays: {len(finite)}/{len(scan.ranges)}')
        if finite:
            print(f'nearest hit: {min(finite):.3f} m')
        stamps = [s.header.stamp.sec + s.header.stamp.nanosec * 1e-9 for s in scans]
        elapsed = stamps[-1] - stamps[0]
        if elapsed > 0 and all(b > a for a, b in zip(stamps, stamps[1:])):
            print(f'received rate (simulation time): {(len(scans) - 1) / elapsed:.2f} Hz')
        else:
            print('WARNING: message timestamps are not strictly increasing.')
        print('Expected at initial stationary pose in lidar_test.world:')
        print('laser_link; 360 rays; limits 0.12..8 m; front about 1.90 m; about 10 Hz.')
        print('These are observations, not an automatic pass or a TF check.')
        return 0
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
