"""
WallFollowerSimple
------------------
Subscribes to: /scan   (sensor_msgs/LaserScan)
Publishes to: /cmd_vel (geometry_msgs/Twist)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowerSimple(Node):
    def __init__(self):
        super().__init__('wall_follower_simple')

        # Desired distance from wall (meters)
        self.follow_offset = 0.7

        # Control gains
        self.k_dist = 0.5       # how hard we correct distance error
        self.k_heading = 1.0    # how hard we correct heading error

        # Speed limits
        self.max_lin = 0.2      # m/s
        self.min_lin = 0.05     # m/s
        self.max_ang = 1.0      # rad/s

        # Latest wall state
        self.wall_distance = None   # meters
        self.wall_angle_deg = None  # degrees (0–360)

        # ROS interfaces
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.process_laser, 10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Run control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.run_loop)

        self.get_logger().info("WallFollowerSimple node started.")

    # -------------------------------------------------------------------------
    # Laser processing
    # -------------------------------------------------------------------------
    def process_laser(self, msg: LaserScan):
        """Find nearest valid point in the scan and record its distance + angle."""
        ranges = np.array(msg.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            self.wall_distance = None
            self.wall_angle_deg = None
            return

        # Filter invalid (inf / NaN) measurements
        valid = np.isfinite(ranges)
        if not np.any(valid):
            self.wall_distance = None
            self.wall_angle_deg = None
            return

        ranges = ranges[valid]

        # Compute angle for each range value
        indices = np.arange(n)[valid]
        angles = msg.angle_min + indices * msg.angle_increment  # radians
        angles_deg = np.degrees(angles)
        # Wrap into [0, 360)
        angles_deg = (angles_deg + 360.0) % 360.0

        # Find the closest point
        idx = int(np.argmin(ranges))
        d = float(ranges[idx])
        a_deg = float(angles_deg[idx])

        # Optionally ignore walls that are too far
        if d > 3.0:
            self.wall_distance = None
            self.wall_angle_deg = None
            return

        self.wall_distance = d
        self.wall_angle_deg = a_deg

    # -------------------------------------------------------------------------
    # Control loop
    # -------------------------------------------------------------------------
    def run_loop(self):
        """Compute Twist command to follow the nearest wall."""
        twist = Twist()

        # If we don't have a valid wall measurement, rotate slowly to search
        if self.wall_distance is None or self.wall_angle_deg is None:
            twist.angular.z = 0.3
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            return

        # Decide which side the wall is on:
        #   left side  ~ 90 deg,  right side ~ 270 deg
        if self.wall_angle_deg >= 180.0:
            closest_perp = 270.0   # wall is on the right
        else:
            closest_perp = 90.0    # wall is on the left

        # Heading error: how far the wall is from the ideal perpendicular
        heading_error_deg = self.wall_angle_deg - closest_perp
        heading_error_rad = math.radians(heading_error_deg)

        # Distance error: positive if we are too far, negative if too close
        dist_error = self.follow_offset - self.wall_distance

        # Simple proportional control
        # Angular velocity: turn to keep wall at closest_perp
        twist.angular.z = -self.k_heading * heading_error_rad

        # Linear velocity: move forward, but adjust based on distance error
        base_speed = 0.15
        twist.linear.x = base_speed + self.k_dist * dist_error

        # Clamp speeds
        twist.linear.x = max(self.min_lin, min(self.max_lin, twist.linear.x))
        twist.angular.z = max(-self.max_ang, min(self.max_ang, twist.angular.z))

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
