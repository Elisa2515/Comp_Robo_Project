"""
Person follower Simple
The way it works is the neato looks in a front window and follows the closest object.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PersonFollowerSimple(Node):
    def __init__(self):
        super().__init__('person_follower_simple')

        self.follow_offset = 0.7   # desired distance [m]
        self.variance = 10.0       # angle tolerance [deg]

        self.person_angle = None   # degrees, 0 = straight ahead
        self.person_distance = None

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.process_laser, 10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.run_loop)
        self.get_logger().info("PersonFollowerSimple node started.")

    def process_laser(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        n = len(ranges)
        if n == 0:
            self.person_angle = None
            self.person_distance = None
            return

        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = np.isfinite(ranges)
        if not np.any(valid):
            self.person_angle = None
            self.person_distance = None
            return

        ranges = ranges[valid]
        angles = angles[valid]

        angle_deg = np.degrees(angles)
        # look only in ~±30 degrees in front
        front_mask = (angle_deg > -30) & (angle_deg < 30)
        if not np.any(front_mask):
            self.person_angle = None
            self.person_distance = None
            return

        front_ranges = ranges[front_mask]
        front_angles = angle_deg[front_mask]

        idx = np.argmin(front_ranges)
        d = float(front_ranges[idx])

        if d < 0.1 or d > 3.0:
            self.person_angle = None
            self.person_distance = None
            return

        self.person_distance = d
        self.person_angle = float(front_angles[idx])

    def run_loop(self):
        twist = Twist()

        if self.person_angle is None or self.person_distance is None:
            # search by rotating slowly
            twist.angular.z = 0.3
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            return

        angle_error = math.radians(self.person_angle)
        dist_error = self.person_distance - self.follow_offset

        k_ang = 1.0
        k_lin = 0.5

        twist.angular.z = -k_ang * angle_error

        if abs(self.person_angle) < self.variance:
            twist.linear.x = k_lin * dist_error
        else:
            twist.linear.x = 0.0

        twist.linear.x = max(-0.2, min(0.3, twist.linear.x))
        twist.angular.z = max(-1.0, min(1.0, twist.angular.z))

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
