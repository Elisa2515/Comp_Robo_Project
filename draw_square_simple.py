"""
DrawSquareSimple
----------------
Drive the robot in a square using open-loop timing (sleep).
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread
from time import sleep


class DrawSquareSimple(Node):
    """Drive in a square using sleep-based timing."""

    def __init__(self):
        super().__init__('draw_square_simple')

        # Publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Start the thread that executes the square
        self.thread = Thread(target=self.run_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("DrawSquareSimple node started.")

    def drive(self, linear, angular):
        """Publish a Twist message."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def stop(self):
        """Stop the robot."""
        self.drive(0.0, 0.0)

    def drive_forward(self, distance_m):
        """Drive straight for a distance using timing."""
        speed = 0.2  # m/s
        duration = abs(distance_m) / speed

        direction = 1.0 if distance_m >= 0 else -1.0
        self.drive(direction * speed, 0.0)
        sleep(duration)
        self.stop()
        sleep(0.2)  # brief settle

    def turn(self, angle_rad):
        """Turn in place for an angle using timing."""
        ang_speed = 0.4  # rad/s
        duration = abs(angle_rad) / ang_speed

        direction = 1.0 if angle_rad >= 0 else -1.0
        self.drive(0.0, direction * ang_speed)
        sleep(duration)
        self.stop()
        sleep(0.2)  # brief settle

    def run_loop(self):
        """Main behavior: drive a 0.5m square."""
        sleep(1.0)  # small delay before starting

        side_len = 0.5
        turn_ang = math.pi / 2

        for i in range(4):
            self.get_logger().info(f"Side {i+1}: driving forward {side_len} m")
            self.drive_forward(side_len)

            self.get_logger().info(f"Side {i+1}: turning 90 degrees")
            self.turn(turn_ang)

        self.get_logger().info("Square complete!")
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
