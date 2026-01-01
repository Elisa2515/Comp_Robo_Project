"""
DrawSquareSimple
----------------
This file is meant to drive the robot in a square
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread
from time import sleep


class DrawSquareSimple(Node):
    """Drive in a square using sleep"""

    def __init__(self):
        super().__init__('draw_square_simple')

        # Publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Start the thread that executes the square
        self.thread = Thread(target=self.run_loop)
        self.thread.start()

        self.get_logger().info("DrawSquareSimple node started.")

    def drive(self, linear, angular):
        """Publish twist message."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def drive_forward(self, distance):
        """Drive straight using simple timing."""
        speed = 0.2  # m/s
        duration = distance / speed

        self.drive(speed, 0.0)
        sleep(duration)
        self.drive(0.0, 0.0)

    def turn(self, angle_radians):
        """Turn in place using timing."""
        ang_speed = 0.4  # rad/s
        duration = angle_radians / ang_speed

        self.drive(0.0, ang_speed)
        sleep(duration)
        self.drive(0.0, 0.0)

    def run_loop(self):
        """Main function that drives the square."""
        sleep(1.0)  # small delay before starting

        for i in range(4):
            self.get_logger().info(f"Side {i+1}: driving forward")
            self.drive_forward(0.5)

            self.get_logger().info(f"Side {i+1}: turning 90 degrees")
            self.turn(math.pi / 2)

        self.get_logger().info("Square complete!")
        self.drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
