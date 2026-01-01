import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import quaternion_from_euler


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MiniPF(Node):
    def __init__(self):
        super().__init__("mini_pf")

        # small + fast
        self.N = 150
        self.particles = np.zeros((self.N, 3))  # x,y,theta (map-ish)
        self.weights = np.ones(self.N) / self.N

        # start near origin (edit if you want)
        self.particles[:, 0] = np.random.normal(0.0, 0.3, self.N)
        self.particles[:, 1] = np.random.normal(0.0, 0.3, self.N)
        self.particles[:, 2] = np.random.normal(0.0, 0.4, self.N)

        self.last_odom = None

        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.pub = self.create_publisher(PoseArray, "/mini_pf_particles", 10)

        self.get_logger().info("MiniPF running: publishes /mini_pf_particles")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        odom = (float(p.x), float(p.y), float(th))

        if self.last_odom is None:
            self.last_odom = odom
            return

        # odom delta
        x0, y0, th0 = self.last_odom
        x1, y1, th1 = odom
        dx, dy = x1 - x0, y1 - y0
        dth = wrap(th1 - th0)

        # motion update (with noise)
        thp = self.particles[:, 2]
        c, s = np.cos(thp), np.sin(thp)

        dxn = dx + np.random.normal(0, 0.02, self.N)
        dyn = dy + np.random.normal(0, 0.02, self.N)
        dtn = dth + np.random.normal(0, 0.03, self.N)

        self.particles[:, 0] += dxn  # simple: apply in global (good enough for demo)
        self.particles[:, 1] += dyn
        self.particles[:, 2] = np.vectorize(wrap)(self.particles[:, 2] + dtn)

        # tiny “confidence” model: prefer particles near the average (keeps cloud tight)
        mx, my = np.mean(self.particles[:, 0]), np.mean(self.particles[:, 1])
        dist2 = (self.particles[:, 0] - mx) ** 2 + (self.particles[:, 1] - my) ** 2
        w = np.exp(-dist2 / 0.5)
        self.weights = w / np.sum(w)

        self.resample()
        self.publish(msg.header.stamp)

        self.last_odom = odom

    def resample(self):
        cdf = np.cumsum(self.weights)
        step = 1.0 / self.N
        start = np.random.uniform(0, step)
        points = start + step * np.arange(self.N)
        idx = np.searchsorted(cdf, points)
        self.particles = self.particles[idx]
        self.weights.fill(1.0 / self.N)

    def publish(self, stamp):
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = "map"  # ok for RViz visualization

        poses = []
        for x, y, th in self.particles:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            qx, qy, qz, qw = quaternion_from_euler(0, 0, float(th))
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
            poses.append(pose)

        pa.poses = poses
        self.pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = MiniPF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
