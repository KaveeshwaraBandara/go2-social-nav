#!/usr/bin/env python3
"""Phase 6c: people_to_cloud -- feed the ground-truth /people into Nav2's costmap.

Nav2's DWA/TEB controllers avoid obstacles via a costmap, which normally needs a
laser/pointcloud sensor. Our leg-locked Go2 has NO such sensor -- pedestrians are
known only through the ground-truth /people topic (the perception stub). To keep
/people the SINGLE source of truth (NOT real perception) while letting Nav2 do its
job, this node republishes each pedestrian as points in a sensor_msgs/PointCloud2
that the costmap's obstacle layer consumes. Every controller (stub, DWA, TEB,
IT2-FLS) thus reacts to the SAME pedestrian information, just in the form each needs.

The cloud is published in the robot BASE frame (origin at the robot) so the costmap
obstacle layer raytraces from the robot and clears moving pedestrians' trails, like
a real lidar would. /people is in the map frame and the scene publishes a static
identity map->odom TF, so we use the /odom pose directly to convert to base frame
(same frame assumption as stub_brain -- see CLAUDE.md).
"""
import math
import struct

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from people_msgs.msg import People
from sensor_msgs.msg import PointCloud2, PointField


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PeopleToCloud(Node):
    def __init__(self):
        super().__init__("people_to_cloud")

        self.base_frame = self.declare_parameter("base_frame", "base").value
        self.person_radius = float(self.declare_parameter("person_radius", 0.3).value)
        self.point_height = float(self.declare_parameter("point_height", 0.3).value)
        rate = float(self.declare_parameter("publish_rate", 15.0).value)

        self.robot = None     # (x, y, yaw) from /odom
        self.people = []

        self.pub = self.create_publisher(PointCloud2, "people_cloud", 10)
        self.create_subscription(Odometry, "odom", self.odom_cb, 20)
        self.create_subscription(People, "people", self.people_cb, 10)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"people_to_cloud: /people -> /people_cloud in '{self.base_frame}' frame "
            f"@ {rate:.0f} Hz (person_radius={self.person_radius} m)"
        )

    def odom_cb(self, msg):
        self.robot = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      yaw_from_quaternion(msg.pose.pose.orientation))

    def people_cb(self, msg):
        self.people = msg.people

    def tick(self):
        if self.robot is None:
            return
        rx, ry, ryaw = self.robot
        c, s = math.cos(ryaw), math.sin(ryaw)

        pts = []
        for person in self.people:
            # world (map==odom) -> robot base frame
            dx = person.position.x - rx
            dy = person.position.y - ry
            bx = dx * c + dy * s
            by = -dx * s + dy * c
            # a small ring of points so the costmap marks the pedestrian's footprint
            pts.append((bx, by, self.point_height))
            for k in range(8):
                a = k * math.pi / 4.0
                pts.append((bx + self.person_radius * math.cos(a),
                            by + self.person_radius * math.sin(a),
                            self.point_height))

        self.pub.publish(self._make_cloud(pts))

    def _make_cloud(self, pts):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        buf = bytearray()
        for (x, y, z) in pts:
            buf += struct.pack("fff", x, y, z)
        msg.data = bytes(buf)
        return msg


def main():
    rclpy.init()
    node = PeopleToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
