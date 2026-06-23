#!/usr/bin/env python3
"""Bridge people_msgs/People (/people) -> visualization_msgs/MarkerArray
(/people_markers) so the HuNavSim pedestrians can be viewed in RViz (robust
under Xwayland, unlike gzclient on the heavy cafe scene).

A cylinder per agent (body) plus an arrow for heading (yaw is packed into
people_msgs position.z, per HuNavSim convention)."""
import math

import rclpy
from rclpy.node import Node
from people_msgs.msg import People
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PeopleMarkers(Node):
    def __init__(self):
        super().__init__("people_markers")
        self.pub = self.create_publisher(MarkerArray, "people_markers", 10)
        self.create_subscription(People, "people", self.cb, 10)

    def cb(self, msg):
        arr = MarkerArray()
        for i, p in enumerate(msg.people):
            yaw = p.position.z  # HuNavSim packs yaw here
            qx, qy, qz, qw = yaw_to_quat(yaw)

            body = Marker()
            body.header = msg.header
            body.ns = "body"
            body.id = i
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = p.position.x
            body.pose.position.y = p.position.y
            body.pose.position.z = 0.85
            body.pose.orientation.w = 1.0
            body.scale.x = 0.5
            body.scale.y = 0.5
            body.scale.z = 1.7
            body.color.r = 0.1
            body.color.g = 0.7
            body.color.b = 0.9
            body.color.a = 1.0
            arr.markers.append(body)

            heading = Marker()
            heading.header = msg.header
            heading.ns = "heading"
            heading.id = i
            heading.type = Marker.ARROW
            heading.action = Marker.ADD
            heading.pose.position.x = p.position.x
            heading.pose.position.y = p.position.y
            heading.pose.position.z = 0.85
            heading.pose.orientation.x = qx
            heading.pose.orientation.y = qy
            heading.pose.orientation.z = qz
            heading.pose.orientation.w = qw
            heading.scale.x = 0.8
            heading.scale.y = 0.1
            heading.scale.z = 0.1
            heading.color.r = 1.0
            heading.color.g = 0.6
            heading.color.b = 0.0
            heading.color.a = 1.0
            arr.markers.append(heading)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = PeopleMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
