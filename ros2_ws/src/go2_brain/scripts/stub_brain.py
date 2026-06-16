#!/usr/bin/env python3
"""Phase 5: stub_brain — the simplest thing that closes the autonomy loop.

This node REPLACES teleop as the producer of /cmd_vel. It is a deliberately
simple, readable placeholder built on a basic Social Force Model (SFM). The
IT2-FLS controller (Phase 7) will later swap in for THIS node at the exact same
interface (/people + /odom in, /cmd_vel out), so nothing around it changes.

Interface (the permanent control contract — see CLAUDE.md):
  IN   /people  people_msgs/People    ground-truth perception stub (frame: map)
                                      per agent: position (yaw packed in
                                      position.z) and velocity.
  IN   /odom    nav_msgs/Odometry     the robot's own pose from planar_move.
  OUT  /cmd_vel geometry_msgs/Twist   body-frame velocity command @ 20 Hz.

FRAME ASSUMPTION (documented, deliberate):
  /people is in the `map` frame. /odom reports the robot pose in the `odom`
  frame. In this scene the launch publishes a STATIC IDENTITY map->odom TF
  (see go2_hunav cafe_isolated.launch.py), so `odom` and `map` coincide and we
  can treat the robot's /odom pose directly as its map-frame pose WITHOUT a TF
  listener. If that static transform ever stops being identity, this node must
  grow a tf2 lookup instead. Kept simple on purpose for Phase 5.

The Social Force Model here is intentionally minimal:
  * one ATTRACTIVE force pulling toward the goal,
  * one REPULSIVE force per nearby person, falling off with distance,
  * sum -> desired world velocity -> rotate into the body frame -> clamp.

The SAFETY FLOOR (hard speed caps + stop-if-too-close) is applied AFTER and is
INDEPENDENT of the force math, so it still protects us no matter what the SFM
(or a future controller) asks for.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from people_msgs.msg import People


def yaw_from_quaternion(q):
    """Extract the planar yaw from a geometry_msgs/Quaternion."""
    # Standard z-axis yaw from a quaternion (roll/pitch are ~0 for our planar base).
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, limit):
    """Clamp a scalar to [-limit, +limit]."""
    return max(-limit, min(limit, value))


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class StubBrain(Node):
    def __init__(self):
        super().__init__("stub_brain")

        # --- Parameters (see config/stub_brain.yaml for the meaning of each) --
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", -4.0)
        self.declare_parameter("goal_tolerance", 0.4)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("k_attractive", 1.0)
        self.declare_parameter("k_repulsive", 2.0)
        self.declare_parameter("repulsion_range", 1.5)
        self.declare_parameter("repulsion_cutoff", 4.0)
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("k_yaw", 1.5)
        self.declare_parameter("stop_distance", 0.8)

        g = self.get_parameter
        self.goal_x = g("goal_x").value
        self.goal_y = g("goal_y").value
        self.goal_tolerance = g("goal_tolerance").value
        self.control_rate = g("control_rate").value
        self.k_attractive = g("k_attractive").value
        self.k_repulsive = g("k_repulsive").value
        self.repulsion_range = g("repulsion_range").value
        self.repulsion_cutoff = g("repulsion_cutoff").value
        self.max_linear_speed = g("max_linear_speed").value
        self.max_angular_speed = g("max_angular_speed").value
        self.k_yaw = g("k_yaw").value
        self.stop_distance = g("stop_distance").value

        # --- State caches (latest message from each input) -------------------
        self.robot_x = None        # set once /odom arrives
        self.robot_y = None
        self.robot_yaw = 0.0
        self.people = []           # list of people_msgs/Person

        # --- ROS I/O ---------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.create_subscription(People, "people", self.people_cb, 10)

        # Fixed-rate control loop (decoupled from input rates) @ 20 Hz target.
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_tick)
        self._goal_reached_logged = False

        self.get_logger().info(
            f"stub_brain up: goal=({self.goal_x:.2f}, {self.goal_y:.2f}), "
            f"{self.control_rate:.0f} Hz, v_max={self.max_linear_speed} m/s, "
            f"stop_distance={self.stop_distance} m"
        )

    # --- Subscription callbacks: just cache, never compute here --------------
    def odom_cb(self, msg):
        # See FRAME ASSUMPTION above: odom == map (static identity TF), so we use
        # this pose directly as the robot's map-frame pose.
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def people_cb(self, msg):
        self.people = msg.people

    # --- The control loop ---------------------------------------------------
    def control_tick(self):
        # Can't act before we know where we are.
        if self.robot_x is None:
            return

        twist = Twist()

        # Vector to the goal (map frame).
        to_goal_x = self.goal_x - self.robot_x
        to_goal_y = self.goal_y - self.robot_y
        dist_to_goal = math.hypot(to_goal_x, to_goal_y)

        # Goal reached -> publish a hard stop and idle.
        if dist_to_goal < self.goal_tolerance:
            if not self._goal_reached_logged:
                self.get_logger().info("goal reached; holding position.")
                self._goal_reached_logged = True
            self.cmd_pub.publish(twist)  # all-zero Twist
            return
        self._goal_reached_logged = False

        # === Social Force Model (all in the map frame) =======================
        # 1) Attractive force: a unit pull toward the goal, scaled by the gain.
        fx = self.k_attractive * (to_goal_x / dist_to_goal)
        fy = self.k_attractive * (to_goal_y / dist_to_goal)

        # 2) Repulsive force: push away from each nearby person, exponential
        #    falloff with distance. Also track the nearest person for the
        #    independent safety floor below.
        nearest_dist = float("inf")
        nearest_dx = 0.0
        nearest_dy = 0.0
        for person in self.people:
            dx = self.robot_x - person.position.x   # vector pointing away from person
            dy = self.robot_y - person.position.y
            dist = math.hypot(dx, dy)
            if dist < 1e-3 or dist > self.repulsion_cutoff:
                continue  # ignore far / coincident agents (cheap + local)

            # Magnitude decays as exp(-dist / range); direction is away from them.
            magnitude = self.k_repulsive * math.exp(-dist / self.repulsion_range)
            fx += magnitude * (dx / dist)
            fy += magnitude * (dy / dist)

            if dist < nearest_dist:
                nearest_dist = dist
                nearest_dx, nearest_dy = dx, dy

        # === Convert the net force into a body-frame velocity command =========
        # Treat the net force as a desired world-frame velocity, clamp its
        # magnitude to the linear cap, then rotate it into the robot body frame
        # (planar_move is holonomic, so body-frame x AND y both drive the base).
        force_mag = math.hypot(fx, fy)
        if force_mag > 1e-6:
            speed = min(force_mag, self.max_linear_speed)
            vx_world = (fx / force_mag) * speed
            vy_world = (fy / force_mag) * speed
        else:
            vx_world = vy_world = 0.0

        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)
        vx_body = vx_world * cos_y + vy_world * sin_y
        vy_body = -vx_world * sin_y + vy_world * cos_y

        # Steer the robot's heading toward its direction of travel (nicer, more
        # robot-like motion than pure strafing). P-control on the heading error.
        desired_heading = math.atan2(fy, fx)
        yaw_error = wrap_angle(desired_heading - self.robot_yaw)
        wz = clamp(self.k_yaw * yaw_error, self.max_angular_speed)

        # === SAFETY FLOOR (independent of the force math) ====================
        # Hard speed caps (defensive — the SFM output above is already clamped,
        # but a future controller swapped in here might not be).
        vx_body = clamp(vx_body, self.max_linear_speed)
        vy_body = clamp(vy_body, self.max_linear_speed)
        wz = clamp(wz, self.max_angular_speed)

        # Stop-if-too-close: if ANY person is within stop_distance, zero all
        # translation (do not drive) and only rotate away from the nearest one.
        # This is a separate guard, NOT part of the force sum.
        if nearest_dist < self.stop_distance:
            vx_body = 0.0
            vy_body = 0.0
            away_heading = math.atan2(nearest_dy, nearest_dx)
            wz = clamp(self.k_yaw * wrap_angle(away_heading - self.robot_yaw),
                       self.max_angular_speed)
            self.get_logger().warn(
                f"person within {nearest_dist:.2f} m (<{self.stop_distance} m): "
                "halting translation, rotating away.",
                throttle_duration_sec=1.0,
            )

        twist.linear.x = vx_body
        twist.linear.y = vy_body
        twist.angular.z = wz
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = StubBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
