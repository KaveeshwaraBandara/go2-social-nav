#!/usr/bin/env python3
"""Phase 7: it2fls_brain -- the Interval Type-2 Fuzzy Logic social controller.

This node is the REAL controller the whole project has been scaffolded for. It is
a DROP-IN REPLACEMENT for stub_brain (Phase 5): identical interface, identical
safety floor, so the benchmark harness (Phase 6) grades it against stub/DWA/TEB
with zero changes.

Interface (the permanent control contract):
  IN   /people  people_msgs/People    ground-truth perception (frame: map);
                                       per agent: position + velocity.
  IN   /odom    nav_msgs/Odometry     the robot's own pose from planar_move.
  OUT  /cmd_vel geometry_msgs/Twist   body-frame velocity command @ 20 Hz.

WHAT IS DIFFERENT FROM stub_brain
---------------------------------
stub_brain used a fixed Social Force Model: every person got the SAME repulsion
law regardless of context. This node replaces that fixed law with the IT2-FLS
(`it2_fls.SocialFLS`): for each person it computes a distance and a closing speed
and asks the fuzzy system for a CAUTION in [0, 1]. Caution then drives, per person:

  * how hard to steer away          (repulsion gain  ~ caution)
  * how much to slow down overall   (speed cap       ~ 1 - max caution)

So a fast-approaching person 0.5 m away produces strong avoidance; a person 4 m
away walking off produces almost none -- a smooth, context-sensitive separation
distance. That is Layer 1 of the WSO2 proposal, made concrete.

JERK-BOUNDED OUTPUT (the proposal's formal smoothness claim)
------------------------------------------------------------
The proposal promises jerk-bounded motion "as a formal mathematical property".
We deliver that two ways, both real and both visible in the benchmark's jerk
metric:
  1. The FLS output surface is continuous (see it2_fls.py), so caution -- and
     therefore the command -- never jumps for a small change in the scene.
  2. An explicit ACCELERATION + JERK LIMITER on the final command: the change in
     acceleration per tick is hard-clamped to j_max*dt, and acceleration to a_max.
     This is a provable bound independent of what the fuzzy layer asks for.

The SAFETY FLOOR (hard caps + stop-if-too-close) is identical to stub_brain and,
like there, is applied last and independently -- safety is never at the mercy of
the fuzzy maths.
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from people_msgs.msg import People

# it2_fls.py is installed into the SAME directory as this script (both go into
# lib/go2_brain via the CMake PROGRAMS install), so add our own dir to the path
# and import it. This mirrors the gesture package's "avoid colcon import-path
# pain" approach without having to inline the whole engine.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from it2_fls import SocialFLS  # noqa: E402


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def rate_limit(v_des, v_prev, a_prev, dt, a_max, j_max):
    """Acceleration + jerk limited step. Returns (v_out, a_out).

    Bounds |da/dt| <= j_max (the jerk bound) and |a| <= a_max, then integrates
    once to get the new velocity. Applied per scalar channel.
    """
    a_des = (v_des - v_prev) / dt
    # Jerk limit: how fast acceleration itself is allowed to change.
    a_lim = a_prev + clamp(a_des - a_prev, j_max * dt)
    # Acceleration limit.
    a_lim = clamp(a_lim, a_max)
    v_out = v_prev + a_lim * dt
    return v_out, a_lim


class IT2FLSBrain(Node):
    def __init__(self):
        super().__init__("it2fls_brain")

        # --- Goal (map frame). Overridden by launch args, like stub_brain. ----
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 4.0)
        self.declare_parameter("goal_tolerance", 0.4)
        self.declare_parameter("control_rate", 20.0)

        # --- Attraction + fuzzy-driven repulsion ------------------------------
        self.declare_parameter("k_attractive", 1.0)
        self.declare_parameter("k_repulsive", 2.5)   # repulsion is SCALED by caution
        self.declare_parameter("repulsion_range", 1.2)
        self.declare_parameter("person_cutoff", 5.0)  # ignore people beyond this
        self.declare_parameter("caution_slowdown", 0.7)  # how much max-caution cuts speed

        # --- Speed clamps + heading -------------------------------------------
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("k_yaw", 1.5)

        # --- Jerk / acceleration limiter (the formal smoothness) --------------
        self.declare_parameter("max_linear_accel", 0.8)   # [m/s^2]
        self.declare_parameter("max_linear_jerk", 2.5)    # [m/s^3]
        self.declare_parameter("max_angular_accel", 2.5)  # [rad/s^2]
        self.declare_parameter("max_angular_jerk", 8.0)   # [rad/s^3]

        # --- Safety floor (identical semantics to stub_brain) -----------------
        self.declare_parameter("stop_distance", 0.8)

        g = self.get_parameter
        self.goal_x = g("goal_x").value
        self.goal_y = g("goal_y").value
        self.goal_tolerance = g("goal_tolerance").value
        self.control_rate = g("control_rate").value
        self.k_attractive = g("k_attractive").value
        self.k_repulsive = g("k_repulsive").value
        self.repulsion_range = g("repulsion_range").value
        self.person_cutoff = g("person_cutoff").value
        self.caution_slowdown = g("caution_slowdown").value
        self.max_linear_speed = g("max_linear_speed").value
        self.max_angular_speed = g("max_angular_speed").value
        self.k_yaw = g("k_yaw").value
        self.max_linear_accel = g("max_linear_accel").value
        self.max_linear_jerk = g("max_linear_jerk").value
        self.max_angular_accel = g("max_angular_accel").value
        self.max_angular_jerk = g("max_angular_jerk").value
        self.stop_distance = g("stop_distance").value

        # --- The fuzzy engine -------------------------------------------------
        self.fls = SocialFLS()

        # --- State caches -----------------------------------------------------
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = 0.0
        self.robot_vx_world = 0.0   # finite-differenced from /odom (map frame)
        self.robot_vy_world = 0.0
        self._last_x = None
        self._last_y = None
        self._last_t = None
        self.people = []

        # Limiter memory (per output channel): previous velocity + acceleration.
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_wz = 0.0
        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_awz = 0.0

        # --- ROS I/O ----------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.create_subscription(People, "people", self.people_cb, 10)

        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self.control_tick)
        self._goal_reached_logged = False

        self.get_logger().info(
            f"it2fls_brain up: goal=({self.goal_x:.2f}, {self.goal_y:.2f}), "
            f"{self.control_rate:.0f} Hz, v_max={self.max_linear_speed} m/s, "
            f"a_max={self.max_linear_accel} m/s^2, j_max={self.max_linear_jerk} m/s^3, "
            f"stop_distance={self.stop_distance} m"
        )

    # --- Callbacks: cache only -------------------------------------------------
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Finite-difference the map-frame velocity (robust to frame conventions;
        # needed for the closing-speed input to the FLS).
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._last_t is not None:
            dt = t - self._last_t
            if dt > 1e-3:
                self.robot_vx_world = (x - self._last_x) / dt
                self.robot_vy_world = (y - self._last_y) / dt
        self._last_x, self._last_y, self._last_t = x, y, t
        self.robot_x, self.robot_y = x, y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def people_cb(self, msg):
        self.people = msg.people

    # --- Control loop ----------------------------------------------------------
    def control_tick(self):
        if self.robot_x is None:
            return

        twist = Twist()

        to_goal_x = self.goal_x - self.robot_x
        to_goal_y = self.goal_y - self.robot_y
        dist_to_goal = math.hypot(to_goal_x, to_goal_y)

        if dist_to_goal < self.goal_tolerance:
            if not self._goal_reached_logged:
                self.get_logger().info("goal reached; holding position.")
                self._goal_reached_logged = True
            self._publish_zero()
            return
        self._goal_reached_logged = False

        # === 1. Attraction toward the goal (map frame) =======================
        fx = self.k_attractive * (to_goal_x / dist_to_goal)
        fy = self.k_attractive * (to_goal_y / dist_to_goal)

        # === 2. Fuzzy, per-person repulsion ==================================
        nearest_dist = float("inf")
        nearest_dx = nearest_dy = 0.0
        max_caution = 0.0

        for person in self.people:
            # Vector person->robot (repulsion pushes the robot along it).
            dx = self.robot_x - person.position.x
            dy = self.robot_y - person.position.y
            dist = math.hypot(dx, dy)
            if dist < 1e-3 or dist > self.person_cutoff:
                continue

            # Closing speed: rate at which the gap shrinks (+ = approaching).
            # unit vector robot->person:
            ux = -dx / dist
            uy = -dy / dist
            rel_vx = self.robot_vx_world - person.velocity.x
            rel_vy = self.robot_vy_world - person.velocity.y
            closing_speed = ux * rel_vx + uy * rel_vy

            # --- ASK THE FUZZY SYSTEM --------------------------------------
            caution, _uncertainty = self.fls.infer(dist, closing_speed)
            max_caution = max(max_caution, caution)

            # Repulsion magnitude is SCALED BY CAUTION (context-sensitive), with
            # a smooth distance falloff. Direction: away from the person.
            magnitude = self.k_repulsive * caution * math.exp(-dist / self.repulsion_range)
            fx += magnitude * (dx / dist)
            fy += magnitude * (dy / dist)

            if dist < nearest_dist:
                nearest_dist = dist
                nearest_dx, nearest_dy = dx, dy

        # === 3. Net force -> world velocity, speed capped by caution =========
        # High overall caution lowers the speed cap: the robot naturally slows in
        # tense, crowded, fast-closing situations (helps both proxemics + jerk).
        speed_cap = self.max_linear_speed * (1.0 - self.caution_slowdown * max_caution)
        speed_cap = max(0.05, speed_cap)  # never fully freeze from caution alone

        force_mag = math.hypot(fx, fy)
        if force_mag > 1e-6:
            speed = min(force_mag, speed_cap)
            vx_world = (fx / force_mag) * speed
            vy_world = (fy / force_mag) * speed
        else:
            vx_world = vy_world = 0.0

        # Rotate world velocity into the (holonomic) body frame.
        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)
        vx_body = vx_world * cos_y + vy_world * sin_y
        vy_body = -vx_world * sin_y + vy_world * cos_y

        # Steer heading toward travel direction (P-control), same as stub.
        desired_heading = math.atan2(fy, fx)
        yaw_error = wrap_angle(desired_heading - self.robot_yaw)
        wz = clamp(self.k_yaw * yaw_error, self.max_angular_speed)

        # === 4. JERK + ACCELERATION LIMITER (formal smoothness) ==============
        vx_body, self.prev_ax = rate_limit(
            vx_body, self.prev_vx, self.prev_ax, self.dt,
            self.max_linear_accel, self.max_linear_jerk)
        vy_body, self.prev_ay = rate_limit(
            vy_body, self.prev_vy, self.prev_ay, self.dt,
            self.max_linear_accel, self.max_linear_jerk)
        wz, self.prev_awz = rate_limit(
            wz, self.prev_wz, self.prev_awz, self.dt,
            self.max_angular_accel, self.max_angular_jerk)

        # === 5. SAFETY FLOOR (independent, hard, applied last) ===============
        vx_body = clamp(vx_body, self.max_linear_speed)
        vy_body = clamp(vy_body, self.max_linear_speed)
        wz = clamp(wz, self.max_angular_speed)

        if nearest_dist < self.stop_distance:
            # Hard stop-if-too-close: zero translation, rotate away from nearest.
            vx_body = vy_body = 0.0
            away_heading = math.atan2(nearest_dy, nearest_dx)
            wz = clamp(self.k_yaw * wrap_angle(away_heading - self.robot_yaw),
                       self.max_angular_speed)
            self.get_logger().warn(
                f"person within {nearest_dist:.2f} m (<{self.stop_distance} m): "
                "halting translation, rotating away.",
                throttle_duration_sec=1.0,
            )

        # Remember what we actually commanded (the limiter integrates from here).
        self.prev_vx, self.prev_vy, self.prev_wz = vx_body, vy_body, wz

        twist.linear.x = vx_body
        twist.linear.y = vy_body
        twist.angular.z = wz
        self.cmd_pub.publish(twist)

    def _publish_zero(self):
        """Stop cleanly and reset the limiter memory so the next start is smooth."""
        self.cmd_pub.publish(Twist())
        self.prev_vx = self.prev_vy = self.prev_wz = 0.0
        self.prev_ax = self.prev_ay = self.prev_awz = 0.0


def main():
    rclpy.init()
    node = IT2FLSBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
