#!/usr/bin/env python3
"""Phase 6b: bench_runner -- the benchmark harness orchestrator + logger.

This is the REAL deliverable of Phase 6: a controller-agnostic harness that runs
ANY /cmd_vel-producing brain (stub_brain, Nav2 DWA/TEB, the future IT2-FLS) through
identical scenarios and logs IDENTICAL metrics, so the social-nav comparison is valid.

What it does for one run (one controller x scenario x run_id):
  1. Waits for the sim (/odom) and the evaluator's /hunav_start_recording service.
  2. After a short settle, calls /hunav_start_recording (experiment_tag, robot_goal,
     run_id) -> hunav_evaluator starts recording the social/proxemic metrics.
  3. Records the robot trajectory from /odom and detects goal-reached / timeout.
  4. On finish, calls /hunav_stop_recording -> the evaluator computes + writes its CSV
     row (keyed by experiment_tag + run_id).
  5. Computes our OWN trajectory JERK from /odom -- resampled to a fixed grid, over the
     full planar (x, y) motion -- the SAME way for every controller (hunav's jerk uses
     only scalar speed + a non-standard formula, so it isn't comparable for our
     holonomic base). Also computes our own path length / time-to-goal / success.
  6. MERGES our metrics with the evaluator's row into one per-run record, appends the
     master results/benchmark.csv, and dumps the raw trajectory for transparency.
  7. Shuts down (so a batch script can sequence runs).

Why /odom for jerk: with the kinematic planar_move base /odom closely tracks the
command, and sampling /odom on a fixed grid gives one identical computation regardless
of each controller's /cmd_vel publish rate -- the comparison stays apples-to-apples.
"""
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from hunav_msgs.srv import StartEvaluation
from nav2_msgs.action import NavigateToPose


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class BenchRunner(Node):
    def __init__(self):
        super().__init__("bench_runner")

        # --- Run identity -----------------------------------------------------
        self.controller = self.declare_parameter("controller", "stub").value
        self.scenario = self.declare_parameter("scenario", "head_on").value
        self.run_id = int(self.declare_parameter("run_id", 0).value)

        # --- Goal + success criteria -----------------------------------------
        self.goal_x = float(self.declare_parameter("goal_x", 0.0).value)
        self.goal_y = float(self.declare_parameter("goal_y", 4.0).value)
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.5).value)

        # --- Timing -----------------------------------------------------------
        self.settle_time = float(self.declare_parameter("settle_time", 4.0).value)
        self.timeout = float(self.declare_parameter("timeout", 90.0).value)
        self.jerk_rate = float(self.declare_parameter("jerk_rate", 50.0).value)

        # Nav2 controllers (DWA/TEB) need the goal sent via the navigate_to_pose
        # action; controllers that take the goal as a param (stub) leave this false.
        self.nav2_goal = bool(self.declare_parameter("nav2_goal", False).value)
        self.nav_goal_sent = False

        # --- Output -----------------------------------------------------------
        default_results = os.path.expanduser("~/ros2_ws/results")
        self.result_dir = os.path.expanduser(
            self.declare_parameter("result_dir", default_results).value
        )
        self.experiment_tag = f"{self.controller}_{self.scenario}"

        # --- Trajectory record: (t, x, y, yaw, vx_body, vy_body, wz) ----------
        self.traj = []
        self.latest = None  # latest (t, x, y)

        # --- State machine ----------------------------------------------------
        self.state = "init"
        self.done = False         # set when the run is logged -> main loop exits
        self.t_settle0 = None
        self.t_start = None       # sim time recording started
        self.success = False
        self.end_reason = ""
        self.start_future = None
        self.stop_future = None

        # --- ROS I/O ----------------------------------------------------------
        self.create_subscription(Odometry, "odom", self.odom_cb, 50)
        self.start_cli = self.create_client(StartEvaluation, "hunav_start_recording")
        self.stop_cli = self.create_client(Empty, "hunav_stop_recording")
        self.nav_ac = ActionClient(self, NavigateToPose, "navigate_to_pose") \
            if self.nav2_goal else None
        self.timer = self.create_timer(0.1, self.control_tick)  # 10 Hz state machine

        self.get_logger().info(
            f"bench_runner: {self.experiment_tag} run {self.run_id}, "
            f"goal=({self.goal_x:.2f},{self.goal_y:.2f}) tol={self.goal_tolerance} "
            f"timeout={self.timeout}s -> {self.result_dir}"
        )

    # --- record every odom sample (sim-time stamped) -------------------------
    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest = (t, x, y)
        if self.state == "recording":
            self.traj.append((t, x, y, yaw,
                              msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y,
                              msg.twist.twist.angular.z))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- state machine -------------------------------------------------------
    def control_tick(self):
        if self.state == "init":
            if self.latest is not None and self.start_cli.service_is_ready():
                self.t_settle0 = self.now()
                self.state = "settle"
                self.get_logger().info("sim + evaluator ready; settling...")

        elif self.state == "settle":
            if self.now() - self.t_settle0 >= self.settle_time:
                req = StartEvaluation.Request()
                req.experiment_tag = self.experiment_tag
                req.run_id = self.run_id
                g = PoseStamped()
                g.header.frame_id = "map"
                g.pose.position.x = self.goal_x
                g.pose.position.y = self.goal_y
                g.pose.orientation.w = 1.0
                req.robot_goal = g
                self.start_future = self.start_cli.call_async(req)
                self.state = "starting"

        elif self.state == "starting":
            if self.start_future.done():
                self.t_start = self.now()
                self.traj.clear()
                self.state = "recording"
                self.get_logger().info("RECORDING started.")

        elif self.state == "recording":
            # Nav2 controllers: send the goal once, after recording has started.
            if self.nav2_goal and not self.nav_goal_sent and self.nav_ac.server_is_ready():
                self._send_nav2_goal()
            elapsed = self.now() - self.t_start
            _, x, y = self.latest
            if math.hypot(self.goal_x - x, self.goal_y - y) < self.goal_tolerance:
                self.success, self.end_reason = True, "goal_reached"
                self._stop()
            elif elapsed >= self.timeout:
                self.success, self.end_reason = False, "timeout"
                self._stop()

        elif self.state == "stopping":
            if self.stop_future.done():
                self.state = "finalize"

        elif self.state == "finalize":
            self.finalize()
            self.state = "done"
            self.get_logger().info("RUN COMPLETE.")
            self.timer.cancel()
            self.done = True  # main loop sees this and exits the process cleanly

    def _send_nav2_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.w = 1.0
        self.nav_ac.send_goal_async(goal)
        self.nav_goal_sent = True
        self.get_logger().info(f"sent Nav2 goal ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def _stop(self):
        self.get_logger().info(f"stopping: {self.end_reason}")
        self.stop_future = self.stop_cli.call_async(Empty.Request())
        self.state = "stopping"

    # --- jerk + own metrics from the recorded trajectory ---------------------
    def compute_jerk(self):
        """Resample the (x, y) path to a uniform grid and compute jerk = d3p/dt3.

        Done over the FULL planar position (captures holonomic x/y motion), on a
        fixed-rate grid so the computation is identical for every controller. Uses
        numpy central differences. Returns mean/rms/max/p95 |jerk| in m/s^3.
        """
        if len(self.traj) < 8:
            return {}
        arr = np.array(self.traj, dtype=float)
        t = arr[:, 0] - arr[0, 0]
        x, y = arr[:, 1], arr[:, 2]
        dur = t[-1]
        if dur <= 0:
            return {}
        # uniform timeline at jerk_rate, interpolate the path onto it
        n = max(8, int(dur * self.jerk_rate))
        tu = np.linspace(0.0, dur, n)
        dt = tu[1] - tu[0]
        xu = np.interp(tu, t, x)
        yu = np.interp(tu, t, y)
        vx, vy = np.gradient(xu, dt), np.gradient(yu, dt)
        ax, ay = np.gradient(vx, dt), np.gradient(vy, dt)
        jx, jy = np.gradient(ax, dt), np.gradient(ay, dt)
        jmag = np.hypot(jx, jy)
        amag = np.hypot(ax, ay)
        return {
            "jerk_mean": float(np.mean(jmag)),
            "jerk_rms": float(np.sqrt(np.mean(jmag ** 2))),
            "jerk_max": float(np.max(jmag)),
            "jerk_p95": float(np.percentile(jmag, 95)),
            "accel_mean": float(np.mean(amag)),
            "jerk_sample_rate": self.jerk_rate,
            "jerk_n_samples": int(n),
        }

    def own_metrics(self):
        m = {
            "controller": self.controller,
            "scenario": self.scenario,
            "run_id": self.run_id,
            "success": int(self.success),
            "end_reason": self.end_reason,
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
        }
        if len(self.traj) >= 2:
            arr = np.array(self.traj, dtype=float)
            x, y, t = arr[:, 1], arr[:, 2], arr[:, 0]
            seg = np.hypot(np.diff(x), np.diff(y))
            m["our_path_length"] = float(np.sum(seg))
            m["our_time_to_goal"] = float(t[-1] - t[0])
        m.update(self.compute_jerk())
        return m

    # --- merge with the evaluator's row + write everything -------------------
    def finalize(self):
        import csv
        import json

        os.makedirs(self.result_dir, exist_ok=True)
        os.makedirs(os.path.join(self.result_dir, "runs"), exist_ok=True)
        os.makedirs(os.path.join(self.result_dir, "trajectories"), exist_ok=True)
        base = f"{self.experiment_tag}_run{self.run_id}"

        record = self.own_metrics()

        # raw trajectory dump (transparency: jerk can be recomputed from this)
        traj_path = os.path.join(self.result_dir, "trajectories", base + "_traj.csv")
        with open(traj_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "yaw", "vx_body", "vy_body", "wz"])
            w.writerows(self.traj)

        # pull the evaluator's just-written row and merge it in
        eval_csv = os.path.join(self.result_dir, "hunav_eval.csv")
        record.update(self._read_eval_row(eval_csv))

        # per-run JSON
        with open(os.path.join(self.result_dir, "runs", base + ".json"), "w") as f:
            json.dump(record, f, indent=2)

        # append to the master comparison table (union of columns, stable order)
        master = os.path.join(self.result_dir, "benchmark.csv")
        self._append_master(master, record)

        self.get_logger().info(
            f"logged: success={record['success']} reason={record['end_reason']} "
            f"jerk_mean={record.get('jerk_mean', float('nan')):.3f} "
            f"path={record.get('our_path_length', float('nan')):.2f}m "
            f"t={record.get('our_time_to_goal', float('nan')):.1f}s -> {master}"
        )

    def _read_eval_row(self, eval_csv):
        """Return the hunav_evaluator metrics for THIS run as a flat dict (eval_* keys)."""
        if not os.path.exists(eval_csv):
            self.get_logger().warn(f"evaluator CSV not found: {eval_csv}")
            return {}
        try:
            import pandas as pd
            df = pd.read_csv(eval_csv)
            # rows are indexed by experiment_tag (first column); run_id is a column
            tagcol = df.columns[0]
            sel = df[(df[tagcol] == self.experiment_tag) & (df["run_id"] == self.run_id)]
            if sel.empty:
                self.get_logger().warn("no matching evaluator row for this run")
                return {}
            row = sel.iloc[-1].to_dict()
            row.pop("run_id", None)
            row.pop(tagcol, None)
            return {f"eval_{k}": v for k, v in row.items()}
        except Exception as e:  # noqa: BLE001 - never lose our own metrics over a merge issue
            self.get_logger().warn(f"could not merge evaluator row: {e}")
            return {}

    def _append_master(self, master, record):
        import csv

        existing_cols = []
        rows = []
        if os.path.exists(master):
            with open(master, newline="") as f:
                r = csv.DictReader(f)
                existing_cols = r.fieldnames or []
                rows = list(r)
        # union of columns, preserving prior order then appending any new ones
        cols = list(existing_cols)
        for k in record:
            if k not in cols:
                cols.append(k)
        with open(master, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for row in rows:
                w.writerow(row)
            w.writerow(record)


def main():
    rclpy.init()
    node = BenchRunner()
    try:
        # Spin until the run is logged, then return so the process exits cleanly
        # (the launch tears down on our exit, letting a batch script sequence runs).
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
