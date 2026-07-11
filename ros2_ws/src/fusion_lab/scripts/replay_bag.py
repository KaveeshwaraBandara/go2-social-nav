#!/usr/bin/env python3
"""
replay_bag.py — drive the real-perception producer from a recorded Go2 rosbag.

    bag -> fusion core -> tracker -> Track      (no robot, no live topics)

This is the OFFLINE DRIVER, and it is what lets the IT2-FLS be validated against
real recorded perception at the desk. It is NOT a ROS node: it publishes nothing
and subscribes to nothing. ROS appears only inside `Go2BagLoader`, which imports
rclpy lazily to deserialize the recorded messages — the producer core underneath
stays framework-agnostic (robot=Foxy, dev=Humble, lab=Jazzy).

The bags are NOT stored in this repo (5.4 GB; they belong to the fusion-lab
bench). docker-compose bind-mounts them read-only at /home/dev/bags.

    ros2 run fusion_lab replay_bag.py --bag /home/dev/bags/social_fusion_20260703_161444

KNOWN DATA FACTS — visible on purpose, NOT corrected here
---------------------------------------------------------
* The deskewed cloud aggregates returns over the preceding few hundred ms, so an
  approaching person reads ~0.2-0.4 m BEHIND true position. Sensor artifact, not
  a filter defect. Correcting it here would bake a bag-specific fudge into the
  producer.
* odom-frame aggregation assumes a STATIONARY robot. Every validation bag is.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

DEFAULT_CALIB = "/home/dev/ros2_ws/src/fusion_lab/config/go2_calib.npz"
DEFAULT_WEIGHTS = os.environ.get("GO2_YOLO_WEIGHTS", "yolov8n.pt")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="rosbag2 directory (contains *.db3)")
    ap.add_argument("--calib", default=DEFAULT_CALIB, help="rig calibration .npz")
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS, help="YOLO weights")
    ap.add_argument("--conf", type=float, default=0.4,
                    help="YOLO confidence threshold (0.4 = the bench-validated value)")
    ap.add_argument("--start", type=float, default=1.0, help="first frame, s after bag start")
    ap.add_argument("--stop", type=float, default=None, help="last frame, s after bag start")
    ap.add_argument("--step", type=float, default=0.2, help="seconds between frames (5 Hz)")
    ap.add_argument("--window", type=float, default=0.8, help="LiDAR aggregation window (s)")
    ap.add_argument("--quiet", action="store_true", help="only print the summary")
    args = ap.parse_args(argv)

    # Imported here, not at module scope: a --help should not pay for torch.
    from fusion_lab import YoloDetector
    from fusion_lab.adapters import Go2BagLoader, Go2PerceptionProducer

    bag = Path(args.bag)
    if not bag.is_dir() or not any(bag.glob("*.db3")):
        print(f"error: no rosbag2 (*.db3) at {bag}", file=sys.stderr)
        return 2

    loader = Go2BagLoader(bag, args.calib, window=args.window)
    producer = Go2PerceptionProducer.from_calib(
        args.calib, detector=YoloDetector(args.weights, conf=args.conf))

    duration = loader.t_end - loader.t_start
    print(f"bag       : {bag.name}")
    print(f"duration  : {duration:.1f} s   (replaying {args.start:.1f}s -> "
          f"{args.stop if args.stop is not None else duration - 1.0:.1f}s @ {1/args.step:.0f} Hz)")
    print(f"calib     : {args.calib}")
    print(f"weights   : {args.weights}")
    print(f"producer  : stationary-robot assumption = {producer.assumes_stationary_robot}")
    print()
    if not args.quiet:
        print(f"{'t[s]':>6} {'scans':>5} {'det':>3} {'trk':>3} │ "
              f"{'id':>3} {'x':>6} {'y':>6} {'vx':>6} {'vy':>6} {'spd':>5} "
              f"{'pos_std':>7} {'vel_std':>7} {'age':>3} {'coast':>5}")
        print("─" * 100)

    n_frames = 0
    n_empty = 0
    ids_seen: set[int] = set()
    t_wall = time.time()

    for offset, frame in loader.frames(args.start, args.stop, args.step):
        detections = producer.detect(frame)
        tracks = producer.track(detections, float(frame.t_img))
        n_frames += 1
        ids_seen.update(t.id for t in tracks)
        if not tracks:
            n_empty += 1

        if args.quiet:
            continue
        if not tracks:
            print(f"{offset:>6.1f} {frame.n_scans:>5} {len(detections):>3} {0:>3} │ "
                  f"  (no confirmed tracks)")
            continue
        for i, tr in enumerate(tracks):
            head = (f"{offset:>6.1f} {frame.n_scans:>5} {len(detections):>3} "
                    f"{len(tracks):>3} │ ") if i == 0 else " " * 24 + "│ "
            print(f"{head}{tr.id:>3} {tr.x:>6.2f} {tr.y:>6.2f} {tr.vx:>6.2f} "
                  f"{tr.vy:>6.2f} {tr.speed:>5.2f} {tr.pos_std:>7.3f} "
                  f"{tr.vel_std:>7.3f} {tr.age:>3} {str(tr.is_coasting):>5}")

    wall = time.time() - t_wall
    loader.close()

    print()
    print(f"frames replayed     : {n_frames}  ({wall:.1f}s wall, "
          f"{n_frames / wall if wall else 0:.1f} frame/s)")
    print(f"frames with no track: {n_empty}")
    print(f"distinct track ids  : {len(ids_seen)}  {sorted(ids_seen)}")
    print()
    print("Tracks above are the canonical `tracking.Track` — the SAME type the "
          "oracle/HuNav\nproducers emit and the IT2-FLS consumes. Nothing here "
          "knows a controller exists.")
    return 0 if n_frames else 1


if __name__ == "__main__":
    sys.exit(main())
