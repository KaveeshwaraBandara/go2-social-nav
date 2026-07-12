#!/usr/bin/env python3
"""
visualize_replay.py — render what the perception producer sees, so a human can
CHECK it rather than trust a table of numbers.

Per frame, side by side:

  LEFT   the real camera image, with the LiDAR returns that survived crop +
         ground-removal projected back onto it (coloured by depth), the YOLO box,
         and the emitted Track: id, speed, and whether it is COASTING.
  RIGHT  bird's-eye view in the ODOM frame: the Go2 at the origin, each track's
         position and trail, a velocity arrow, and the 1-sigma `pos_std` circle —
         the uncertainty the IT2-FLS turns into its footprint of uncertainty.
         The circle GROWS while a track coasts; that is the point of drawing it.

A DEBUG / figure tool, not part of the producer. Nothing in the pipeline depends
on it, and it emits no ROS topics.

    ros2 run fusion_lab visualize_replay.py \
        --bag /home/dev/bags/social_fusion_20260703_161444 \
        --start 2 --stop 7 --out outputs/perception_replay_early

Rendered output is reproducible from the bag, so `outputs/` is gitignored.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

DEFAULT_CALIB = "/home/dev/ros2_ws/src/fusion_lab/config/go2_calib.npz"
DEFAULT_WEIGHTS = os.environ.get("GO2_YOLO_WEIGHTS", "yolov8n.pt")

BEV_W, BEV_H = 640, 720
BEV_RANGE_M = 8.0          # metres shown to each side

# Stable per-id colours (BGR).
PALETTE = [(66, 135, 245), (245, 176, 66), (66, 245, 141), (245, 66, 197),
           (245, 66, 66), (66, 245, 236), (168, 245, 66), (150, 120, 255)]


def colour(track_id: int):
    return PALETTE[track_id % len(PALETTE)]


def bev_px(x: float, y: float):
    """odom (x forward, y left) -> BEV pixel. Robot bottom-centre, +x up-screen."""
    u = int(BEV_W / 2 - y / BEV_RANGE_M * (BEV_W / 2))
    v = int(BEV_H - x / (2 * BEV_RANGE_M) * BEV_H)
    return u, v


def draw_bev(tracks, trails, offset):
    bev = np.full((BEV_H, BEV_W, 3), 24, np.uint8)

    for r in range(2, int(BEV_RANGE_M * 2) + 1, 2):
        radius = int(r / (2 * BEV_RANGE_M) * BEV_H)
        cv2.circle(bev, bev_px(0, 0), radius, (55, 55, 55), 1)
        cv2.putText(bev, f"{r}m", (BEV_W // 2 + 4, BEV_H - radius),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (90, 90, 90), 1)

    cv2.drawMarker(bev, bev_px(0, 0), (255, 255, 255), cv2.MARKER_TRIANGLE_UP, 16, 2)
    cv2.putText(bev, "Go2 (odom origin)", (BEV_W // 2 - 60, BEV_H - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    cv2.putText(bev, f"t = {offset:5.1f}s   tracks: {len(tracks)}", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)

    for tr in tracks:
        col = colour(tr.id)
        trails.setdefault(tr.id, []).append((tr.x, tr.y))
        for a, b in zip(trails[tr.id], trails[tr.id][1:]):
            cv2.line(bev, bev_px(*a), bev_px(*b), col, 1)

        p = bev_px(tr.x, tr.y)
        cv2.circle(bev, p, max(3, int(tr.pos_std / (2 * BEV_RANGE_M) * BEV_H)),
                   col, 1, cv2.LINE_AA)
        cv2.circle(bev, p, 5, col, 1 if tr.is_coasting else -1)
        if tr.speed > 0.05:
            cv2.arrowedLine(bev, p, bev_px(tr.x + tr.vx, tr.y + tr.vy), col, 2,
                            tipLength=0.25)
        tag = f"#{tr.id} {tr.speed:.1f}m/s" + ("  COAST" if tr.is_coasting else "")
        cv2.putText(bev, tag, (p[0] + 10, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, col, 1, cv2.LINE_AA)
    return bev


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--calib", default=DEFAULT_CALIB)
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--start", type=float, default=2.0)
    ap.add_argument("--stop", type=float, default=20.0)
    ap.add_argument("--step", type=float, default=0.2)
    ap.add_argument("--out", default="outputs/perception_replay")
    args = ap.parse_args(argv)

    from fusion_lab import YoloDetector
    from fusion_lab.adapters import Go2BagLoader, Go2PerceptionProducer
    from fusion_lab.lidar import crop_range, remove_ground_ransac

    os.makedirs(args.out, exist_ok=True)

    loader = Go2BagLoader(args.bag, args.calib, window=0.8)
    prod = Go2PerceptionProducer.from_calib(
        args.calib, detector=YoloDetector(args.weights, conf=args.conf))

    trails: dict[int, list] = {}
    writer = None
    n = 0

    for offset, frame in loader.frames(args.start, args.stop, args.step):
        img = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR).copy()

        # Draw the cloud the fusion ACTUALLY uses — after crop + ground removal —
        # not the raw one. If the floor were still in it, it would show here.
        pts = crop_range(frame.points, x_range=prod.crop_x, y_range=prod.crop_y,
                         z_range=prod.crop_z)
        nonground, _ = remove_ground_ransac(
            pts, distance_threshold=prod.ground_distance_threshold)
        uv, depth, _ = prod.rig.calib.points_in_image_fov(
            nonground, prod.rig.img_width, prod.rig.img_height)
        for (u, v), d in zip(uv.astype(int), depth):
            c = int(np.clip(255 - d * 20, 40, 255))
            cv2.circle(img, (u, v), 1, (c, 90, 255 - c), -1)

        for box in prod.detector.detect(frame.image):
            if box.label != "person":
                continue
            x1, y1, x2, y2 = box.bbox.astype(int)
            cv2.rectangle(img, (x1, y1), (x2, y2), (200, 200, 200), 1)

        tracks = prod.step(frame)

        # Project each track's odom position back into the image, so the 3D result
        # can be eyeballed against the person it claims to be.
        for tr in tracks:
            col = colour(tr.id)
            p_velo = (prod.rig.T_velo_from_odom
                      @ np.array([tr.x, tr.y, tr.z, 1.0]))[:3]
            uv_t, _, mask = prod.rig.calib.points_in_image_fov(
                p_velo.reshape(1, 3), prod.rig.img_width, prod.rig.img_height)
            if not mask.any():
                continue
            u, v = uv_t[0].astype(int)
            cv2.circle(img, (u, v), 7, col, 2)
            cv2.putText(img, f"#{tr.id} {tr.speed:.1f}m/s"
                        + ("  COASTING" if tr.is_coasting else ""),
                        (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

        cv2.putText(img, f"t={offset:5.1f}s   tracks:{len(tracks)}", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        bev = draw_bev(tracks, trails, offset)
        left = cv2.resize(img, (int(BEV_H * frame.width / frame.height), BEV_H))
        canvas = np.hstack([left, bev])

        cv2.imwrite(f"{args.out}/frame_{offset:06.2f}.png", canvas)
        if writer is None:
            writer = cv2.VideoWriter(f"{args.out}/replay.mp4",
                                     cv2.VideoWriter_fourcc(*"mp4v"), 5.0,
                                     (canvas.shape[1], canvas.shape[0]))
        writer.write(canvas)
        n += 1

    if writer is not None:
        writer.release()
    loader.close()
    print(f"wrote {n} annotated frames + replay.mp4 -> {args.out}/")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
