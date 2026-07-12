"""
go2_bag.py — offline loader that turns a Go2 ROS 2 bag into fusion_lab
frames (mirrors the KittiFrame pattern).

Design notes, driven by the Phase-0 findings for this rig:

- The bag's sqlite storage is queried directly by time range, giving random
  access without streaming the whole 5.4 GB file. Messages are deserialized
  with rclpy, so this module needs a sourced ROS 2 environment (the KITTI
  path does not — that's why this lives in an adapter, not the core).
- One `/utlidar/cloud_deskewed` message is only a rotating ~180° slice of
  the L1 rosette (~1.3k real points), so a frame AGGREGATES all scans within
  ±window/2 of the frame time. The robot is stationary, so aggregation in
  the odom frame is exact for static structure (moving people smear by
  ~walking-speed × window).
- Each cloud message carries 10k zero-padding points; Go2Rig.cloud_to_velo
  strips them and moves the cloud into the camera-centred pseudo-velo frame
  the core expects.
- Camera + LiDAR are paired by nearest header stamp (median gap in this bag
  is ~13 ms — an ApproximateTimeSynchronizer equivalent for offline use).
"""
from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np

from .go2 import Go2Rig

COLOR_IMG = "/camera/camera/color/image_raw"
CLOUD = "/utlidar/cloud_deskewed"


@dataclass
class Go2Frame:
    """Duck-types KittiFrame for the rest of the pipeline."""

    image: np.ndarray        # (H, W, 3) uint8 RGB
    points: np.ndarray       # (N, 4) x,y,z,intensity in pseudo-velo frame
    calib: object            # fusion_lab.Calibration (from Go2Rig)
    stamp: float             # frame reference time (s, epoch) -- CHOSEN by the
                             # loader (t_start + offset), not a sensor clock
    t_img: float             # REAL sensor stamp of the color image (s, epoch)
    image_dt_ms: float       # |image stamp - frame time|
    n_scans: int             # LiDAR messages aggregated

    @property
    def height(self) -> int:
        return self.image.shape[0]

    @property
    def width(self) -> int:
        return self.image.shape[1]


class Go2BagLoader:
    """Random-access frame loader for a Go2 rosbag2 (sqlite3 storage)."""

    def __init__(self, bag_dir: str | Path, calib_npz: str | Path,
                 window: float = 0.8) -> None:
        self.rig = Go2Rig.from_npz(calib_npz)
        self.window = window
        db_path = next(Path(bag_dir).glob("*.db3"))
        self._con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
        topics = dict(self._con.execute("SELECT name, id FROM topics"))
        self._img_tid = topics[COLOR_IMG]
        self._cloud_tid = topics[CLOUD]
        t0, t1 = self._con.execute(
            "SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
        self.t_start = t0 * 1e-9
        self.t_end = t1 * 1e-9

        # Lazy ROS imports so `import fusion_lab` works without ROS.
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image, PointCloud2
        from sensor_msgs_py import point_cloud2
        self._deser = deserialize_message
        self._Image = Image
        self._PointCloud2 = PointCloud2
        self._pc2 = point_cloud2

    # ---- raw message access -------------------------------------------------
    def _rows(self, tid: int, t_lo: float, t_hi: float):
        return self._con.execute(
            "SELECT timestamp, data FROM messages "
            "WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp",
            (tid, int(t_lo * 1e9), int(t_hi * 1e9))).fetchall()

    def _image_near(self, t: float):
        """Nearest color image within ±0.25 s of t."""
        rows = self._rows(self._img_tid, t - 0.25, t + 0.25)
        if not rows:
            return None, None
        ts, raw = min(rows, key=lambda r: abs(r[0] * 1e-9 - t))
        msg = self._deser(raw, self._Image)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)          # encoding rgb8
        return img, ts * 1e-9

    def _clouds_in(self, t_lo: float, t_hi: float):
        rows = self._rows(self._cloud_tid, t_lo, t_hi)
        clouds = []
        for _, raw in rows:
            msg = self._deser(raw, self._PointCloud2)
            pts = self._pc2.read_points_numpy(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            clouds.append(pts.astype(np.float64))
        return clouds

    # ---- frames ---------------------------------------------------------------
    def frame_at(self, offset_s: float) -> Go2Frame | None:
        """Build one frame centred at bag-start + offset_s."""
        t = self.t_start + offset_s
        clouds = self._clouds_in(t - self.window / 2, t + self.window / 2)
        img, t_img = self._image_near(t)
        if img is None or not clouds:
            return None
        points_odom = np.vstack(clouds)
        points_velo = self.rig.cloud_to_velo(points_odom)
        return Go2Frame(
            image=img,
            points=points_velo,
            calib=self.rig.calib,
            stamp=t,
            t_img=t_img,
            image_dt_ms=abs(t_img - t) * 1000,
            n_scans=len(clouds),
        )

    def frames(self, start: float = 1.0, stop: float | None = None,
               step: float = 1.0) -> Iterator[tuple[float, Go2Frame]]:
        """Yield (offset_s, frame) pairs across the bag."""
        duration = self.t_end - self.t_start
        stop = duration - 1.0 if stop is None else min(stop, duration - 0.5)
        t = start
        while t <= stop:
            frame = self.frame_at(t)
            if frame is not None:
                yield t, frame
            t += step

    def close(self) -> None:
        self._con.close()
