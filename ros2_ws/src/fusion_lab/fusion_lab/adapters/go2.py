"""
go2.py — calibration bridge from a Unitree Go2 (+ RealSense) rig to the
fusion_lab core.

The core's projection chain is  velo → Tr_velo_to_cam → R0_rect → P_rect.
KITTI hands those matrices over in calib.txt; the Go2 bag does not, so this
adapter builds them from two ingredients:

  1. Camera intrinsics K       — from the bag's `CameraInfo` (zero distortion).
  2. T_cam_from_odom (4x4)     — LiDAR→camera extrinsic recovered offline by
     ICP-aligning a RealSense depth back-projection against the LiDAR cloud
     (see scripts/calibrate_go2.py), stored in config/go2_calib.npz.

Frame trick: the Go2 bag publishes `cloud_deskewed` in the `odom` (world)
frame, and the core assumes a sensor-centred forward-left-up (FLU) "velo"
frame — `crop_range` limits and `norm(centroid)` distances only make sense
there. So the adapter defines a pseudo-velo frame: origin at the camera
optical centre, axes FLU-aligned with the camera's viewing direction. Clouds
are pre-transformed odom→velo by the loader, and `Tr_velo_to_cam` becomes the
constant FLU→optical axis permutation. The core stays untouched.

Pure numpy — no ROS imports, so this module works outside the container too.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..calibration import Calibration

# FLU (x fwd, y left, z up) -> camera optical (x right, y down, z fwd):
#   x_opt = -y_flu,  y_opt = -z_flu,  z_opt = x_flu
R_OPT_FROM_FLU = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


@dataclass
class Go2Rig:
    """Everything needed to feed Go2 frames to the fusion_lab core."""

    calib: Calibration          # what the core consumes
    T_velo_from_odom: np.ndarray  # (4, 4) applied to each odom-frame cloud
    img_width: int
    img_height: int

    @classmethod
    def from_npz(cls, path: str | Path) -> "Go2Rig":
        """Load config/go2_calib.npz produced by scripts/calibrate_go2.py."""
        data = np.load(str(path))
        return cls.from_matrices(
            K=data["K"],
            T_cam_from_odom=data["T_cam_from_odom"],
            img_width=int(data["width"]),
            img_height=int(data["height"]),
        )

    @classmethod
    def from_matrices(
        cls,
        K: np.ndarray,
        T_cam_from_odom: np.ndarray,
        img_width: int,
        img_height: int,
    ) -> "Go2Rig":
        # Constant pseudo-velo -> optical rotation (no translation: the velo
        # frame is centred on the camera by construction).
        Tr_velo_to_cam = np.eye(4)
        Tr_velo_to_cam[:3, :3] = R_OPT_FROM_FLU

        # odom -> velo = (velo->cam)^-1 ∘ (odom->cam)
        T_velo_from_odom = np.linalg.inv(Tr_velo_to_cam) @ T_cam_from_odom

        P_rect = np.hstack([K, np.zeros((3, 1))])  # intrinsics, no rectification
        calib = Calibration(
            P_rect=P_rect,
            R0_rect=np.eye(4),
            Tr_velo_to_cam=Tr_velo_to_cam,
        )
        return cls(
            calib=calib,
            T_velo_from_odom=T_velo_from_odom,
            img_width=img_width,
            img_height=img_height,
        )

    def cloud_to_velo(self, points_odom: np.ndarray) -> np.ndarray:
        """Transform an (N, 3+) odom-frame cloud into the pseudo-velo frame.

        Extra columns (intensity, ...) are preserved.
        """
        points_odom = strip_zero_padding(points_odom)
        xyz = points_odom[:, :3]
        homo = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
        velo = (self.T_velo_from_odom @ homo.T).T[:, :3]
        if points_odom.shape[1] > 3:
            return np.hstack([velo, points_odom[:, 3:]])
        return velo


def strip_zero_padding(points: np.ndarray) -> np.ndarray:
    """Drop the Go2's fixed-size buffer padding.

    `/utlidar/cloud_deskewed` messages carry exactly 10,000 points at
    (0, 0, 0) — zero-padding of a fixed-size buffer, located at the odom
    origin, NOT real returns. Left in, they form a degenerate blob that
    hijacks ICP/clustering. ~1.1–1.5k real points remain per scan.
    """
    keep = np.abs(points[:, :3]).sum(axis=1) > 1e-6
    return points[keep]


