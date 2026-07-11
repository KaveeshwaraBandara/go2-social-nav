"""
calibration.py — coordinate frames, intrinsics/extrinsics, and projection.

This is the heart of LiDAR-camera fusion. Master this file and the rest of the
project is bookkeeping.

Frames involved (KITTI convention used as the reference):
    - velo   : LiDAR (Velodyne) frame.  x=forward, y=left, z=up
    - cam0   : reference (left grayscale) camera frame. x=right, y=down, z=fwd
    - cam2   : left color camera (the one images usually come from)
    - image  : 2D pixel coordinates (u, v)

The projection chain for a LiDAR point X_velo -> pixel (u, v):

    X_cam   = Tr_velo_to_cam @ [X_velo; 1]      # 3D LiDAR -> 3D camera
    X_rect  = R0_rect @ X_cam                    # rectify (stereo alignment)
    x_img   = P_rect @ [X_rect; 1]               # 3x4 projection -> homogeneous
    (u, v)  = (x_img[0]/x_img[2], x_img[1]/x_img[2])

P_rect already folds in the camera intrinsics (focal length, principal point)
plus the rectified projection, which is why KITTI is such a good teacher: the
calib file hands you every matrix pre-computed.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Calibration:
    """Holds the matrices needed to move between LiDAR, camera, and image."""

    P_rect: np.ndarray       # (3, 4) rectified projection (cam2)
    R0_rect: np.ndarray      # (4, 4) rectifying rotation, homogeneous
    Tr_velo_to_cam: np.ndarray  # (4, 4) LiDAR -> reference cam, homogeneous

    # ---- Loading ------------------------------------------------------------
    @classmethod
    def from_kitti(cls, calib_path: str | Path) -> "Calibration":
        """Parse a KITTI calibration .txt file.

        Works with the odometry-style 'calib.txt' (keys P0..P3, Tr) and the
        raw/object style (keys P2, R0_rect, Tr_velo_to_cam). We normalize both.
        """
        raw: dict[str, np.ndarray] = {}
        for line in Path(calib_path).read_text().strip().splitlines():
            if ":" not in line:
                continue
            key, vals = line.split(":", 1)
            raw[key.strip()] = np.array([float(x) for x in vals.split()])

        # Projection for cam2 (left color). Fall back to P2 then P0.
        p = raw.get("P2", raw.get("P0"))
        P_rect = p.reshape(3, 4)

        # Rectifying rotation -> 4x4 homogeneous.
        R0 = np.eye(4)
        if "R0_rect" in raw:
            R0[:3, :3] = raw["R0_rect"].reshape(3, 3)
        elif "R_rect" in raw:
            R0[:3, :3] = raw["R_rect"].reshape(3, 3)

        # LiDAR -> camera -> 4x4 homogeneous.
        tr = raw.get("Tr_velo_to_cam", raw.get("Tr"))
        Tr = np.eye(4)
        Tr[:3, :4] = tr.reshape(3, 4)

        return cls(P_rect=P_rect, R0_rect=R0, Tr_velo_to_cam=Tr)

    # ---- Projection ---------------------------------------------------------
    def project_velo_to_image(
        self, points_velo: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Project Nx3 LiDAR points into the image plane.

        Returns
        -------
        uv : (M, 2) float pixel coordinates for points in front of the camera.
        depth : (M,) the camera-frame depth (z) of each kept point.

        Points behind the image plane (depth <= 0) are dropped, so M <= N.
        Caller is responsible for clipping to image width/height.
        """
        n = points_velo.shape[0]
        homo = np.hstack([points_velo[:, :3], np.ones((n, 1))])  # (N, 4)

        # velo -> cam -> rect
        cam = (self.Tr_velo_to_cam @ homo.T).T          # (N, 4)
        rect = (self.R0_rect @ cam.T).T                 # (N, 4)

        depth = rect[:, 2]
        in_front = depth > 1e-3
        rect = rect[in_front]
        depth = depth[in_front]

        # rect -> image (homogeneous)
        img = (self.P_rect @ rect.T).T                  # (M, 3)
        uv = img[:, :2] / img[:, 2:3]
        return uv, depth

    def points_in_image_fov(
        self,
        points_velo: np.ndarray,
        img_width: int,
        img_height: int,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Like project_velo_to_image but also clips to the image rectangle.

        Returns
        -------
        uv     : (K, 2) pixel coords inside the image
        depth  : (K,) depths
        mask   : (N,) boolean mask back into the ORIGINAL point array, so you
                 can recover which original LiDAR points landed on-screen
                 (useful for colouring the full cloud).
        """
        n = points_velo.shape[0]
        homo = np.hstack([points_velo[:, :3], np.ones((n, 1))])
        cam = (self.Tr_velo_to_cam @ homo.T).T
        rect = (self.R0_rect @ cam.T).T
        depth_all = rect[:, 2]

        img = (self.P_rect @ rect.T).T
        uv_all = img[:, :2] / np.where(img[:, 2:3] == 0, 1e-6, img[:, 2:3])

        mask = (
            (depth_all > 1e-3)
            & (uv_all[:, 0] >= 0)
            & (uv_all[:, 0] < img_width)
            & (uv_all[:, 1] >= 0)
            & (uv_all[:, 1] < img_height)
        )
        return uv_all[mask], depth_all[mask], mask
