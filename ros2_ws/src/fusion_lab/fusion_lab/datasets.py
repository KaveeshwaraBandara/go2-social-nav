"""
datasets.py — minimal loaders for KITTI-format data.

We deliberately keep this dependency-light (just numpy + pillow) so the core
projection demo runs even before you install pykitti / nuscenes-devkit.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


def load_velodyne_bin(path: str | Path) -> np.ndarray:
    """Load a KITTI Velodyne .bin scan.

    Returns (N, 4) array: x, y, z, reflectance.
    """
    scan = np.fromfile(str(path), dtype=np.float32)
    return scan.reshape(-1, 4)


def load_image(path: str | Path) -> np.ndarray:
    """Load an image as an (H, W, 3) uint8 RGB array."""
    from PIL import Image

    return np.array(Image.open(path).convert("RGB"))


class KittiFrame:
    """Convenience holder pairing one image, one scan, and its calibration."""

    def __init__(self, image: np.ndarray, points: np.ndarray, calib) -> None:
        self.image = image
        self.points = points
        self.calib = calib

    @property
    def height(self) -> int:
        return self.image.shape[0]

    @property
    def width(self) -> int:
        return self.image.shape[1]

    @classmethod
    def load(
        cls,
        image_path: str | Path,
        velo_path: str | Path,
        calib_path: str | Path,
    ) -> "KittiFrame":
        from .calibration import Calibration

        return cls(
            image=load_image(image_path),
            points=load_velodyne_bin(velo_path),
            calib=Calibration.from_kitti(calib_path),
        )
