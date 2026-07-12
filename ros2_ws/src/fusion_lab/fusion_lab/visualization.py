"""
visualization.py — draw projected points and fused objects on images.
"""
from __future__ import annotations

import numpy as np


def colormap_depth(depth: np.ndarray, max_depth: float = 50.0) -> np.ndarray:
    """Map depth values to BGR colors (near=red, far=blue) for overlay."""
    import cv2

    d = np.clip(depth / max_depth, 0, 1)
    # Use a perceptual colormap; invert so near objects are warm.
    colors = cv2.applyColorMap(((1 - d) * 255).astype(np.uint8), cv2.COLORMAP_JET)
    return colors.reshape(-1, 3)


def draw_projected_points(
    image: np.ndarray,
    uv: np.ndarray,
    depth: np.ndarray,
    radius: int = 2,
) -> np.ndarray:
    """Overlay projected LiDAR points colored by depth. Returns a BGR image."""
    import cv2

    canvas = cv2.cvtColor(image, cv2.COLOR_RGB2BGR).copy()
    colors = colormap_depth(depth)
    for (u, v), col in zip(uv.astype(int), colors):
        cv2.circle(canvas, (u, v), radius, tuple(int(c) for c in col), -1)
    return canvas


def draw_fused_objects(image: np.ndarray, fused, calib) -> np.ndarray:
    """Draw 2D boxes annotated with the fused 3D distance."""
    import cv2

    canvas = cv2.cvtColor(image, cv2.COLOR_RGB2BGR).copy()
    for obj in fused:
        x1, y1, x2, y2 = obj.detection.bbox.astype(int)
        dist = float(np.linalg.norm(obj.centroid_velo))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text = f"{obj.label} {dist:.1f}m"
        cv2.putText(canvas, text, (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return canvas
