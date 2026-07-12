"""association_utils.py — small geometry/matching helpers."""
from __future__ import annotations

import numpy as np


def iou_2d(box_a: np.ndarray, box_b: np.ndarray) -> float:
    """Intersection-over-union for two axis-aligned boxes [x1, y1, x2, y2]."""
    xa1, ya1, xa2, ya2 = box_a
    xb1, yb1, xb2, yb2 = box_b

    ix1, iy1 = max(xa1, xb1), max(ya1, yb1)
    ix2, iy2 = min(xa2, xb2), min(ya2, yb2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0

    area_a = max(0.0, xa2 - xa1) * max(0.0, ya2 - ya1)
    area_b = max(0.0, xb2 - xb1) * max(0.0, yb2 - yb1)
    return inter / (area_a + area_b - inter + 1e-9)


def hungarian_match(cost: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Optimal assignment. Uses scipy if present, else a greedy fallback."""
    try:
        from scipy.optimize import linear_sum_assignment

        return linear_sum_assignment(cost)
    except ImportError:
        # Greedy fallback: repeatedly take the global minimum.
        cost = cost.copy()
        rows, cols = [], []
        while np.isfinite(cost).any():
            r, c = np.unravel_index(np.argmin(cost), cost.shape)
            rows.append(r)
            cols.append(c)
            cost[r, :] = np.inf
            cost[:, c] = np.inf
        return np.array(rows), np.array(cols)
