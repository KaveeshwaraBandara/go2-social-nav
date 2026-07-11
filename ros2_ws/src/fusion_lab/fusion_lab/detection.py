"""
detection.py — Phase 1: camera-only 2D object detection.

Thin wrapper over Ultralytics YOLO so the rest of the pipeline depends on a
small, stable interface (a list of Detection dataclasses) rather than on
YOLO's result objects directly.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    """A single 2D detection in image pixel coordinates."""

    bbox: np.ndarray   # (4,) x1, y1, x2, y2
    score: float
    class_id: int
    label: str

    def contains(self, uv: np.ndarray) -> np.ndarray:
        """Boolean mask over an (N, 2) array of pixels inside this box."""
        x1, y1, x2, y2 = self.bbox
        return (
            (uv[:, 0] >= x1) & (uv[:, 0] <= x2)
            & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
        )


class YoloDetector:
    """Lazy-loaded YOLO detector. Model weights download on first use."""

    def __init__(self, model_name: str = "yolov8n.pt", conf: float = 0.35) -> None:
        self.model_name = model_name
        self.conf = conf
        self._model = None

    def _ensure_model(self):
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(self.model_name)
        return self._model

    def detect(self, image: np.ndarray) -> list[Detection]:
        model = self._ensure_model()
        results = model.predict(image, conf=self.conf, verbose=False)[0]
        names = results.names

        detections: list[Detection] = []
        for box in results.boxes:
            cls_id = int(box.cls.item())
            detections.append(
                Detection(
                    bbox=box.xyxy.cpu().numpy().ravel(),
                    score=float(box.conf.item()),
                    class_id=cls_id,
                    label=names[cls_id],
                )
            )
        return detections
