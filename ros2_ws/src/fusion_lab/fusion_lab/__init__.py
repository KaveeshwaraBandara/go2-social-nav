"""fusion_lab — LiDAR + camera object association, from scratch."""
from .calibration import Calibration
from .datasets import KittiFrame, load_image, load_velodyne_bin
from .detection import Detection, YoloDetector
from .lidar import Cluster, cluster_dbscan, crop_range, remove_ground_ransac
from .association import FusedObject, frustum_association, match_by_iou

__version__ = "0.1.0"

__all__ = [
    "Calibration",
    "KittiFrame",
    "load_image",
    "load_velodyne_bin",
    "Detection",
    "YoloDetector",
    "Cluster",
    "cluster_dbscan",
    "crop_range",
    "remove_ground_ransac",
    "FusedObject",
    "frustum_association",
    "match_by_iou",
]
