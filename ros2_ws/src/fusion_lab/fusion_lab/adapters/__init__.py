"""
adapters — data-source adapters that feed the fusion_lab core.

The core (calibration / detection / lidar / association) is KITTI-shaped:
it consumes an (N, 3+) numpy point cloud in a forward-left-up "velo" frame,
an RGB image, and a `Calibration` object. Every new sensor rig gets an
adapter here that produces exactly those three things — the core never
changes.

`go2_producer` composes the whole chain into the real-perception PRODUCER that
emits the canonical `Track` contract. It is the only public entry point a
consumer of this package should need.
"""
from .go2 import Go2Rig, strip_zero_padding
from .go2_bag import Go2BagLoader, Go2Frame
from .go2_producer import Go2PerceptionProducer

__all__ = [
    "Go2BagLoader",
    "Go2Frame",
    "Go2PerceptionProducer",
    "Go2Rig",
    "strip_zero_padding",
]
