"""
adapters — data-source adapters that feed the fusion_lab core.

The core (calibration / detection / lidar / association) is KITTI-shaped:
it consumes an (N, 3+) numpy point cloud in a forward-left-up "velo" frame,
an RGB image, and a `Calibration` object. Every new sensor rig gets an
adapter here that produces exactly those three things — the core never
changes.
"""
