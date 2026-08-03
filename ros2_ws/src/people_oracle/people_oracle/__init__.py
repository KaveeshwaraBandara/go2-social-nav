"""
people_oracle — the ground-truth perception producer for go2-social-nav.

Self-contained by construction: imports `tracking` and nothing else. No rclpy,
no numpy, no people_msgs — the `/people` message is duck-typed, so this package
is pure stdlib and runs unchanged on Foxy, Humble and Jazzy.

    people_msgs/People (HuNav ground truth) -> [PeopleOracle] -> list[Track]

Sibling of `fusion_lab`, which produces the same `Track` from real sensors.
The controller cannot tell them apart, which is the entire point.
"""
from .oracle import (
    GROUND_TRUTH_CONFIDENCE,
    GROUND_TRUTH_POS_STD,
    GROUND_TRUTH_VEL_STD,
    NOMINAL_PERSON_CENTROID_Z,
    PeopleOracle,
    yaw_from_person,
)

__version__ = "0.1.0"

__all__ = [
    "PeopleOracle",
    "yaw_from_person",
    "NOMINAL_PERSON_CENTROID_Z",
    "GROUND_TRUTH_CONFIDENCE",
    "GROUND_TRUTH_POS_STD",
    "GROUND_TRUTH_VEL_STD",
]
