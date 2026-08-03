"""
oracle.py — THE GROUND-TRUTH PERCEPTION PRODUCER.

The second producer behind the canonical track contract, and the one that makes
the seam real in simulation:

    [THIS: HuNav /people ground truth]  ─┐
    [bag -> fusion -> tracker (real)]  ──┴──> Track ──> IT2-FLS controller

It converts HuNavSim's `people_msgs/People` into `Track` and emits nothing else.
Dependency direction is one-way: people_oracle -> tracking.

WHY THIS IS A LIBRARY AND NOT A NODE
------------------------------------
`Track` is deliberately an IN-PROCESS Python contract — there is no custom ROS
.msg for it, because that would couple the controller to a ROS distro, which is
exactly the Foxy(robot)/Humble(dev)/Jazzy(lab) problem the contract exists to
avoid (see CLAUDE.md, "Transport is deliberately undecided").

So the oracle does not subscribe to anything. The controller keeps its own
`/people` subscription — it is a ROS node, it has to get bytes from somewhere —
and hands the message straight here. What changes is the controller's INTERNAL
interface: it stops reading `people_msgs` fields and starts reading `Track`.
That is the whole point of the swap. When the live perception node lands, the
controller's maths does not move.

NO ROS IMPORTS HERE. The message is duck-typed: anything with `.people`, where
each element has `.name`, `.position.x/.y/.z` and `.velocity.x/.y`, works. That
keeps this package pure stdlib, testable at the desk with a plain namespace, and
usable unchanged on any distro — the same discipline that keeps `fusion_lab`'s
core free of rclpy.

WHAT AN ORACLE IS FOR
---------------------
It is the UPPER BOUND. Identity is never lost, position is never wrong, nothing
ever coasts. Any gap between the controller's behaviour on oracle tracks and on
real fusion tracks is attributable to perception, not to the controller — which
is what makes the sim benchmark and the hardware path comparable at all.

The corollary is the trap this file must not hide: **ground truth has zero
uncertainty**, so `pos_std` and `vel_std` are 0.0 here. An interval type-2
fuzzifier sizes its footprint of uncertainty from exactly those two numbers, so
a controller evaluated ONLY against this producer has a degenerate footprint and
silently behaves as a type-1 system. Injectable noise — a model that perturbs
these positions and reports honest non-zero std, calibrated against the ~0.30 m
the real fusion path actually achieves — is therefore not a nicety, it is what
makes the IT2 claim measurable. It is deliberately NOT built yet: this file's
job is the zero-noise identity swap, verified against the existing benchmark,
and nothing else.
"""
from __future__ import annotations

import math

from tracking import HEADING_HOLD_SPEED, Track

#: HuNav publishes no height: `position.z` carries the agent's YAW, not a
#: coordinate (see `yaw_from_person`). `Track.z` is documented as a standing
#: person's centroid height, which the real fusion path measures from the LiDAR
#: cluster, so the oracle emits a plausible constant rather than a lie shaped
#: like a measurement. Nothing in the planar motion model reads it.
NOMINAL_PERSON_CENTROID_Z = 0.9  # metres

#: Ground truth: the detection is certain and the association is certain.
#: `Track.confidence` is documented as "ground-truth producers emit 1.0".
GROUND_TRUTH_CONFIDENCE = 1.0

#: Ground truth carries no uncertainty. See the module docstring for why this
#: being zero is load-bearing rather than convenient.
GROUND_TRUTH_POS_STD = 0.0
GROUND_TRUTH_VEL_STD = 0.0


def yaw_from_person(person) -> float:
    """HuNav packs each agent's yaw into `position.z`.

    That is a real quirk of the `/people` contract, not a mistake to correct:
    `people_msgs/Person` has no orientation field, so HuNavSim reuses the unused
    z coordinate. Reading it as a height would put every pedestrian at a
    "height" of a few radians.
    """
    return float(person.position.z)


class PeopleOracle:
    """Turns HuNav `/people` messages into canonical `Track`s.

    Stateful across calls, because two `Track` fields only exist over time:
    `id` (which must be stable for the same agent) and `age`. Feed messages in
    order; one oracle per run.
    """

    def __init__(self, frame_id: str = "odom",
                 person_height: float = NOMINAL_PERSON_CENTROID_Z):
        """
        `frame_id` defaults to "odom" while `/people` is published in `map`.

        That is correct ONLY because this scene publishes a static IDENTITY
        map->odom transform (go2_hunav/cafe_isolated.launch.py), so the two
        frames coincide numerically — the same assumption stub_brain has
        documented since Phase 5, now stated in one more place because a Track
        carries its frame explicitly and a silent mismatch would show up as
        mirrored headings rather than as an error.

        If that transform ever stops being identity, this is where the tf2
        lookup goes.
        """
        self.frame_id = frame_id
        self.person_height = person_height
        self._ids: dict[str, int] = {}
        self._ages: dict[str, int] = {}
        self._next_id = 0

    def update(self, people_msg, timestamp: float) -> list[Track]:
        """Convert one `/people` message into `Track`s.

        `timestamp` is epoch seconds for the frame this message describes.
        """
        tracks = []
        for person in getattr(people_msg, "people", []):
            name = str(person.name)
            tracks.append(self._track_for(person, name, timestamp))
        return tracks

    def _track_for(self, person, name: str, timestamp: float) -> Track:
        if name not in self._ids:
            self._ids[name] = self._next_id
            self._next_id += 1
            self._ages[name] = 0
        else:
            self._ages[name] += 1

        vx = float(person.velocity.x)
        vy = float(person.velocity.y)

        return Track(
            id=self._ids[name],
            label="person",
            x=float(person.position.x),
            y=float(person.position.y),
            z=self.person_height,
            vx=vx,
            vy=vy,
            heading=self._heading(person, vx, vy),
            pos_std=GROUND_TRUTH_POS_STD,
            vel_std=GROUND_TRUTH_VEL_STD,
            confidence=GROUND_TRUTH_CONFIDENCE,
            timestamp=timestamp,
            # An oracle never coasts and never loses anyone: every agent in the
            # message is a fresh measurement. This staying 0.0 is precisely what
            # the controller reads to know it is NOT dead-reckoning, so it must
            # not be faked non-zero to "look realistic".
            time_since_update=0.0,
            age=self._ages[name],
            frame_id=self.frame_id,
        )

    def _heading(self, person, vx: float, vy: float) -> float:
        """Direction of travel while moving; true facing while standing still.

        `Track.heading` is defined as atan2(vy, vx), but the contract also says
        it is HELD when the object is too slow for velocity direction to mean
        anything — a standing person still faces a direction, and a social
        controller wants a stable heading rather than atan2 of noise.

        A real tracker satisfies that by holding its last confident value. The
        oracle can do strictly better: HuNav publishes the agent's actual yaw,
        which is the quantity the held value is an approximation OF. So below
        HEADING_HOLD_SPEED we use the true facing.

        This is not a producer inventing a field the others cannot emit — it is
        the same field, sourced more accurately, which is what "oracle" means.
        """
        if math.hypot(vx, vy) >= HEADING_HOLD_SPEED:
            return math.atan2(vy, vx)
        return yaw_from_person(person)
