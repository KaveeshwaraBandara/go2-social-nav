#!/usr/bin/env python3
"""
check_oracle.py — desk verification for the ground-truth perception producer.

No ROS graph, no Gazebo, no robot: it builds synthetic `/people` messages,
runs them through `PeopleOracle`, and checks the `Track`s that come out.

    ros2 run people_oracle check_oracle.py

The single most important check here is EXACT POSITION PASS-THROUGH.

`stub_brain` reads exactly two things off each person — `position.x` and
`position.y` — and nothing else. So if this oracle reproduces those two numbers
bit-for-bit, porting the controller from `people_msgs/Person` to `Track` cannot
change its output, whatever the rest of the conversion does. That is why the
port is safe to make in one step, and why the Phase-6 benchmark is expected to
return IDENTICAL metrics rather than merely similar ones.
"""
import math
import sys
from types import SimpleNamespace


def person(name, x, y, yaw=0.0, vx=0.0, vy=0.0):
    """A duck-typed people_msgs/Person. HuNav packs yaw into position.z."""
    return SimpleNamespace(
        name=name,
        position=SimpleNamespace(x=x, y=y, z=yaw),
        velocity=SimpleNamespace(x=vx, y=vy, z=0.0),
    )


def people(*persons):
    """A duck-typed people_msgs/People."""
    return SimpleNamespace(people=list(persons))


def main() -> int:
    from people_oracle import (
        GROUND_TRUTH_CONFIDENCE,
        NOMINAL_PERSON_CENTROID_Z,
        PeopleOracle,
    )
    from tracking import HEADING_HOLD_SPEED, TRACK_SCHEMA_VERSION, Track

    failures = []

    def check(name, ok, detail=""):
        print(f"  {'ok  ' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail else ""))
        if not ok:
            failures.append(name)

    print(f"tracking schema version {TRACK_SCHEMA_VERSION}\n")

    # --- THE ONE THAT MATTERS: exact pass-through --------------------------
    # Deliberately awkward values: if anything rounds, re-packs through a float32
    # or routes via a trig identity, these will not survive.
    print("position pass-through (the equivalence proof for the stub_brain port)")
    oracle = PeopleOracle()
    ugly = [(-3.141592653589793, 2.718281828459045),
            (1e-7, -1e-7),
            (12345.6789, -98765.4321)]
    msg = people(*[person(f"a{i}", x, y) for i, (x, y) in enumerate(ugly)])
    tracks = oracle.update(msg, timestamp=100.0)
    check("one track per person", len(tracks) == len(ugly), f"{len(tracks)}")
    exact = all(t.x == p.position.x and t.y == p.position.y
                for t, p in zip(tracks, msg.people))
    check("x/y are bit-identical to /people", exact)

    # --- the yaw-in-z quirk -------------------------------------------------
    print("\nHuNav packs yaw into position.z")
    oracle = PeopleOracle()
    t = oracle.update(people(person("p", 1.0, 2.0, yaw=1.234)), 0.0)[0]
    check("z is a nominal height, NOT the yaw",
          t.z == NOMINAL_PERSON_CENTROID_Z, f"z={t.z}")
    check("standing person's heading is the true facing yaw",
          t.heading == 1.234, f"heading={t.heading}")

    fast = 2.0 * HEADING_HOLD_SPEED
    t = oracle.update(people(person("q", 0.0, 0.0, yaw=1.234, vx=fast, vy=fast)), 0.0)[0]
    check("moving person's heading is direction of travel",
          math.isclose(t.heading, math.atan2(fast, fast)), f"heading={t.heading:.4f}")

    # --- ground truth means ZERO uncertainty --------------------------------
    print("\nground truth (this is what makes an oracle an oracle)")
    check("pos_std is zero", t.pos_std == 0.0)
    check("vel_std is zero", t.vel_std == 0.0)
    check("confidence is 1.0", t.confidence == GROUND_TRUTH_CONFIDENCE)
    check("never coasting", t.time_since_update == 0.0 and not t.is_coasting)
    check("frame is odom (static identity map->odom)", t.frame_id == "odom")

    # --- identity over time -------------------------------------------------
    print("\nidentity")
    oracle = PeopleOracle()
    oracle.update(people(person("alice", 0, 0), person("bob", 1, 1)), 0.0)
    second = oracle.update(people(person("alice", 0, 0), person("bob", 1, 1)), 0.1)
    ids = {t.id for t in second}
    check("distinct people get distinct ids", len(ids) == 2, f"{sorted(ids)}")
    check("same person keeps its id across frames", second[0].id == 0)
    check("age increments", second[0].age == 1, f"age={second[0].age}")

    # An oracle knows who is who. A real tracker would assign a NEW id here,
    # and that difference is exactly the perception error the oracle bounds.
    oracle.update(people(person("bob", 1, 1)), 0.2)          # alice vanishes
    back = oracle.update(people(person("alice", 0, 0)), 0.3)[0]
    check("identity survives a disappearance (oracle never loses anyone)",
          back.id == 0, f"id={back.id}")

    # --- degenerate input ---------------------------------------------------
    print("\nedge cases")
    check("empty message -> no tracks", PeopleOracle().update(people(), 0.0) == [])
    check("tracks are frozen", _is_frozen(tracks[0]))
    check("emits the canonical Track type", isinstance(tracks[0], Track))

    print()
    if failures:
        print(f"ORACLE CHECK FAILED: {len(failures)} check(s): {', '.join(failures)}")
        return 1
    print("ORACLE CHECK PASSED — /people converts to Track with exact positions.")
    print("NOTE: pos_std/vel_std are 0.0 by construction. A controller measured")
    print("      only against this producer has a degenerate footprint of")
    print("      uncertainty and behaves as type-1. Injected noise is next.")
    return 0


def _is_frozen(track) -> bool:
    try:
        track.x = 0.0
        return False
    except Exception:
        return True


if __name__ == "__main__":
    raise SystemExit(main())
