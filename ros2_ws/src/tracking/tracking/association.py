"""
association.py — frame-to-frame data association.

Given the tracks we are already following and the detections that just arrived,
decide which detection continues which track.

Greedy nearest-neighbour under a hard distance gate. Deliberately the simplest
thing that can work: with a handful of well-separated people in a room, the
assignment is usually unambiguous, and a wrong-but-simple answer is easier to
debug than a right-but-opaque one. Hungarian/JPDA go in only if a validation
failure demands them.

Two details that are not arbitrary:

  * Candidate pairs are sorted by distance GLOBALLY, not resolved per-track in
    track order. Plain per-track greedy gives different answers depending on the
    order tracks happen to sit in the list — the same frame could associate two
    ways. Global sorting makes the result a deterministic function of geometry.

  * A track only matches a detection of its own class. A "person" track must not
    absorb a "chair" detection that happens to be 30 cm closer.

Gating is planar (xy). Height is carried through the pipeline but people do not
pass over or under each other, so folding z into the distance would only let
centroid-height noise break otherwise-clean matches.

Stdlib only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, Sequence


class _Positioned(Protocol):
    """Anything with a planar position and a class label."""
    x: float
    y: float
    label: str


@dataclass(frozen=True)
class Association:
    """The partition of tracks and detections produced by one association pass."""

    matches: list[tuple[int, int]]
    """(track_index, detection_index) pairs, both indexing the input sequences."""

    unmatched_tracks: list[int]
    """Track indices with no detection this frame — occluded, missed, or gone.
    The lifecycle decides which."""

    unmatched_detections: list[int]
    """Detection indices continuing no existing track — a new object, or noise.
    The lifecycle decides which."""


def greedy_associate(
    tracks: Sequence[_Positioned],
    detections: Sequence[_Positioned],
    gate_m: float | Sequence[float],
    match_labels: bool = True,
) -> Association:
    """Match detections to tracks by nearest neighbour within a distance gate.

    `gate_m` is the hard limit on how far an object may have moved since that
    track was last seen: a pair further apart is never matched, however
    unambiguous it looks.

    Pass a single float for a uniform gate, or one gate PER TRACK. Per-track is
    what you want in practice, because tracks do not all have the same staleness
    — one that coasted through three frames of occlusion has had three times as
    long to move as one updated last frame, and holding both to the same radius
    either strangles the stale track or lets the fresh one steal a neighbour's
    detection. `Tracker` sizes them from the real elapsed time.
    """
    if isinstance(gate_m, (int, float)):
        gates = [float(gate_m)] * len(tracks)
    else:
        gates = [float(g) for g in gate_m]
        if len(gates) != len(tracks):
            raise ValueError(
                f"gate_m has {len(gates)} entries for {len(tracks)} tracks"
            )

    candidates: list[tuple[float, int, int]] = []
    for ti, trk in enumerate(tracks):
        for di, det in enumerate(detections):
            if match_labels and trk.label != det.label:
                continue
            d = math.dist((trk.x, trk.y), (det.x, det.y))
            if d <= gates[ti]:
                candidates.append((d, ti, di))

    # Closest pair wins outright, then the next closest among what remains.
    # Sorting on (d, ti, di) keeps ties deterministic rather than dict-ordered.
    candidates.sort()

    taken_tracks: set[int] = set()
    taken_dets: set[int] = set()
    matches: list[tuple[int, int]] = []
    for _, ti, di in candidates:
        if ti in taken_tracks or di in taken_dets:
            continue
        matches.append((ti, di))
        taken_tracks.add(ti)
        taken_dets.add(di)

    return Association(
        matches=matches,
        unmatched_tracks=[i for i in range(len(tracks)) if i not in taken_tracks],
        unmatched_detections=[i for i in range(len(detections)) if i not in taken_dets],
    )
