#!/usr/bin/env python3
"""Phase 6d: summarise results/benchmark.csv into a controller-vs-controller table.

Reads the master benchmark.csv (one row per controller x scenario x run), averages
over runs, and writes a compact Markdown comparison table (results/comparison.md) plus
prints it. This is the headline artifact of Phase 6: stub vs DWA vs TEB on identical
scenarios, measured identically.

Usage:  python3 compare.py [results_dir]   (default ~/ros2_ws/results)
"""
import csv
import os
import sys
from collections import defaultdict


# (csv column, header, format) -- the headline metrics for the comparison.
COLS = [
    ("success", "succ", "{:.0f}"),
    ("our_time_to_goal", "t_goal[s]", "{:.1f}"),
    ("our_path_length", "path[m]", "{:.2f}"),
    ("jerk_mean", "jerk_mean", "{:.2f}"),
    ("jerk_rms", "jerk_rms", "{:.2f}"),
    ("eval_minimum_distance_to_people", "min_dist[m]", "{:.2f}"),
    ("eval_intimate_space_intrusions", "intim%", "{:.1f}"),
    ("eval_personal_space_intrusions", "pers%", "{:.1f}"),
    ("eval_social_space_intrusions", "soc%", "{:.1f}"),
    ("eval_robot_on_person_collision", "coll", "{:.0f}"),
]
CONTROLLER_ORDER = ["stub", "dwa", "teb"]


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main():
    results_dir = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                                     else "~/ros2_ws/results")
    csv_path = os.path.join(results_dir, "benchmark.csv")
    rows = list(csv.DictReader(open(csv_path)))

    # group rows by (scenario, controller), average each metric over runs
    groups = defaultdict(list)
    for r in rows:
        groups[(r["scenario"], r["controller"])].append(r)

    scenarios = sorted({s for s, _ in groups})
    header = ["scenario", "controller", "runs"] + [h for _, h, _ in COLS]
    lines = ["| " + " | ".join(header) + " |",
             "|" + "|".join(["---"] * len(header)) + "|"]

    for sc in scenarios:
        for ctrl in CONTROLLER_ORDER:
            rs = groups.get((sc, ctrl))
            if not rs:
                continue
            cells = [sc, ctrl, str(len(rs))]
            for col, _, fmt in COLS:
                vals = [fnum(r.get(col)) for r in rs]
                vals = [v for v in vals if v is not None]
                cells.append(fmt.format(sum(vals) / len(vals)) if vals else "-")
            lines.append("| " + " | ".join(cells) + " |")

    table = "\n".join(lines)
    print(table)
    out = os.path.join(results_dir, "comparison.md")
    with open(out, "w") as f:
        f.write("# Phase 6 benchmark: stub vs DWA vs TEB\n\n")
        f.write("Lower jerk + fewer intimate/personal intrusions + larger min_dist is "
                "better; succ=1 means the goal was reached.\n\n")
        f.write(table + "\n")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
