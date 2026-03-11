#!/usr/bin/env python3
"""
Flip a Choreo trajectory over the field Y-axis (left/right mirror).

Usage: python3 flip_trajectory.py <input.traj> [output.traj]

If no output path is given, creates <input>_flipped.traj
"""

import json
import re
import sys
from pathlib import Path

FIELD_WIDTH = 8.069  # 2026 Reefscape field width in meters


def flip_y(y: float) -> float:
    """Mirror Y coordinate over field centerline."""
    return FIELD_WIDTH - y


def flip_heading(h: float) -> float:
    """Mirror heading over Y-axis."""
    return -h


def flip_expression(exp: str, is_heading: bool = False) -> str:
    """Update expression string like '0.699 m' or '2.15 rad' with flipped value."""
    # Match number at start of expression
    match = re.match(r"^(-?\d+\.?\d*)", exp)
    if not match:
        return exp

    value = float(match.group(1))
    if is_heading:
        new_value = flip_heading(value)
    else:
        new_value = flip_y(value)

    # Replace the number, keeping the unit
    return re.sub(r"^-?\d+\.?\d*", f"{new_value}", exp)


def flip_waypoint_snapshot(wp: dict) -> dict:
    """Flip a waypoint in snapshot.waypoints format."""
    wp["y"] = flip_y(wp["y"])
    wp["heading"] = flip_heading(wp["heading"])
    return wp


def flip_waypoint_params(wp: dict) -> dict:
    """Flip a waypoint in params.waypoints format (has exp/val structure)."""
    wp["y"]["val"] = flip_y(wp["y"]["val"])
    wp["y"]["exp"] = flip_expression(wp["y"]["exp"], is_heading=False)
    wp["heading"]["val"] = flip_heading(wp["heading"]["val"])
    wp["heading"]["exp"] = flip_expression(wp["heading"]["exp"], is_heading=True)
    return wp


def flip_sample(sample: dict) -> dict:
    """Flip a trajectory sample point."""
    sample["y"] = flip_y(sample["y"])
    sample["heading"] = flip_heading(sample["heading"])
    sample["vy"] = -sample["vy"]
    sample["omega"] = -sample["omega"]
    sample["ay"] = -sample["ay"]
    sample["alpha"] = -sample["alpha"]
    # fy is an array of forces for each wheel
    sample["fy"] = [-f for f in sample["fy"]]
    return sample


def flip_trajectory(data: dict) -> dict:
    """Flip entire trajectory data structure."""
    # Flip snapshot waypoints
    if "snapshot" in data and "waypoints" in data["snapshot"]:
        data["snapshot"]["waypoints"] = [
            flip_waypoint_snapshot(wp) for wp in data["snapshot"]["waypoints"]
        ]

    # Flip params waypoints
    if "params" in data and "waypoints" in data["params"]:
        data["params"]["waypoints"] = [
            flip_waypoint_params(wp) for wp in data["params"]["waypoints"]
        ]

    # Flip trajectory samples
    if "trajectory" in data and "samples" in data["trajectory"]:
        data["trajectory"]["samples"] = [
            flip_sample(s) for s in data["trajectory"]["samples"]
        ]

    return data


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_path = Path(sys.argv[1])
    if not input_path.exists():
        print(f"Error: {input_path} not found")
        sys.exit(1)

    if len(sys.argv) >= 3:
        output_path = Path(sys.argv[2])
    else:
        output_path = input_path.with_stem(input_path.stem + "_flipped")

    # Read input
    with open(input_path) as f:
        data = json.load(f)

    # Flip trajectory
    flipped = flip_trajectory(data)

    # Update name if present
    if "name" in flipped:
        flipped["name"] = flipped["name"] + "_flipped"

    # Write output
    with open(output_path, "w") as f:
        json.dump(flipped, f, indent=1)

    print(f"Flipped trajectory written to: {output_path}")


if __name__ == "__main__":
    main()
