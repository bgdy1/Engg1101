#!/usr/bin/env python3
"""Capture hand-positioned joint angles as waypoints playable by script 05."""

from datetime import datetime
import json
import os
from pathlib import Path
import re
import sys
import tempfile

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_FOLDER))

import arm_settings as settings
from arm_helpers import (
    angle_to_position,
    connect,
    disable_all_torque,
    positions_to_angles,
    read_all,
)

MOTION_FOLDER = PROJECT_FOLDER / "motions"
NAME_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}$")


def choose_path():
    """Ask for an unused sequence name without accepting directory paths."""
    while True:
        name = input("Waypoint sequence name (letters, numbers, _ or -): ").strip()
        if not NAME_PATTERN.fullmatch(name):
            print("Use 1-48 characters, starting with a letter or number; then letters, numbers, _ or -.")
            continue
        path = MOTION_FOLDER / f"{name}.json"
        if path.exists():
            print(f"{path} already exists; choose a new name.")
            continue
        return path


def read_angle_point(packet_handler):
    """Read one complete pose and validate its angles against playback limits."""
    angles = positions_to_angles(read_all(packet_handler))
    for name in settings.JOINT_NAMES:
        angle_to_position(name, angles[name])
    return {"input_values": [str(angles[name]) for name in settings.JOINT_NAMES]}


def save_sequence(path, sequence):
    """Replace a completed temporary JSON file, refusing an existing sequence."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", suffix=".tmp", delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(sequence, temporary, indent=2, allow_nan=False, ensure_ascii=False)
            temporary.write("\n")
        if path.exists():
            raise FileExistsError(f"{path} already exists; refusing to overwrite it")
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def record_points(packet_handler, points):
    """Capture one pose per q + Enter, then accept an optional action name."""
    print("Move and support the arm at each pose, then enter q to record.")
    print("Enter e to save (at least two points). Press Enter after each command.")
    print("Action names are optional. Ctrl+C ends recording.")
    print("Order: " + ", ".join(settings.JOINT_NAMES))
    while True:
        command = input("Command (q = record, e = save): ").strip().lower()
        if command == "e":
            if len(points) >= 2:
                return
            print("Record at least two waypoints before saving.")
        elif command == "q":
            try:
                point = read_angle_point(packet_handler)
            except ValueError as error:
                print(f"Point rejected: {error}. Adjust the pose and enter q again.")
                continue
            point["action_name"] = input("Action name (optional; Enter to skip): ").strip()
            points.append(point)
            action_name = point["action_name"] or f"步骤 {len(points)}"
            display = ", ".join(
                f"{name}: {float(value):.2f} deg"
                for name, value in zip(settings.JOINT_NAMES, point["input_values"])
            )
            print(f"Point {len(points)} recorded ({action_name}): {display}")
        else:
            print("Please enter q to record or e to save, then press Enter.")


def record_sequence():
    """Own the serial session; preserve complete points when recording stops."""
    port = None
    try:
        input("Support the arm before torque is released, then press Enter...")
        port, packet_handler = connect()
        disable_all_torque(packet_handler)
        print("Torque disabled on all configured servos; keep supporting the arm.")
        path = choose_path()
        sequence = {
            "schema_version": 1,
            "type": "pose_sequence",
            "name": path.stem,
            "description": "",
            "input_type": "angles",
            "joint_order": [
                {"servo_id": settings.SERVO_IDS[name], "joint": name}
                for name in settings.JOINT_NAMES
            ],
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "points": [],
        }
        recording_error = None
        try:
            record_points(packet_handler, sequence["points"])
        except (EOFError, KeyboardInterrupt):
            print("\nRecording interrupted; preserving complete waypoints.")
        except Exception as error:
            recording_error = error
            print(f"\nRecording failed: {error}. Preserving complete waypoints.", file=sys.stderr)

        saved_path = None
        if len(sequence["points"]) >= 2:
            save_sequence(path, sequence)
            saved_path = path
            print(f"Saved {len(sequence['points'])} angle waypoints to {path.resolve()}")
            print("Run scripts/05_waypoint_sequence.py and choose 2 to play this sequence.")
        else:
            print("Fewer than two complete waypoints; no file saved.")
        if recording_error is not None:
            raise recording_error
        return saved_path
    finally:
        if port is not None:
            port.closePort()


def main():
    try:
        record_sequence()
    except (EOFError, KeyboardInterrupt):
        print("\nWaypoint recording stopped.")
    except Exception as error:
        print(f"\nWaypoint recording could not finish: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
