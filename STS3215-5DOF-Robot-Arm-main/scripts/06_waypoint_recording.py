#!/usr/bin/env python3
"""Capture hand-positioned joint angles as waypoints playable by script 05."""

from datetime import datetime
import json
import math
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
    check_result,
    connect,
    disable_all_torque,
    position_to_angle,
    read_servo,
)
from scservo_sdk import SMS_STS_TORQUE_ENABLE

MOTION_FOLDER = PROJECT_FOLDER / "motions"
NAME_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}$")
GRIPPER = "gripper"


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


def read_gripper_target(action):
    """Return a validated angle in degrees and its servo position."""
    while True:
        value = input(f"Gripper {action} angle (degrees): ").strip()
        try:
            angle = float(value)
            if not math.isfinite(angle):
                raise ValueError("Enter a finite number of degrees")
            return angle, angle_to_position(GRIPPER, angle)
        except ValueError as error:
            print(f"Invalid {action} angle: {error}.")


def read_angle_point(packet_handler, gripper_angle):
    """Sample the free joints and add the selected gripper target angle."""
    angles = {GRIPPER: gripper_angle}
    for name in settings.JOINT_NAMES:
        if name != GRIPPER:
            position = read_servo(packet_handler, settings.SERVO_IDS[name])
            angles[name] = position_to_angle(name, position)
            angle_to_position(name, angles[name])
    return {"input_values": [str(angles[name]) for name in settings.JOINT_NAMES]}


class RecordingGripper:
    """Manage the one servo that can be powered during manual recording."""

    def __init__(self, packet_handler):
        self.packet_handler = packet_handler
        self.servo_id = settings.SERVO_IDS[GRIPPER]
        self.needs_release = False

    def write_position(self, position):
        result, error = self.packet_handler.WritePosEx(
            self.servo_id, position,
            settings.GRIPPER_RECORD_MOVE_SPEED, settings.GRIPPER_RECORD_MOVE_ACCELERATION,
        )
        check_result(self.packet_handler, result, error, "Set gripper target")

    def set_torque(self, enabled):
        result, error = self.packet_handler.write1ByteTxRx(
            self.servo_id, SMS_STS_TORQUE_ENABLE, int(enabled)
        )
        action = "Enable" if enabled else "Disable"
        check_result(self.packet_handler, result, error, f"{action} gripper torque")

    def move_to(self, target):
        if not self.needs_release:
            present = read_servo(self.packet_handler, self.servo_id)
            if not 0 <= present <= 4095:
                raise RuntimeError("Gripper current position is outside 0–4095 ticks")
            # Register cleanup before any write whose outcome could be uncertain.
            self.needs_release = True
            self.write_position(present)
            self.set_torque(True)
        self.write_position(target)

    def release(self):
        if self.needs_release:
            self.set_torque(False)
            self.needs_release = False


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


def record_points(packet_handler, points, gripper, grasp, release):
    """Capture one pose per grasp or release command, then accept its name."""
    commands = {"q": ("Grasp", grasp), "w": ("Release", release)}
    print("Support and hold the arm steady, then enter q to grasp or w to open the gripper.")
    print(f"Grasp: {grasp[0]:g} deg. Release: {release[0]:g} deg.")
    print("Enter e to save (at least two points). Press Enter after each command.")
    print("Keep clear of the moving jaws and support the object before exiting.")
    print("Ctrl+C, end of input, or a fault may release gripper torque.")
    print("Order: " + ", ".join(settings.JOINT_NAMES))
    while True:
        command = input("Command (q = grasp, w = release, e = save): ").strip().lower()
        if command == "e":
            if len(points) < 2:
                print("Record at least two waypoints before saving.")
                continue
            if gripper.needs_release:
                confirmation = input(
                    "Support any object, then disable gripper torque and save? [y/N]: "
                ).strip().lower()
                if confirmation not in ("y", "yes"):
                    print("Save canceled. Continue recording.")
                    continue
            return
        elif command in commands:
            action, (angle, position) = commands[command]
            try:
                point = read_angle_point(packet_handler, angle)
            except ValueError as error:
                print(f"Point rejected: {error}. Adjust the pose and enter {command} again.")
                continue
            gripper.move_to(position)
            print(f"{action} target accepted ({angle:g} deg). Check the gripper before moving the arm.")
            point["action_name"] = input("Action name (optional; Enter to skip): ").strip()
            points.append(point)
            action_name = point["action_name"] or f"步骤 {len(points)}"
            display = ", ".join(
                f"{name}: {float(value):.2f} deg"
                for name, value in zip(settings.JOINT_NAMES, point["input_values"])
            )
            print(f"Point {len(points)} recorded ({action_name}): {display}")
        else:
            print("Please enter q to grasp, w to release, or e to save, then press Enter.")


def record_sequence():
    """Own the serial session; preserve complete points when recording stops."""
    port = None
    try:
        input("Support the arm before torque is released, then press Enter...")
        port, packet_handler = connect()
        disable_all_torque(packet_handler)
        print("Torque disabled on all configured servos; keep supporting the arm.")
        path = choose_path()
        grasp = read_gripper_target("grasp")
        release = read_gripper_target("release")
        gripper = RecordingGripper(packet_handler)
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
            record_points(packet_handler, sequence["points"], gripper, grasp, release)
        except (EOFError, KeyboardInterrupt):
            print("\nRecording interrupted; preserving complete waypoints.")
        except Exception as error:
            recording_error = error
            print(f"\nRecording failed: {error}. Preserving complete waypoints.", file=sys.stderr)
        finally:
            try:
                gripper.release()
            except (Exception, KeyboardInterrupt) as error:
                detail = str(error) or type(error).__name__
                message = f"Gripper torque off could not be confirmed: {detail}"
                print(message, file=sys.stderr)
                if recording_error is None:
                    recording_error = RuntimeError(message)

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
