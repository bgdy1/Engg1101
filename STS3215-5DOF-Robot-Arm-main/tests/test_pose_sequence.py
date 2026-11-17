import json
from contextlib import redirect_stdout
import io
import runpy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import arm_settings as settings

sequence_module = runpy.run_path(
    "scripts/05_waypoint_sequence.py", run_name="waypoint_sequence_test"
)


class PoseSequenceTests(unittest.TestCase):
    def test_player_uses_configured_sequence_speed_and_acceleration(self):
        class FakePort:
            def closePort(self):
                pass

        class FakeTerminal:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                pass

            def read_key(self, _timeout):
                return None

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "speed_test.json"
            positions = {name: settings.ZERO_TICKS[name] for name in settings.JOINT_NAMES}
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "speed_test",
                        "input_type": "raw",
                        "points": [
                            {"input_values": ["2048"] * 5},
                            {"input_values": ["2048"] * 5},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            move = Mock()
            wait = Mock(return_value=True)
            overrides = {
                "connect": Mock(return_value=(FakePort(), object())),
                "enable_all_torque": Mock(),
                "move_all_sync": move,
                "wait_for_servos_or_estop": wait,
                "RawTerminal": FakeTerminal,
            }
            with patch.dict(
                sequence_module["play_sequence"].__globals__, overrides
            ):
                with patch("builtins.input", return_value=""):
                    sequence_module["play_sequence"](path)

        self.assertEqual(move.call_count, 2)
        self.assertTrue(
            all(
                call.kwargs["speed"] == settings.POSE_SEQUENCE_MOVE_SPEED
                and call.kwargs["acceleration"]
                == settings.POSE_SEQUENCE_MOVE_ACCELERATION
                for call in move.call_args_list
            )
        )

    def test_raw_point_parses_five_positions_in_configured_order(self):
        positions = sequence_module["parse_point"]("0 90 4095 2048 1234", "raw")
        self.assertEqual(list(positions), list(settings.JOINT_NAMES))
        self.assertEqual(list(positions.values()), [0, 90, 4095, 2048, 1234])

    def test_angle_point_converts_to_raw_positions(self):
        positions = sequence_module["parse_point"]("0 90 -90 90 90", "angles")
        expected = {
            name: sequence_module["angle_to_position"](name, angle)
            for name, angle in zip(settings.JOINT_NAMES, (0, 90, -90, 90, 90))
        }
        self.assertEqual(positions, expected)

    def test_point_validation_rejects_bad_count_range_and_angle(self):
        with self.assertRaisesRegex(ValueError, "exactly 5"):
            sequence_module["parse_point"]("0 0 0", "raw")
        with self.assertRaisesRegex(ValueError, "between 0 and 4095"):
            sequence_module["parse_point"]("0 0 -1 0 0", "raw")
        with self.assertRaisesRegex(ValueError, "outside the configured limits"):
            sequence_module["parse_point"]("0 0 0 0 91", "angles")

    def test_sequence_load_converts_minimal_angle_points_to_raw_positions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "demo_sequence.json"
            point_values = ["0", "10", "-10", "20", "0"]
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "demo_sequence",
                        "points": [
                            {"input_values": point_values},
                            {"input_values": point_values},
                        ],
                        "input_type": "angles",
                    }
                ),
                encoding="utf-8",
            )
            _data, points = sequence_module["load_sequence"](path)
        self.assertEqual(len(points), 2)
        self.assertEqual(list(points[0]), list(settings.JOINT_NAMES))
        self.assertEqual(
            points[0],
            sequence_module["parse_point_values"](point_values, "angles"),
        )

    def test_sequence_load_accepts_filename_when_saved_name_differs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test1.json"
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "gripper",
                        "input_type": "raw",
                        "points": [
                            {"input_values": ["2048"] * 5},
                            {"input_values": ["2048"] * 5},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            sequence, points = sequence_module["load_sequence"](path)
        self.assertEqual(sequence["name"], "gripper")
        self.assertEqual(len(points), 2)

    def test_sequence_load_still_accepts_legacy_raw_position_points(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.json"
            raw_positions = {
                name: settings.ZERO_TICKS[name] for name in settings.JOINT_NAMES
            }
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "legacy",
                        "input_type": "raw",
                        "points": [
                            {"positions_raw": raw_positions},
                            {"positions_raw": raw_positions},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            _sequence, points = sequence_module["load_sequence"](path)
        self.assertEqual(points, [raw_positions, raw_positions])

    def test_entering_play_saves_sequence_and_returns_path_for_playback(self):
        with tempfile.TemporaryDirectory() as directory:
            create_globals = sequence_module["create_sequence"].__globals__
            answers = iter(
                (
                    "paper_demo",
                    "Example points",
                    "2",
                    "0 0 0 0 0",
                    "1 0 0 0 0",
                    "play",
                )
            )
            with patch.dict(create_globals, {"MOTION_FOLDER": Path(directory)}):
                with patch(
                    "builtins.input", side_effect=lambda _prompt: next(answers)
                ):
                    path = sequence_module["create_sequence"]()
            self.assertEqual(path, Path(directory) / "paper_demo.json")
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(len(data["points"]), 2)
            self.assertEqual(data["type"], "pose_sequence")
            self.assertEqual(data["name"], "paper_demo")
            self.assertEqual(
                data["joint_order"],
                [
                    {"servo_id": settings.SERVO_IDS[joint], "joint": joint}
                    for joint in settings.JOINT_NAMES
                ],
            )
            self.assertEqual(
                data["points"],
                [
                    {"input_values": ["0", "0", "0", "0", "0"]},
                    {"input_values": ["1", "0", "0", "0", "0"]},
                ],
            )
            self.assertTrue(
                all(set(point) == {"input_values"} for point in data["points"])
            )
            saved_text = path.read_text(encoding="utf-8")
            self.assertIn(
                '    {"input_values": ["0", "0", "0", "0", "0"]},',
                saved_text,
            )

    def test_regular_motion_picker_does_not_list_pose_sequences(self):
        motion_module = runpy.run_path(
            "scripts/04_timestamp_motion.py", run_name="motion_filter_test"
        )
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "demo_sequence.json").write_text(
                json.dumps({"type": "pose_sequence", "name": "demo_sequence"}),
                encoding="utf-8",
            )
            (folder / "demo_motion.json").write_text(
                json.dumps({"name": "demo_motion", "steps": []}),
                encoding="utf-8",
            )
            motion_module["find_motion_files"].__globals__["MOTION_FOLDER"] = folder
            self.assertEqual(
                [path.name for path in motion_module["find_motion_files"]()],
                ["demo_motion.json"],
            )


class PlaybackActionTests(unittest.TestCase):
    def play(self, points, input_type="angles", keys=None, waits=None):
        output = io.StringIO()
        snapshots = []
        port = Mock()
        terminal = MagicMock()
        terminal.__enter__.return_value = terminal
        terminal.read_key.side_effect = keys if keys is not None else [None] * len(points)
        move = Mock(side_effect=lambda *_args, **_kwargs: snapshots.append(output.getvalue()))
        wait = Mock(side_effect=waits) if waits is not None else Mock(return_value=True)
        disable = Mock()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "actions.json"
            path.write_text(json.dumps({
                "type": "pose_sequence", "input_type": input_type, "points": points,
            }), encoding="utf-8")
            overrides = {
                "connect": Mock(return_value=(port, object())),
                "enable_all_torque": Mock(),
                "disable_all_torque": disable,
                "move_all_sync": move,
                "wait_for_servos_or_estop": wait,
                "RawTerminal": Mock(return_value=terminal),
            }
            with patch.dict(sequence_module["play_sequence"].__globals__, overrides):
                with patch("builtins.input", return_value=""), redirect_stdout(output):
                    sequence_module["play_sequence"](path)
        port.closePort.assert_called_once()
        terminal.__exit__.assert_called_once()
        return output.getvalue(), move, wait, disable, snapshots

    def test_each_action_and_all_five_target_angles_are_printed_before_moving(self):
        output, move, wait, _disable, snapshots = self.play([
            {"action_name": "夹住物体", "input_values": ["0", "10", "-10", "20", "90"]},
            {"action_name": "移动物体", "input_values": ["0"] * 5},
        ])
        self.assertIn("正在执行 1/2: 夹住物体", snapshots[0])
        self.assertNotIn("移动物体", snapshots[0])
        self.assertIn("正在执行 2/2: 移动物体", snapshots[1])
        for joint, value in zip(settings.JOINT_NAMES, ("0.00", "10.02", "-10.02", "20.04", "90.00")):
            self.assertIn(f"{joint}: {value} deg", snapshots[0])
        self.assertIn("Target joint angles", output)
        self.assertIn("\r\n正在执行 2/2", output)
        self.assertEqual(move.call_count, 2)
        self.assertEqual(wait.call_count, 2)
        self.assertTrue(all(
            call.kwargs == {"speed": settings.POSE_SEQUENCE_MOVE_SPEED,
                            "acceleration": settings.POSE_SEQUENCE_MOVE_ACCELERATION}
            for call in move.call_args_list
        ))

    def test_missing_empty_and_whitespace_names_have_step_labels(self):
        points = [{"input_values": ["0"] * 5} for _ in range(3)]
        points[1]["action_name"] = ""
        points[2]["action_name"] = "   "
        output, *_rest = self.play(points)
        for index in range(1, 4):
            self.assertIn(f"正在执行 {index}/3: 步骤 {index}", output)

    def test_raw_and_legacy_targets_are_displayed_as_angles(self):
        raw = dict(zip(settings.JOINT_NAMES, (2048, 2162, 1934, 2276, 1024)))
        for point in ({"input_values": list(raw.values())}, {"positions_raw": raw}):
            with self.subTest(point=point):
                output, move, *_rest = self.play([point, point], input_type="raw")
                self.assertIn("shoulder: 10.02 deg", output)
                self.assertIn("elbow: -10.02 deg", output)
                self.assertIn("gripper: 90.00 deg", output)
                self.assertEqual(move.call_args_list[0].args[1], raw)

    def test_repeated_waypoints_are_both_executed(self):
        point = {"action_name": "保持姿态", "input_values": ["0"] * 5}
        output, move, wait, *_rest = self.play([point, point])
        self.assertEqual(move.call_count, 2)
        self.assertEqual(wait.call_count, 2)
        self.assertEqual(output.count("保持姿态"), 2)

    def test_space_before_next_action_prevents_its_output_and_movement(self):
        output, move, _wait, disable, _snapshots = self.play([
            {"action_name": "first", "input_values": ["0"] * 5},
            {"action_name": "must not run", "input_values": ["0"] * 5},
        ], keys=[None, " "])
        self.assertEqual(move.call_count, 1)
        disable.assert_called_once()
        self.assertNotIn("must not run", output)
        self.assertNotIn("sequence finished", output)

    def test_stop_during_wait_prevents_the_next_action(self):
        output, move, *_rest = self.play([
            {"action_name": "first", "input_values": ["0"] * 5},
            {"action_name": "must not run", "input_values": ["0"] * 5},
        ], waits=[False])
        self.assertEqual(move.call_count, 1)
        self.assertNotIn("must not run", output)
        self.assertNotIn("sequence finished", output)

    def test_invalid_action_name_is_rejected_before_connecting(self):
        for invalid in (None, 123, {}, []):
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "invalid.json"
                path.write_text(json.dumps({
                    "type": "pose_sequence", "input_type": "angles", "points": [
                        {"input_values": ["0"] * 5},
                        {"input_values": ["0"] * 5, "action_name": invalid},
                    ],
                }), encoding="utf-8")
                connect = Mock()
                with patch.dict(sequence_module["play_sequence"].__globals__, {"connect": connect}):
                    with self.assertRaisesRegex(ValueError, "point 2: action_name must be a string"):
                        sequence_module["play_sequence"](path)
                connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
