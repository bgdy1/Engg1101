from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import Mock, call, patch

import arm_settings as settings

recording = runpy.run_path(
    "scripts/06_waypoint_recording.py", run_name="waypoint_recording_test"
)
player = runpy.run_path(
    "scripts/05_waypoint_sequence.py", run_name="waypoint_recording_player_test"
)


class WaypointRecordingTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.folder = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.output = io.StringIO()
        self.stack.enter_context(redirect_stdout(self.output))
        self.stack.enter_context(redirect_stderr(self.output))
        self.port = Mock()
        self.handler = object()
        self.events = []
        self.disable = Mock(side_effect=lambda _handler: self.events.append("disable"))
        self.read = Mock()
        self.stack.enter_context(patch.dict(recording["record_sequence"].__globals__, {
            "MOTION_FOLDER": self.folder,
            "connect": Mock(return_value=(self.port, self.handler)),
            "disable_all_torque": self.disable,
            "read_all": self.read,
        }))
        self.pose1 = dict(zip(settings.JOINT_NAMES, (2049, 2162, 1934, 2276, 1024)))
        self.pose2 = dict(zip(settings.JOINT_NAMES, (2051, 2050, 2047, 2052, 1707)))

    def record(self, commands, poses=None, names=None, action_names=None):
        self.read.side_effect = poses if poses is not None else [self.pose1, self.pose2]
        answers = iter([""] + (names if names is not None else ["demo"]))
        commands = iter(commands)
        action_names = iter(action_names) if action_names is not None else None

        def answer(prompt):
            self.events.append("input")
            if prompt.startswith("Command"):
                value = next(commands)
            elif prompt.startswith("Action name"):
                value = next(action_names) if action_names is not None else ""
            else:
                value = next(answers)
            if isinstance(value, BaseException):
                raise value
            return value

        with patch("builtins.input", side_effect=answer):
            return recording["record_sequence"]()

    def assert_closed(self):
        self.port.closePort.assert_called_once_with()

    def test_recorded_angles_round_trip_through_05_without_precision_loss(self):
        path = self.record(["q", "q", "e"])
        metadata, points = player["load_sequence"](path)
        self.assertEqual(points, [self.pose1, self.pose2])
        self.assertEqual(metadata["input_type"], "angles")
        self.assertEqual(metadata["schema_version"], 1)
        self.assertEqual(metadata["name"], "demo")
        self.assertEqual(metadata["description"], "")
        self.assertEqual(metadata["joint_order"], [
            {"servo_id": settings.SERVO_IDS[name], "joint": name}
            for name in settings.JOINT_NAMES
        ])
        self.assertEqual(set(metadata["points"][0]), {"input_values", "action_name"})
        values = metadata["points"][0]["input_values"]
        self.assertTrue(all(isinstance(value, str) for value in values))
        self.assertEqual(float(values[0]), 360 / 4096)
        self.assertEqual(float(values[4]), 90.0)  # Reversed gripper direction.
        self.assertEqual(self.events[:3], ["input", "disable", "input"])
        self.assertEqual(self.read.call_args_list, [call(self.handler), call(self.handler)])
        self.assert_closed()

    def test_e_before_two_points_keeps_recording_and_other_commands_do_not_read(self):
        self.record(["e", "x", "", "q", "e", "q", "e"])
        self.assertEqual(self.read.call_count, 2)
        self.assertEqual(self.output.getvalue().count("Record at least two"), 2)
        self.assertTrue((self.folder / "demo.json").exists())

    def test_invalid_names_and_existing_file_are_reprompted(self):
        existing = self.folder / "used.json"
        existing.write_text("original", encoding="utf-8")
        path = self.record(["q", "q", "e"], names=["../escape", "", "used", "demo"])
        self.assertEqual(path, self.folder / "demo.json")
        self.assertEqual(existing.read_text(encoding="utf-8"), "original")

    def test_out_of_limit_pose_is_rejected_as_a_whole(self):
        invalid = dict(self.pose1, shoulder=4095)
        path = self.record(["q", "q", "q", "e"], poses=[invalid, self.pose1, self.pose2])
        self.assertIn("shoulder", self.output.getvalue())
        self.assertEqual(player["load_sequence"](path)[1], [self.pose1, self.pose2])

    def test_eof_saves_completed_points(self):
        path = self.record(["q", "q", EOFError()])
        self.assertEqual(player["load_sequence"](path)[1], [self.pose1, self.pose2])
        self.assert_closed()

    def test_keyboard_interrupt_saves_completed_points(self):
        path = self.record(["q", "q", KeyboardInterrupt()])
        self.assertTrue(path.exists())
        self.assert_closed()

    def test_interruption_with_fewer_than_two_points_does_not_save(self):
        for commands in ([KeyboardInterrupt()], ["q", EOFError()]):
            with self.subTest(commands=commands):
                self.assertIsNone(self.record(commands))
                self.assertEqual(list(self.folder.iterdir()), [])

    def test_read_failure_saves_only_previously_completed_points(self):
        with self.assertRaisesRegex(RuntimeError, "serial failure"):
            self.record(["q", "q", "q"], poses=[
                self.pose1, self.pose2, RuntimeError("serial failure"),
            ])
        self.assertEqual(player["load_sequence"](self.folder / "demo.json")[1],
                         [self.pose1, self.pose2])
        self.assert_closed()

    def test_incomplete_read_failure_does_not_save_a_partial_point(self):
        with self.assertRaisesRegex(RuntimeError, "serial failure"):
            self.record(["q", "q"], poses=[self.pose1, RuntimeError("serial failure")])
        self.assertEqual(list(self.folder.iterdir()), [])
        self.assert_closed()

    def test_partial_servo_read_failure_never_appends_a_point(self):
        # Exercise the real sequential reader: three axes succeed, the fourth fails.
        with patch("arm_helpers.read_servo", side_effect=[2048, 2048, 2048, RuntimeError("axis 4")]):
            from arm_helpers import read_all
            with patch.dict(recording["read_angle_point"].__globals__, {"read_all": read_all}):
                points = []
                with patch("builtins.input", return_value="q"):
                    with self.assertRaisesRegex(RuntimeError, "axis 4"):
                        recording["record_points"](self.handler, points)
                self.assertEqual(points, [])

    def test_torque_failure_prevents_naming_and_closes_port(self):
        self.disable.side_effect = RuntimeError("torque failure")
        with self.assertRaisesRegex(RuntimeError, "torque failure"):
            self.record([])
        self.assertEqual(self.events, ["input"])
        self.port.closePort.assert_called_once()
        self.read.assert_not_called()

    def test_filename_cancellation_closes_port(self):
        with patch("builtins.input", side_effect=["", EOFError()]):
            with self.assertRaises(EOFError):
                recording["record_sequence"]()
        self.port.closePort.assert_called_once()

    def test_failed_save_cleans_temporary_file_and_reports_no_success(self):
        with patch("os.replace", side_effect=OSError("disk failure")):
            with self.assertRaisesRegex(OSError, "disk failure"):
                self.record(["q", "q", "e"])
        self.assertEqual(list(self.folder.iterdir()), [])
        self.assertNotIn("Saved 2", self.output.getvalue())
        self.assert_closed()

    def test_main_reports_save_error_and_exits_unsuccessfully(self):
        with patch.dict(recording["main"].__globals__, {
            "record_sequence": Mock(side_effect=OSError("disk failure")),
        }):
            with self.assertRaises(SystemExit) as error:
                recording["main"]()
        self.assertEqual(error.exception.code, 1)
        self.assertIn("disk failure", self.output.getvalue())
        self.assertNotIn("Saved", self.output.getvalue())

    def test_import_has_no_raw_terminal_dependency(self):
        with patch.dict("sys.modules", {"terminal_keys": None}):
            module = runpy.run_path("scripts/06_waypoint_recording.py", run_name="no_raw_terminal")
        self.assertNotIn("RawTerminal", module)

    def test_chinese_and_empty_names_are_saved_as_editable_json(self):
        path = self.record(["q", "q", "e"], action_names=["  夹住物体  ", "   "])
        metadata, _points = player["load_sequence"](path)
        self.assertEqual([point["action_name"] for point in metadata["points"]], ["夹住物体", ""])
        text = path.read_text(encoding="utf-8")
        self.assertIn("夹住物体", text)
        self.assertIn("步骤 2", self.output.getvalue())

    def test_repeated_poses_are_sampled_again_and_kept_as_independent_points(self):
        path = self.record([" Q ", " q ", " E "], poses=[self.pose1, self.pose1])
        metadata, points = player["load_sequence"](path)
        self.assertEqual(points, [self.pose1, self.pose1])
        self.assertEqual(len(metadata["points"]), 2)
        self.assertEqual(self.read.call_count, 2)

    def test_names_q_and_e_are_text_and_sampling_precedes_the_name_prompt(self):
        answers = iter(["q", "q", "q", "e", "e"])
        self.read.side_effect = [self.pose1, self.pose2]

        def answer(prompt):
            if prompt.startswith("Action name"):
                self.assertEqual(self.read.call_count, len(points) + 1)
            return next(answers)

        points = []
        with patch("builtins.input", side_effect=answer):
            recording["record_points"](self.handler, points)
        self.assertEqual([point["action_name"] for point in points], ["q", "e"])
        self.assertEqual(self.read.call_count, 2)

    def test_interrupted_name_discards_only_the_unconfirmed_point(self):
        for interruption in (KeyboardInterrupt(), EOFError()):
            with self.subTest(interruption=type(interruption).__name__):
                with patch("builtins.input", side_effect=["q", "", "q", interruption]):
                    self.read.side_effect = [self.pose1, self.pose2]
                    points = []
                    with self.assertRaises(type(interruption)):
                        recording["record_points"](self.handler, points)
                self.assertEqual(len(points), 1)
                self.assertEqual(points[0]["action_name"], "")

    def test_interrupt_during_naming_saves_previous_complete_points(self):
        path = self.record(["q", "q", "q"], poses=[self.pose1, self.pose2, self.pose1],
                           action_names=["first", "", KeyboardInterrupt()])
        self.assertEqual(player["load_sequence"](path)[1], [self.pose1, self.pose2])
        self.assert_closed()

    def test_save_refuses_a_file_created_during_recording(self):
        path = self.folder / "demo.json"
        path.write_text("original", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            recording["save_sequence"](path, {"points": []})
        self.assertEqual(path.read_text(encoding="utf-8"), "original")
        self.assertEqual(list(self.folder.iterdir()), [path])

    def test_import_does_not_connect_or_send_commands(self):
        with patch("arm_helpers.connect") as connect, patch("arm_helpers.disable_all_torque") as disable:
            runpy.run_path("scripts/06_waypoint_recording.py", run_name="import_only")
        connect.assert_not_called()
        disable.assert_not_called()


if __name__ == "__main__":
    unittest.main()
