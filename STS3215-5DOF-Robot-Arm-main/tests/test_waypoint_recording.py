from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import Mock, call, patch

import arm_settings as settings
from scservo_sdk import SMS_STS_TORQUE_ENABLE

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
        self.handler = Mock()
        self.gripper_id = 9
        self.stack.enter_context(patch.dict(settings.SERVO_IDS, {"gripper": self.gripper_id}))
        self.handler.WritePosEx.return_value = (0, 0)
        self.handler.write1ByteTxRx.return_value = (0, 0)
        self.handler.getTxRxResult.return_value = "serial failure"
        self.handler.ReadPos.side_effect = self.read_position
        self.events = []
        self.disable = Mock(side_effect=lambda _handler: self.events.append("disable"))
        self.read = Mock()
        self.stack.enter_context(patch.dict(recording["record_sequence"].__globals__, {
            "MOTION_FOLDER": self.folder,
            "connect": Mock(return_value=(self.port, self.handler)),
            "disable_all_torque": self.disable,
        }))
        self.pose1 = dict(zip(settings.JOINT_NAMES, (2049, 2162, 1934, 2276, 1024)))
        self.pose2 = dict(zip(settings.JOINT_NAMES, (2051, 2050, 2047, 2052, 1707)))

    def read_position(self, servo_id):
        if servo_id == self.gripper_id:
            return 2100, 0, 0
        if servo_id == settings.SERVO_IDS["base_yaw"]:
            self.current_pose = self.read()
        name = next(name for name, configured_id in settings.SERVO_IDS.items()
                    if configured_id == servo_id)
        return self.current_pose[name], 0, 0

    def record(self, commands, poses=None, names=None, action_names=None,
               angles=("-40", "0"), confirmations=("yes",)):
        self.read.side_effect = poses if poses is not None else [self.pose1, self.pose2]
        answers = iter([""] + (names if names is not None else ["demo"]))
        commands = iter(commands)
        angles = iter(angles)
        confirmations = iter(confirmations)
        action_names = iter(action_names) if action_names is not None else None

        def answer(prompt):
            self.events.append("input")
            if prompt.startswith("Command"):
                value = next(commands)
            elif prompt.startswith("Action name"):
                value = next(action_names) if action_names is not None else ""
                self.events.append(("name", self.read.call_count, self.handler.WritePosEx.call_count))
            elif prompt.startswith("Gripper"):
                value = next(angles)
            elif prompt.startswith("Support any object"):
                value = next(confirmations)
            else:
                value = next(answers)
            if isinstance(value, BaseException):
                raise value
            return value

        with patch("builtins.input", side_effect=answer):
            return recording["record_sequence"]()

    def assert_closed(self):
        self.port.closePort.assert_called_once_with()

    def grasped(self, pose):
        return dict(pose, gripper=2503)

    def test_recorded_angles_round_trip_through_05_without_precision_loss(self):
        path = self.record(["q", "w", "e"])
        metadata, points = player["load_sequence"](path)
        self.assertEqual(points, [self.grasped(self.pose1), dict(self.pose2, gripper=2048)])
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
        self.assertEqual(float(values[4]), -40.0)
        self.assertEqual(float(metadata["points"][1]["input_values"][4]), 0.0)
        self.assertEqual(self.events[:3], ["input", "disable", "input"])
        write = (settings.GRIPPER_RECORD_MOVE_SPEED, settings.GRIPPER_RECORD_MOVE_ACCELERATION)
        axes = [call.ReadPos(settings.SERVO_IDS[name]) for name in settings.JOINT_NAMES[:4]]
        self.assertEqual(self.handler.mock_calls, [
            *axes, call.ReadPos(self.gripper_id), call.WritePosEx(self.gripper_id, 2100, *write),
            call.write1ByteTxRx(self.gripper_id, SMS_STS_TORQUE_ENABLE, 1),
            call.WritePosEx(self.gripper_id, 2503, *write),
            *axes, call.WritePosEx(self.gripper_id, 2048, *write),
            call.write1ByteTxRx(self.gripper_id, SMS_STS_TORQUE_ENABLE, 0),
        ])
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
        self.assertEqual(player["load_sequence"](path)[1], [self.grasped(self.pose1), self.grasped(self.pose2)])
        self.assertEqual(self.handler.mock_calls[:2], [call.ReadPos(1), call.ReadPos(2)])
        self.assertEqual(self.handler.WritePosEx.call_count, 3)

    def test_eof_saves_completed_points(self):
        path = self.record(["q", "q", EOFError()])
        self.assertEqual(player["load_sequence"](path)[1], [self.grasped(self.pose1), self.grasped(self.pose2)])
        self.handler.write1ByteTxRx.assert_called_with(self.gripper_id, SMS_STS_TORQUE_ENABLE, 0)
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
                         [self.grasped(self.pose1), self.grasped(self.pose2)])
        self.assert_closed()

    def test_incomplete_read_failure_does_not_save_a_partial_point(self):
        with self.assertRaisesRegex(RuntimeError, "serial failure"):
            self.record(["q", "q"], poses=[self.pose1, RuntimeError("serial failure")])
        self.assertEqual(list(self.folder.iterdir()), [])
        self.assert_closed()

    def test_partial_servo_read_failure_never_appends_a_point(self):
        self.handler.ReadPos.side_effect = [(2048, 0, 0)] * 3 + [(0, -1, 0)]
        with self.assertRaisesRegex(RuntimeError, "Read servo 4"):
            self.record(["q"])
        self.assertEqual(list(self.folder.iterdir()), [])
        self.handler.WritePosEx.assert_not_called()
        self.handler.write1ByteTxRx.assert_not_called()

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
        self.assertEqual(points, [self.grasped(self.pose1)] * 2)
        self.assertEqual(len(metadata["points"]), 2)
        self.assertEqual(self.read.call_count, 2)

    def test_names_are_text_and_sampling_and_movement_precede_the_name_prompt(self):
        path = self.record(["q", "w", "e"], action_names=["q", "w"])
        metadata, _ = player["load_sequence"](path)
        self.assertEqual([point["action_name"] for point in metadata["points"]], ["q", "w"])
        self.assertEqual([event for event in self.events if isinstance(event, tuple)],
                         [("name", 1, 2), ("name", 2, 3)])

    def test_interrupted_name_discards_only_the_unconfirmed_point(self):
        for interruption in (KeyboardInterrupt(), EOFError()):
            with self.subTest(interruption=type(interruption).__name__):
                result = self.record(["q", "w"], action_names=["", interruption])
                self.assertIsNone(result)
                self.assertEqual(list(self.folder.iterdir()), [])
                self.handler.write1ByteTxRx.assert_called_with(
                    self.gripper_id, SMS_STS_TORQUE_ENABLE, 0
                )

    def test_interrupt_during_naming_saves_previous_complete_points(self):
        path = self.record(["q", "q", "q"], poses=[self.pose1, self.pose2, self.pose1],
                           action_names=["first", "", KeyboardInterrupt()])
        self.assertEqual(player["load_sequence"](path)[1], [self.grasped(self.pose1), self.grasped(self.pose2)])
        self.assert_closed()

    def test_invalid_angles_are_reprompted_and_decimal_target_is_saved(self):
        path = self.record(["w", "q", "e"], angles=("", "no", "NaN", "-91", "-40.5", "inf", "0.5"))
        output = self.output.getvalue()
        self.assertEqual(output.count("Invalid grasp angle"), 4)
        self.assertEqual(output.count("Invalid release angle"), 1)
        metadata, _ = player["load_sequence"](path)
        self.assertEqual([point["input_values"][4] for point in metadata["points"]], ["0.5", "-40.5"])

    def test_canceling_exit_keeps_gripper_active_and_recording_continues(self):
        path = self.record(["q", "w", "e", "q", "e"], poses=[self.pose1, self.pose2, self.pose1],
                           confirmations=("", "Y"))
        self.assertEqual(len(player["load_sequence"](path)[1]), 3)
        self.assertIn("Save canceled", self.output.getvalue())
        self.assertEqual(self.handler.write1ByteTxRx.call_args_list, [
            call(self.gripper_id, SMS_STS_TORQUE_ENABLE, 1),
            call(self.gripper_id, SMS_STS_TORQUE_ENABLE, 0),
        ])

    def test_target_write_failure_does_not_add_current_point_and_cleans_up(self):
        self.handler.WritePosEx.side_effect = [(0, 0)] * 3 + [(-1, 0)]
        with self.assertRaisesRegex(RuntimeError, "Set gripper target: serial failure"):
            self.record(["q", "w", "q"], poses=[self.pose1, self.pose2, self.pose1])
        _, points = player["load_sequence"](self.folder / "demo.json")
        self.assertEqual(points, [self.grasped(self.pose1), dict(self.pose2, gripper=2048)])
        self.handler.write1ByteTxRx.assert_called_with(self.gripper_id, SMS_STS_TORQUE_ENABLE, 0)
        self.assert_closed()

    def test_failed_enable_is_followed_by_torque_off_attempt(self):
        self.handler.write1ByteTxRx.side_effect = [(-1, 0), (0, 0)]
        with self.assertRaisesRegex(RuntimeError, "Enable gripper torque: serial failure"):
            self.record(["q"])
        self.assertEqual(list(self.folder.iterdir()), [])
        self.assertEqual(self.handler.WritePosEx.call_count, 1)
        self.handler.write1ByteTxRx.assert_called_with(self.gripper_id, SMS_STS_TORQUE_ENABLE, 0)
        self.assert_closed()

    def test_failed_torque_off_is_reported_and_complete_points_are_preserved(self):
        self.handler.write1ByteTxRx.side_effect = [(0, 0), (-1, 0)]
        with self.assertRaisesRegex(RuntimeError, "Gripper torque off could not be confirmed"):
            self.record(["q", "w", "e"])
        self.assertTrue((self.folder / "demo.json").exists())
        self.assertIn("Gripper torque off could not be confirmed", self.output.getvalue())
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
