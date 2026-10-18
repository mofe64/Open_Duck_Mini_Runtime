"""Check sensor offsets and idle standing with fake hardware."""

import ast
import importlib.util
from pathlib import Path
from queue import Queue
import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch

import numpy as np

from autonomy.protocol import Command, Mode
import test_imu_i2c
import test_walk_autonomy as harness
from test_walk_startup import load_walk_module


ROOT = Path(__file__).resolve().parents[1]


def load_reader():
    path = ROOT / "mini_bdx_runtime" / "mini_bdx_runtime" / "raw_imu.py"
    spec = importlib.util.spec_from_file_location("offset_test_reader", path)
    reader = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {
        "adafruit_bno055": ModuleType("adafruit_bno055"),
        "imu_i2c": test_imu_i2c.imu_i2c,
    }):
        spec.loader.exec_module(reader)
    return reader


def fake_imu(offset=(0, 0, 0)):
    reader = load_reader()
    imu = reader.Imu.__new__(reader.Imu)
    imu.accel_offset = np.array(offset, dtype=float)
    imu.last_imu_data = {"gyro": np.array([0.1, 0.2, 0.3]),
                         "accelero": np.array([1.3, -0.65, 9.7])}
    imu.imu_queue = Queue()
    return imu


class AccelerationOffsetTests(unittest.TestCase):
    def test_repeated_reads_apply_offset_once_without_changing_stored_data(self):
        imu = fake_imu((1.3, -0.65, 0.2))
        original = imu.last_imu_data["accelero"].copy()
        for _ in range(3):
            data = imu.get_data()
            np.testing.assert_allclose(data["accelero"], [0, 0, 9.5])
            np.testing.assert_array_equal(data["gyro"], [0.1, 0.2, 0.3])
            np.testing.assert_array_equal(imu.last_imu_data["accelero"], original)
        imu.imu_queue.put({"gyro": np.zeros(3), "accelero": np.array([2.3, 0.35, 10.7])})
        np.testing.assert_allclose(imu.get_data()["accelero"], [1, 1, 10.5])

    def test_zero_offset_returns_the_original_reading(self):
        imu = fake_imu()
        self.assertIs(imu.get_data(), imu.last_imu_data)

    def test_invalid_offset_fails_before_sensor_setup(self):
        reader = load_reader()
        for offset in ((0, 0), (np.nan, 0, 0), (0, np.inf, 0)):
            with self.subTest(offset=offset), patch.object(reader, "open_imu_i2c") as bus:
                with self.assertRaises(ValueError):
                    reader.Imu(50, accel_offset=offset)
                bus.assert_not_called()

    def test_policy_fall_reference_and_state_use_the_same_corrected_reading(self):
        module, walk, _controller, link = harness.WalkAutonomyTests().make_walk(freeze_head=True)
        walk.imu = fake_imu((1.3, -0.65, 0.2))
        with patch.object(module.time, "time", side_effect=[0, 0.1, 0.2, 0.6]), \
             patch.object(module.time, "sleep"):
            walk.upright_gravity = walk.capture_upright_gravity()
        np.testing.assert_allclose(walk.upright_gravity, [0, 0, 1])
        self.assertAlmostEqual(walk.get_tilt_deg(), 0)
        walk.hwi.set_position_all.side_effect = KeyboardInterrupt
        with patch.object(module.time, "monotonic", return_value=100):
            walk.run()
        np.testing.assert_allclose(walk.policy.infer.call_args.args[0][3:6], [0, 0, 9.5])
        np.testing.assert_allclose(link.send_state.call_args.args[3], [0, 0, 9.5])
        # Add the offset to a tilted reading to model the sensor's stored data.
        angle = np.radians(60)
        walk.imu.last_imu_data["accelero"] = (
            9.5 * np.array([np.sin(angle), 0, np.cos(angle)]) + walk.imu.accel_offset)
        self.assertAlmostEqual(walk.get_tilt_deg(), 60)
        self.assertFalse(walk.check_fall(10))
        self.assertTrue(walk.check_fall(10.25))

    def test_nonzero_pitch_bias_prints_a_clear_warning(self):
        with patch("builtins.print") as printed:
            harness.WalkAutonomyTests().make_walk(freeze_head=True, pitch_bias=3)
        self.assertTrue(any("--pitch_bias has no effect" in str(call) for call in printed.call_args_list))

    def test_walking_passes_the_offset_to_the_shared_reader(self):
        factory = harness.WalkAutonomyTests()
        factory.make_walk(accel_offset=(1.3, -0.2, 0.1), freeze_head=True)
        self.assertEqual(factory.created_imu_arguments["accel_offset"], (1.3, -0.2, 0.1))


class IdleStandingTests(unittest.TestCase):
    def setUp(self):
        self.module, self.walk, self.controller, self.link = harness.WalkAutonomyTests().make_walk(
            stand_when_idle=True, freeze_head=True)
        self.walk.feet_contacts.get.return_value = [True, True]
        self.walk.motor_targets += 0.25
        self.walk.motor_targets[5:9] = self.walk.init_pos[5:9]
        self.walk.last_commands = np.zeros(7)

    def enter_idle(self):
        self.assertFalse(self.walk.update_idle(0))
        self.assertTrue(self.walk.update_idle(0.5))

    def run_steps(self, count, on_step=None):
        clock = [100.0]
        writes = []
        sleeps = []

        def write(targets):
            writes.append((clock[0], dict(targets), self.walk.idle_state))

        def sleep(_seconds):
            sleeps.append(clock[0])
            if on_step:
                on_step(len(sleeps))
            if len(sleeps) >= count:
                raise KeyboardInterrupt
            clock[0] += 0.02

        self.walk.hwi.set_position_all.side_effect = write
        with patch.object(self.module.time, "time", side_effect=lambda: clock[0]), \
             patch.object(self.module.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(self.module.time, "sleep", side_effect=sleep), \
             patch.object(self.module, "make_action_dict", side_effect=lambda values, names: dict(zip(names, values))):
            self.walk.run()
        return writes

    def test_delay_and_smooth_move_end_at_the_standing_pose(self):
        start = self.walk.motor_targets.copy()
        self.assertFalse(self.walk.update_idle(0))
        self.assertFalse(self.walk.update_idle(0.499))
        self.assertTrue(self.walk.update_idle(0.5))
        np.testing.assert_array_equal(self.walk.motor_targets, start)
        self.assertTrue(self.walk.update_idle(0.75))
        np.testing.assert_allclose(self.walk.motor_targets, (start + self.walk.init_pos) / 2)
        self.walk.update_idle(1)
        self.assertEqual(self.walk.idle_state, "standing")
        self.walk.update_idle(5)
        np.testing.assert_array_equal(self.walk.motor_targets, self.walk.init_pos)

    def test_each_walking_axis_resets_the_zero_command_timer(self):
        for axis, limit in enumerate((0.01, 0.01, 0.05)):
            with self.subTest(axis=axis):
                self.walk.idle_zero_since = None
                self.walk.last_commands = np.zeros(7)
                self.walk.last_commands[axis] = limit
                self.assertFalse(self.walk.update_idle(0))
                self.walk.last_commands[axis] = -limit - 0.001
                self.assertFalse(self.walk.update_idle(0.4))
                self.walk.last_commands[axis] = 0
                self.assertFalse(self.walk.update_idle(0.5))
                self.assertFalse(self.walk.update_idle(0.9))

    def test_entry_waits_for_both_feet(self):
        self.walk.feet_contacts.get.return_value = [True, False]
        self.assertFalse(self.walk.update_idle(0))
        self.assertFalse(self.walk.update_idle(0.5))
        self.walk.feet_contacts.get.return_value = [True, True]
        self.assertTrue(self.walk.update_idle(0.6))
        self.assertTrue(self.walk.idle_use_contacts)

    def test_missing_contact_uses_the_smooth_move_after_the_wait(self):
        for contacts in ([False, False], None, [True], [np.nan, True]):
            with self.subTest(contacts=contacts):
                self.walk.idle_state = "walking"
                self.walk.idle_zero_since = None
                self.walk.feet_contacts.get.return_value = contacts
                self.assertFalse(self.walk.update_idle(0))
                self.assertFalse(self.walk.update_idle(1.49))
                self.assertTrue(self.walk.update_idle(1.5))
                self.assertFalse(self.walk.idle_use_contacts)
                self.walk.update_idle(2)
                np.testing.assert_array_equal(self.walk.motor_targets, self.walk.init_pos)

    def test_contact_loss_stops_move_time_until_contact_returns(self):
        self.enter_idle()
        self.walk.update_idle(0.6)
        targets = self.walk.motor_targets.copy()
        self.walk.feet_contacts.get.return_value = [False, True]
        self.walk.update_idle(0.7)
        self.walk.update_idle(0.9)
        np.testing.assert_array_equal(self.walk.motor_targets, targets)
        self.walk.feet_contacts.get.return_value = [True, True]
        self.walk.update_idle(1)
        self.assertGreater(self.walk.idle_ramp_elapsed, 0.1)

    def test_long_contact_loss_uses_the_smooth_move_alone(self):
        self.enter_idle()
        self.walk.feet_contacts.get.side_effect = OSError("No foot data.")
        self.walk.update_idle(0.6)
        self.walk.update_idle(1.7)
        self.assertFalse(self.walk.idle_use_contacts)
        self.walk.update_idle(2.2)
        self.assertEqual(self.walk.idle_state, "standing")

    def test_new_command_resets_all_walking_state_and_prints_once(self):
        for wake_time in (0.6, 1.5):
            with self.subTest(wake_time=wake_time), patch("builtins.print") as printed:
                self.walk.last_commands = np.zeros(7)
                self.walk.idle_state = "walking"
                self.walk.idle_zero_since = None
                self.enter_idle()
                self.walk.update_idle(wake_time)
                for name in ("last_action", "last_last_action", "last_last_last_action"):
                    setattr(self.walk, name, np.ones(14))
                self.walk.imitation_i = 40
                self.walk.imitation_phase = np.ones(2)
                old_filter = self.walk.action_filter = Mock()
                self.walk.action_filter_cutoff = 10
                self.walk.last_commands[0] = 0.02
                with patch.object(self.module, "LowPassActionFilter", return_value=Mock()) as factory, \
                     patch.object(self.module.time, "time", return_value=123):
                    self.assertFalse(self.walk.update_idle(wake_time + 0.1))
                factory.assert_called_once_with(50, 10)
                self.assertIsNot(self.walk.action_filter, old_filter)
                for name in ("last_action", "last_last_action", "last_last_last_action"):
                    np.testing.assert_array_equal(getattr(self.walk, name), np.zeros(14))
                np.testing.assert_array_equal(self.walk.motor_targets, self.walk.init_pos)
                np.testing.assert_array_equal(self.walk.prev_motor_targets, self.walk.init_pos)
                np.testing.assert_array_equal(self.walk.imitation_phase, [0, 0])
                self.assertEqual(self.walk.imitation_i, 0)
                self.assertEqual(self.walk.walk_start_t, 123)
                self.walk.update_idle(wake_time + 0.2)
                lines = [call.args[0] for call in printed.call_args_list]
                self.assertEqual(sum(line.startswith("Idle:") for line in lines), 1)
                self.assertEqual(sum(line.startswith("Walking:") for line in lines), 1)

    def test_idle_loop_skips_policy_and_phase_but_keeps_state_messages(self):
        writes = self.run_steps(70)
        self.assertEqual(self.walk.idle_state, "standing")
        self.assertLess(self.walk.policy.infer.call_count, 35)
        self.assertEqual(self.walk.imitation_i, self.walk.policy.infer.call_count)
        np.testing.assert_array_equal(list(writes[-1][1].values()), self.walk.init_pos)
        self.assertEqual(self.link.poll.call_count, 70)
        self.assertEqual(self.link.send_state.call_count, 70)
        self.walk.hwi.turn_on_smooth.assert_not_called()

    def test_without_flag_zero_commands_keep_running_the_policy(self):
        self.walk.stand_when_idle = False
        self.walk.update_idle = Mock(side_effect=AssertionError("Idle code ran without its flag."))
        self.run_steps(70)
        self.assertEqual(self.walk.policy.infer.call_count, 70)
        self.assertEqual(self.walk.imitation_i, 70)
        self.walk.update_idle.assert_not_called()

    def test_idle_standing_also_works_without_autonomy(self):
        self.module, self.walk, self.controller, self.link = harness.WalkAutonomyTests().make_walk(
            autonomy=False, stand_when_idle=True, freeze_head=True)
        self.walk.feet_contacts.get.return_value = [True, True]
        self.run_steps(70)
        self.assertEqual(self.walk.idle_state, "standing")
        self.link.poll.assert_not_called()
        self.link.send_state.assert_not_called()
        self.link.close.assert_not_called()

    def test_idle_still_stops_when_motor_feedback_is_missing(self):
        self.enter_idle()
        self.walk.hwi.get_present_positions.return_value = None
        with self.assertRaisesRegex(RuntimeError, "Motor feedback lost"):
            self.run_steps(1)
        self.walk.policy.infer.assert_not_called()
        self.walk.hwi.set_position_all.assert_not_called()
        self.walk.hwi.turn_off.assert_called_once()

    def test_pause_stops_idle_targets_and_does_not_count_paused_time(self):
        self.enter_idle()
        self.walk.update_idle(0.75)
        before = self.walk.motor_targets.copy()
        self.walk.paused = True
        self.run_steps(5)
        self.walk.hwi.set_position_all.assert_not_called()
        np.testing.assert_array_equal(self.walk.motor_targets, before)
        self.walk.paused = False
        self.walk.update_idle(1000)
        np.testing.assert_array_equal(self.walk.motor_targets, before)

    def test_fall_in_idle_turns_off_torque_and_sends_no_targets(self):
        self.enter_idle()
        self.walk.imu.get_data.return_value["accelero"] = np.array([9.81, 0, 0])
        self.walk.tilted_since = 99
        self.run_steps(5)
        self.assertTrue(self.walk.fallen)
        self.walk.policy.infer.assert_not_called()
        self.walk.hwi.set_position_all.assert_not_called()
        self.assertEqual(self.walk.hwi.turn_off.call_count, 2)

    def test_manual_head_commands_and_auto_head_hold_work_while_idle(self):
        self.walk.freeze_head = False
        self.walk.last_commands[3:] = [0.1, 0.2, 0.3, 0.4]
        self.enter_idle()
        self.walk.update_idle(1)
        np.testing.assert_allclose(self.walk.motor_targets[5:9], np.array(self.walk.init_pos[5:9]) + [0.1, 0.2, 0.3, 0.4])
        self.walk.autonomy_head_frozen = True
        self.walk.autonomy_head_targets = self.walk.motor_targets[5:9].copy()
        self.walk.last_commands[3:] = 0
        self.walk.update_idle(2)
        np.testing.assert_array_equal(self.walk.motor_targets[5:9], self.walk.autonomy_head_targets)

    def test_auto_command_wakes_idle_with_speed_caps_and_head_hold(self):
        self.enter_idle()
        self.walk.update_idle(1)
        self.walk.buttons = harness.buttons(rb=True)
        self.walk.autonomy_safety.accept(Command(1, 1, -1, 3), 100)
        with patch.object(self.module.time, "monotonic", return_value=100.1):
            self.walk.update_autonomy()
        self.assertEqual(self.walk.autonomy_safety.mode, Mode.AUTO)
        np.testing.assert_allclose(self.walk.last_commands[:3], [0.1, -0.05, 0.6])
        self.assertFalse(self.walk.update_idle(100.1))
        self.assertTrue(self.walk.autonomy_head_frozen)
        self.assertEqual(self.walk.imitation_i, 0)

    def test_watchdog_rb_and_stick_override_return_auto_to_idle(self):
        for exit_kind in ("watchdog", "rb", "stick"):
            with self.subTest(exit_kind=exit_kind):
                self.setUp()
                self.enter_idle()
                self.walk.buttons = harness.buttons(rb=True)
                self.walk.autonomy_safety.accept(Command(1, 0.05, 0, 0), 100)
                with patch.object(self.module.time, "monotonic", return_value=100.1):
                    self.walk.update_autonomy()
                self.assertFalse(self.walk.update_idle(100.1))
                self.walk.last_commands = np.zeros(7)
                self.walk.buttons = harness.buttons(rb=exit_kind == "rb")
                if exit_kind == "stick":
                    self.controller.get_stick_axes.return_value = (0.3, 0, 0, 0)
                now = 100.5 if exit_kind == "watchdog" else 100.2
                with patch.object(self.module.time, "monotonic", return_value=now):
                    self.walk.update_autonomy()
                self.assertEqual(self.walk.autonomy_safety.mode, Mode.MANUAL)
                np.testing.assert_array_equal(self.walk.last_commands[:3], [0, 0, 0])
                self.assertFalse(self.walk.update_idle(now))
                self.assertTrue(self.walk.update_idle(now + 0.5))

    def test_button_a_pauses_idle_without_motor_writes(self):
        self.enter_idle()
        self.controller.get_last_command.return_value = (np.zeros(7), harness.buttons(a=True), 0, 0)
        self.run_steps(1)
        self.assertTrue(self.walk.paused)
        self.walk.hwi.set_position_all.assert_not_called()
        self.assertEqual(self.link.send_state.call_args.args[0], Mode.PAUSED)

    def test_nonzero_command_restarts_policy_with_cleared_history(self):
        self.enter_idle()
        self.walk.update_idle(1)
        self.walk.last_action = np.ones(14)
        self.walk.imitation_i = 44
        self.controller.get_last_command.return_value = (
            np.array([0.02, 0, 0, 0, 0, 0, 0]), harness.buttons(), 0, 0)
        self.run_steps(1)
        obs = self.walk.policy.infer.call_args.args[0]
        np.testing.assert_array_equal(obs[41:83], np.zeros(42))
        np.testing.assert_array_equal(obs[-2:], [0, 0])
        self.assertEqual(self.walk.imitation_i, 1)

    def test_recovery_clears_idle_before_the_next_walk(self):
        self.enter_idle()
        self.walk.walk_kps = [30] * 14
        self.walk.imitation_phase = np.ones(2)
        self.walk.fallen = True
        self.walk.try_recover()
        self.assertFalse(self.walk.fallen)
        self.assertEqual(self.walk.idle_state, "walking")
        self.assertIsNone(self.walk.idle_zero_since)
        np.testing.assert_array_equal(self.walk.imitation_phase, [0, 0])


class WalkingFlagTests(unittest.TestCase):
    def parser(self):
        tree = ast.parse((ROOT / "scripts" / "v2_rl_walk_mujoco.py").read_text())
        main = tree.body[-1]
        statements = []
        for node in main.body:
            statements.append(node)
            if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "args" for target in node.targets):
                break
        scope = {"HOME_DIR": "/tmp"}
        with patch.object(sys, "argv", ["walk", "--onnx_model_path", "unused.onnx"]):
            exec(compile(ast.Module(body=statements, type_ignores=[]), "walk_flags", "exec"), scope)
        return scope["parser"]

    def test_flag_defaults_and_values(self):
        parser = self.parser()
        defaults = parser.parse_args(["--onnx_model_path", "unused.onnx"])
        self.assertEqual(defaults.accel_offset, (0, 0, 0))
        self.assertFalse(defaults.stand_when_idle)
        args = parser.parse_args(["--onnx_model_path", "unused.onnx", "--accel-offset", "-1.3", "0", "0",
                                  "--stand-when-idle", "--idle-delay", "0.7", "--idle-ramp-time", "0.8"])
        self.assertEqual(args.accel_offset, [-1.3, 0, 0])
        self.assertTrue(args.stand_when_idle)
        self.assertEqual(args.idle_delay, 0.7)
        self.assertEqual(args.idle_ramp_time, 0.8)
        self.assertIn("no effect with the raw IMU", parser.format_help())

    def test_invalid_settings_fail_before_hardware_setup(self):
        module = load_walk_module()
        cases = [dict(accel_offset=(np.nan, 0, 0)), dict(accel_offset=(0, 0)),
                 dict(idle_ramp_time=0), dict(stand_when_idle=True, replay_obs="unused.pkl"),
                 dict(stand_when_idle=True, control_freq=0)]
        for name in ("idle_delay", "idle_contact_wait", "idle_linear_deadband", "idle_turn_deadband"):
            cases.extend(({name: -0.1}, {name: np.inf}))
        for options in cases:
            with self.subTest(options=options), patch.object(module, "HWI") as hwi:
                with self.assertRaises(ValueError):
                    module.RLWalk("unused.onnx", **options)
                hwi.assert_not_called()


if __name__ == "__main__":
    unittest.main()
