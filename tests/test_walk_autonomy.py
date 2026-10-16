"""Test the optional walking connection with mocked hardware."""

import builtins
from contextlib import ExitStack
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from autonomy.protocol import Command, Mode
from autonomy.safety import SafetyControl
from test_walk_startup import load_walk_module


def buttons(a=False, rb=False):
    result = SimpleNamespace()
    for name in ("A", "B", "X", "Y", "LB", "RB", "dpad_up", "dpad_down"):
        setattr(result, name, SimpleNamespace(triggered=False, is_pressed=False))
    result.A.triggered, result.RB.triggered = a, rb
    return result


class WalkAutonomyTests(unittest.TestCase):
    def make_walk(self, autonomy=True, freeze_head=False, speaker=False,
                  startup_error=None, startup_command=None):
        module = load_walk_module()
        config = SimpleNamespace(imu_upside_down=True, start_paused=False,
                                 phase_frequency_factor_offset=0.0, speaker=speaker,
                                 antennas=False, eyes=False, projector=False)
        hwi = Mock()
        hwi.init_pos = {f"joint_{i}": float(i) for i in range(14)}
        hwi.joints = dict.fromkeys(hwi.init_pos)
        hwi.get_present_positions.return_value = np.zeros(14)
        hwi.get_present_velocities.return_value = np.zeros(14)
        imu = SimpleNamespace(get_data=Mock(return_value={"gyro": np.zeros(3), "accelero": np.array([0, 0, -9.81])}))
        controller = Mock()
        controller.get_stick_axes.return_value = (0.0,) * 4
        controller.get_last_command.return_value = (np.zeros(7), buttons(), 0, 0)
        policy = SimpleNamespace(infer=Mock(return_value=np.ones(14)))
        feet = Mock()
        feet.get.return_value = [False, False]
        link = Mock(healthy=True)
        self.created_hwi, self.created_link = hwi, link
        def configure(walk, *_args):
            walk.autonomy_safety = SafetyControl()
            walk.autonomy_link = link
            if startup_command is not None:
                walk.autonomy_safety.accept(startup_command, 100)
        with ExitStack() as stack:
            replacements = {"DuckConfig": config, "HWI": hwi, "Imu": imu,
                            "OnnxInfer": policy, "FeetContacts": feet,
                            "XBoxController": controller, "PolyReferenceMotion": SimpleNamespace(nb_steps_in_period=100),
                            "Sounds": Mock()}
            for name, value in replacements.items():
                stack.enter_context(patch.object(module, name, return_value=value))
            stack.enter_context(patch.object(module.RLWalk, "start"))
            stack.enter_context(patch.object(module.RLWalk, "capture_upright_gravity",
                                            return_value=np.array([0, 0, -1]), side_effect=startup_error))
            stack.enter_context(patch.object(module.RLWalk, "configure_autonomy", configure))
            walk = module.RLWalk("unused.onnx", commands=True, autonomy=autonomy, freeze_head=freeze_head)
        return module, walk, controller, link

    def arm(self, walk, controller, manual=None):
        source = np.array(manual if manual is not None else [0.02, 0, 0, 0.4, 0.3, 0.2, 0.1])
        walk.last_commands = source
        walk.buttons = buttons(rb=True)
        controller.get_last_command.return_value = (source, walk.buttons, 0, 0)
        walk.autonomy_safety.accept(Command(1, 5, -5, 5), 100)
        return source

    def test_auto_uses_caps_zeroes_head_and_keeps_controller_values(self):
        module, walk, controller, link = self.make_walk()
        source = self.arm(walk, controller)
        previous = walk.motor_targets[5:9].copy()
        with patch.object(module.time, "monotonic", return_value=100.1):
            walk.update_autonomy()
        np.testing.assert_array_equal(walk.last_commands, [0.1, -0.05, 0.6, 0, 0, 0, 0])
        np.testing.assert_array_equal(source, [0.02, 0, 0, 0.4, 0.3, 0.2, 0.1])
        np.testing.assert_array_equal(walk.autonomy_head_targets, previous)
        self.assertEqual(link.send_state.call_args.args[:2], (Mode.AUTO, (0.1, -0.05, 0.6)))

    def test_auto_blocks_policy_head_motion_without_freeze_head_flag(self):
        module, walk, controller, _link = self.make_walk()
        self.arm(walk, controller)
        walk.motor_targets[5:9] = [9.1, 9.2, 9.3, 9.4]
        expected = walk.motor_targets[5:9].copy()
        sent = {}
        def capture_and_stop(targets):
            sent.update(targets)
            raise KeyboardInterrupt
        walk.hwi.set_position_all.side_effect = capture_and_stop
        with patch.object(module.time, "monotonic", return_value=100.1), \
             patch.object(module, "make_action_dict", side_effect=lambda values, names: dict(zip(names, values))):
            walk.run()
        np.testing.assert_array_equal([sent[f"joint_{i}"] for i in range(5, 9)], expected)
        np.testing.assert_array_equal(walk.last_action[5:9], np.zeros(4))
        obs = walk.policy.infer.call_args.args[0]
        np.testing.assert_array_equal(obs[9:13], np.zeros(4))
        self.assertEqual(sent["joint_4"], walk.init_pos[4] + 0.25)

    def test_global_freeze_head_keeps_the_start_pose(self):
        module, walk, controller, _link = self.make_walk(freeze_head=True)
        self.arm(walk, controller)
        walk.motor_targets[5:9] = [9.1, 9.2, 9.3, 9.4]
        sent = {}
        def capture_and_stop(targets):
            sent.update(targets)
            raise KeyboardInterrupt
        walk.hwi.set_position_all.side_effect = capture_and_stop
        with patch.object(module.time, "monotonic", return_value=100.1), \
             patch.object(module, "make_action_dict", side_effect=lambda values, names: dict(zip(names, values))):
            walk.run()
        np.testing.assert_array_equal([sent[f"joint_{i}"] for i in range(5, 9)], walk.init_pos[5:9])

    def test_head_control_stick_override_uses_raw_travel(self):
        module, walk, controller, _link = self.make_walk()
        self.arm(walk, controller)
        with patch.object(module.time, "monotonic", return_value=100.1):
            walk.update_autonomy()
        walk.last_commands = np.array([0, 0, 0, 0, 0.5, 0.4, 0.3])
        walk.buttons = buttons()
        controller.get_stick_axes.return_value = (0.25, 0, 0, 0)
        with patch.object(module.time, "monotonic", return_value=100.2):
            walk.update_autonomy()
        self.assertEqual(walk.autonomy_safety.mode, Mode.MANUAL)
        np.testing.assert_array_equal(walk.last_commands[:3], [0, 0, 0])
        np.testing.assert_array_equal(walk.last_commands[3:], [0, 0.5, 0.4, 0.3])
        self.assertFalse(walk.autonomy_head_frozen)

    def test_mode_sounds_are_short_and_nonblocking(self):
        module, walk, controller, _link = self.make_walk(speaker=True)
        self.arm(walk, controller)
        with patch.object(module.time, "monotonic", return_value=100.1):
            walk.update_autonomy()
        walk.sounds.play.assert_called_once_with("beep1.wav", maxtime_ms=150)
        walk.last_commands = np.zeros(7)
        with patch.object(module.time, "monotonic", return_value=100.2):
            walk.update_autonomy()
        self.assertEqual(walk.sounds.play.call_count, 2)
        walk.sounds.play.assert_called_with("beep1.wav", maxtime_ms=150)

    def test_no_autonomy_flag_never_imports_or_calls_autonomy(self):
        real_import = builtins.__import__
        def guard(name, *args, **kwargs):
            if name == "autonomy" or name.startswith("autonomy."):
                raise AssertionError("Autonomy code ran without its flag.")
            return real_import(name, *args, **kwargs)
        with patch("builtins.__import__", side_effect=guard):
            module, walk, _controller, link = self.make_walk(autonomy=False)
            walk.paused = True
            walk.update_autonomy = Mock(side_effect=AssertionError("Unexpected autonomy call."))
            with patch.object(module.time, "sleep", side_effect=KeyboardInterrupt):
                walk.run()
        walk.update_autonomy.assert_not_called()
        link.poll.assert_not_called()
        link.send_state.assert_not_called()
        link.close.assert_not_called()
        self.assertFalse(hasattr(walk, "autonomy_safety"))

    def test_startup_warning_only_without_freeze_head(self):
        with patch("builtins.print") as printed:
            self.make_walk(freeze_head=False)
        self.assertTrue(any("WARNING: --freeze-head is off" in str(call) for call in printed.call_args_list))
        with patch("builtins.print") as printed:
            self.make_walk(freeze_head=True)
        self.assertFalse(any("WARNING" in str(call) for call in printed.call_args_list))

    def test_autonomy_rejects_options_that_bypass_safety_before_hardware_setup(self):
        module = load_walk_module()
        for flags in (dict(commands=False), dict(fall_detection=False), dict(replay_obs="unused.pkl"),
                      dict(autonomy_forward_cap=float("nan")), dict(autonomy_port=0), dict(control_freq=0)):
            with patch.object(module, "HWI") as hwi, self.assertRaises(ValueError):
                module.RLWalk("unused.onnx", autonomy=True, **flags)
            hwi.assert_not_called()

    def test_commands_from_startup_do_not_allow_auto_entry(self):
        _module, walk, _controller, link = self.make_walk(startup_command=Command(1, 0.05, 0, 0))
        self.assertEqual(walk.autonomy_safety.step(100.1, rb=True).mode, Mode.MANUAL)
        link.discard_waiting.assert_called_once()

    def test_failed_autonomy_startup_closes_link_and_disables_mock_torque(self):
        with self.assertRaisesRegex(RuntimeError, "Reference failed"):
            self.make_walk(startup_error=RuntimeError("Reference failed."))
        self.created_hwi.turn_off.assert_called_once()
        self.created_link.close.assert_called_once()

    def test_a_pause_and_fall_still_bypass_policy_and_publish_zero_speeds(self):
        for fallen, paused in ((False, True), (True, False)):
            module, walk, controller, link = self.make_walk()
            self.arm(walk, controller)
            walk.fallen, walk.paused = fallen, paused
            walk.buttons.RB.triggered = False
            with patch.object(module.time, "monotonic", return_value=100.1), \
                 patch.object(module.time, "sleep", side_effect=KeyboardInterrupt):
                walk.run()
            walk.policy.infer.assert_not_called()
            walk.hwi.set_position_all.assert_not_called()
            self.assertEqual(link.send_state.call_args.args[1], (0, 0, 0))
            link.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
