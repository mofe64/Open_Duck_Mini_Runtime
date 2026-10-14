import importlib.util
from collections import defaultdict
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

import numpy  # Keep the module loaded while mocked imports are restored.


ROOT = Path(__file__).resolve().parents[1]


def load_hwi_module():
    rustypot = types.ModuleType("rustypot")
    config_module = types.ModuleType("mini_bdx_runtime.duck_config")
    config_module.DuckConfig = object
    package = types.ModuleType("mini_bdx_runtime")
    package.__path__ = []
    spec = importlib.util.spec_from_file_location(
        "walk_test_hwi",
        ROOT / "mini_bdx_runtime" / "mini_bdx_runtime" / "rustypot_position_hwi.py",
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(
        sys.modules,
        {
            "rustypot": rustypot,
            "mini_bdx_runtime": package,
            "mini_bdx_runtime.duck_config": config_module,
        },
    ):
        spec.loader.exec_module(module)
    return module, rustypot


class FakeMotorBus:
    def __init__(self, stuck=False):
        self.positions = {13: 0.2}
        self.goals = {}
        self.enabled = False
        self.stuck = stuck
        self.calls = []

    def read_present_position(self, ids):
        return [self.positions.get(motor_id, 0.0) for motor_id in ids]

    def write_goal_position(self, ids, positions):
        self.calls.append(("goal", list(ids), list(positions)))
        self.goals.update(zip(ids, positions))
        if self.enabled and not self.stuck:
            self.positions.update(self.goals)

    def set_kps(self, ids, gains):
        self.calls.append(("gains", list(ids), list(gains)))

    def enable_torque(self, ids):
        self.calls.append(("enable", list(ids)))
        self.enabled = True
        if not self.stuck:
            self.positions.update(self.goals)

    def disable_torque(self, ids):
        self.calls.append(("disable", list(ids)))
        self.enabled = False


class SmoothStartupTests(unittest.TestCase):
    def setUp(self):
        self.module, rustypot = load_hwi_module()
        self.bus = FakeMotorBus()
        rustypot.feetech = lambda *args: self.bus
        config = types.SimpleNamespace(
            joints_offset=defaultdict(float, {"left_knee": 0.2})
        )
        self.hwi = self.module.HWI(config)
        self.final_gains = [30] * 14
        self.final_gains[5:9] = [8] * 4

    def test_moves_from_current_pose_with_offsets_before_raising_gains(self):
        with patch.object(self.module.time, "sleep"):
            self.hwi.turn_on_smooth(self.final_gains, ramp_seconds=0.1)

        self.assertTrue(self.bus.enabled)
        self.assertEqual(self.bus.calls[0], ("gains", list(self.hwi.joints.values()), [8] * 14))
        self.assertEqual(self.bus.calls[1][0], "goal")
        self.assertEqual(self.bus.calls[2][0], "enable")
        self.assertAlmostEqual(self.bus.goals[13], 1.368 + 0.2)
        self.assertEqual(self.bus.calls[-1][2], self.final_gains)

    def test_tracking_failure_releases_all_motor_torque(self):
        self.bus.stuck = True
        with patch.object(self.module.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "not following its target"):
                self.hwi.turn_on_smooth(self.final_gains, ramp_seconds=0.1)
        self.assertFalse(self.bus.enabled)
        self.assertEqual(self.bus.calls[-1][0], "disable")


def load_walk_module():
    package = types.ModuleType("mini_bdx_runtime")
    package.__path__ = []
    modules = {"mini_bdx_runtime": package}
    imports = {
        "rustypot_position_hwi": "HWI",
        "onnx_infer": "OnnxInfer",
        "raw_imu": "Imu",
        "poly_reference_motion": "PolyReferenceMotion",
        "xbox_controller": "XBoxController",
        "feet_contacts": "FeetContacts",
        "eyes": "Eyes",
        "sounds": "Sounds",
        "antennas": "Antennas",
        "projector": "Projector",
        "duck_config": "DuckConfig",
    }
    for module_name, class_name in imports.items():
        module = types.ModuleType(f"mini_bdx_runtime.{module_name}")
        setattr(module, class_name, object)
        modules[module.__name__] = module
    utils = types.ModuleType("mini_bdx_runtime.rl_utils")
    utils.make_action_dict = lambda *args: {}
    utils.LowPassActionFilter = object
    modules[utils.__name__] = utils

    spec = importlib.util.spec_from_file_location(
        "walk_test_script", ROOT / "scripts" / "v2_rl_walk_mujoco.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class WalkShutdownTests(unittest.TestCase):
    def setUp(self):
        self.module = load_walk_module()
        self.walk = self.module.RLWalk.__new__(self.module.RLWalk)
        self.walk.hwi = types.SimpleNamespace(turn_off=Mock())
        self.walk.hwi.get_present_positions = Mock(return_value=numpy.zeros(14))
        self.walk.hwi.get_present_velocities = Mock(return_value=numpy.zeros(14))
        self.walk.feet_contacts = types.SimpleNamespace(stop=Mock())
        self.walk.duck_config = types.SimpleNamespace(
            antennas=False, eyes=False, projector=False
        )
        self.walk.commands = False
        self.walk.save_obs = False
        self.walk.control_freq = 50
        self.walk.fall_detection = False
        self.walk.fallen = False

    def test_ctrl_c_releases_motor_torque(self):
        self.walk.paused = True
        with patch.object(self.module.time, "sleep", side_effect=KeyboardInterrupt):
            self.walk.run()
        self.walk.hwi.turn_off.assert_called_once()
        self.walk.feet_contacts.stop.assert_called_once()

    def test_control_error_releases_motor_torque(self):
        self.walk.paused = False
        self.walk.get_obs = Mock(side_effect=RuntimeError("sensor failed"))
        with self.assertRaisesRegex(RuntimeError, "sensor failed"):
            self.walk.run()
        self.walk.hwi.turn_off.assert_called_once()
        self.walk.feet_contacts.stop.assert_called_once()

    def test_missing_motor_feedback_stops_instead_of_retrying(self):
        self.walk.paused = False
        self.walk.get_obs = Mock(return_value=None)
        with self.assertRaisesRegex(RuntimeError, "Motor feedback lost"):
            self.walk.run()
        self.walk.get_obs.assert_called_once()
        self.walk.hwi.turn_off.assert_called_once()

    def test_missing_positions_skip_velocity_read(self):
        self.walk.imu = types.SimpleNamespace(get_data=Mock(return_value={}))
        self.walk.hwi.get_present_positions.return_value = None
        self.assertIsNone(self.walk.get_obs())
        self.walk.hwi.get_present_velocities.assert_not_called()

    def test_missing_motor_feedback_while_paused_stops(self):
        self.walk.paused = True
        self.walk.hwi.get_present_positions.return_value = None
        with self.assertRaisesRegex(RuntimeError, "Motor feedback lost while paused"):
            self.walk.run()
        self.walk.hwi.turn_off.assert_called_once()

    def test_failed_torque_off_reports_manual_power_cut(self):
        self.walk.paused = True
        self.walk.hwi.turn_off.side_effect = OSError("I/O error")
        with patch.object(self.module.time, "sleep", side_effect=KeyboardInterrupt):
            with self.assertRaisesRegex(RuntimeError, "torque could not be disabled"):
                self.walk.run()
        self.walk.feet_contacts.stop.assert_called_once()

    def test_freeze_head_ignores_policy_and_controller_head_commands(self):
        self.walk.freeze_head = True
        self.walk.paused = False
        self.walk.init_pos = numpy.arange(14, dtype=float)
        self.walk.last_commands = numpy.array([0, 0, 0, 0.5, 0.6, 0.7, 0.8])
        self.walk.policy = types.SimpleNamespace(infer=Mock(return_value=numpy.ones(14)))
        self.walk.get_obs = Mock(return_value=numpy.zeros(100))
        self.walk.last_action = numpy.zeros(14)
        self.walk.last_last_action = numpy.zeros(14)
        self.walk.last_last_last_action = numpy.zeros(14)
        self.walk.phase_frequency_factor = 1.0
        self.walk.phase_frequency_factor_offset = 0.0
        self.walk.PRM = types.SimpleNamespace(nb_steps_in_period=100)
        self.walk.imitation_i = 0
        self.walk.action_filter = None
        self.walk.action_scale = 0.25
        self.walk.replay_obs = None
        names = [f"joint_{i}" for i in range(14)]
        self.walk.hwi.joints = dict.fromkeys(names)
        sent = {}

        def capture_and_stop(targets):
            sent.update(targets)
            raise KeyboardInterrupt

        self.walk.hwi.set_position_all = Mock(side_effect=capture_and_stop)
        with patch.object(
            self.module,
            "make_action_dict",
            side_effect=lambda targets, joints: dict(zip(joints, targets)),
        ):
            self.walk.run()

        for i in range(5, 9):
            self.assertEqual(sent[f"joint_{i}"], self.walk.init_pos[i])
        self.assertEqual(sent["joint_4"], self.walk.init_pos[4] + 0.25)
        self.assertEqual(sent["joint_9"], self.walk.init_pos[9] + 0.25)
        numpy.testing.assert_array_equal(self.walk.last_action[5:9], numpy.zeros(4))
        self.walk.hwi.turn_off.assert_called_once()

    def test_freeze_head_hides_controller_head_commands_from_policy(self):
        self.walk.freeze_head = True
        self.walk.num_dofs = 14
        self.walk.last_commands = numpy.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        self.walk.imu = types.SimpleNamespace(
            get_data=Mock(
                return_value={"gyro": numpy.zeros(3), "accelero": numpy.zeros(3)}
            )
        )
        self.walk.init_pos = numpy.zeros(14)
        self.walk.last_action = numpy.zeros(14)
        self.walk.last_last_action = numpy.zeros(14)
        self.walk.last_last_last_action = numpy.zeros(14)
        self.walk.motor_targets = numpy.zeros(14)
        self.walk.imitation_phase = numpy.zeros(2)
        self.walk.feet_contacts.get = Mock(return_value=[False, False])

        obs = self.walk.get_obs()

        numpy.testing.assert_array_equal(
            obs[6:13], [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0]
        )
        numpy.testing.assert_array_equal(
            self.walk.last_commands, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        )


class FallDetectionTests(unittest.TestCase):
    def setUp(self):
        self.module = load_walk_module()
        self.walk = self.module.RLWalk.__new__(self.module.RLWalk)
        self.accelero = numpy.array([0.0, 0.0, -9.8])
        self.walk.imu = types.SimpleNamespace(
            get_data=lambda: {"gyro": numpy.zeros(3), "accelero": self.accelero}
        )
        self.walk.upright_gravity = numpy.array([0.0, 0.0, -1.0])
        self.walk.fall_detection = True
        self.walk.fall_tilt_deg = 45.0
        self.walk.fall_time = 0.2
        self.walk.recover_tilt_deg = 20.0
        self.walk.recover_ramp_seconds = 3.0
        self.walk.fallen = False
        self.walk.tilted_since = None
        self.walk.commands = True
        self.walk.duck_config = types.SimpleNamespace(
            speaker=False, antennas=False, eyes=False, projector=False
        )
        self.walk.hwi = types.SimpleNamespace(turn_off=Mock(), turn_on_smooth=Mock())
        self.walk.walk_kps = [30] * 14
        self.walk.num_dofs = 14
        self.walk.init_pos = list(numpy.arange(14, dtype=float))
        self.walk.action_filter = None
        self.walk.control_freq = 50
        self.walk.paused = True

    def tip_over(self, degrees):
        angle = numpy.radians(degrees)
        self.accelero = 9.8 * numpy.array([numpy.sin(angle), 0.0, -numpy.cos(angle)])

    def test_upright_is_zero_tilt(self):
        self.assertAlmostEqual(self.walk.get_tilt_deg(), 0.0)

    def test_brief_tilt_is_ignored(self):
        self.tip_over(80)
        self.assertFalse(self.walk.check_fall(10.0))
        self.assertFalse(self.walk.check_fall(10.1))
        self.tip_over(5)
        self.assertFalse(self.walk.check_fall(10.15))
        self.tip_over(80)
        self.assertFalse(self.walk.check_fall(10.3))

    def test_sustained_tilt_is_a_fall(self):
        self.tip_over(60)
        self.assertFalse(self.walk.check_fall(10.0))
        self.assertTrue(self.walk.check_fall(10.25))

    def test_small_tilt_never_triggers(self):
        self.tip_over(30)
        for step in range(100):
            self.assertFalse(self.walk.check_fall(step * 0.02))

    def test_fall_cuts_torque(self):
        self.walk.handle_fall()
        self.walk.hwi.turn_off.assert_called_once()
        self.assertTrue(self.walk.fallen)

    def test_recover_refused_while_still_tilted(self):
        self.walk.fallen = True
        self.tip_over(50)
        self.walk.try_recover()
        self.walk.hwi.turn_on_smooth.assert_not_called()
        self.assertTrue(self.walk.fallen)

    def test_recover_ramps_back_and_resets_policy_state(self):
        self.walk.fallen = True
        self.walk.last_action = numpy.ones(14)
        self.walk.imitation_i = 37
        self.tip_over(10)
        self.walk.try_recover()
        self.walk.hwi.turn_on_smooth.assert_called_once()
        self.assertEqual(
            self.walk.hwi.turn_on_smooth.call_args.kwargs["ramp_seconds"], 3.0
        )
        self.assertFalse(self.walk.fallen)
        self.assertFalse(self.walk.paused)
        numpy.testing.assert_array_equal(self.walk.last_action, numpy.zeros(14))
        numpy.testing.assert_array_equal(self.walk.motor_targets, self.walk.init_pos)
        self.assertEqual(self.walk.imitation_i, 0)

    def test_failed_recovery_stays_fallen(self):
        self.walk.fallen = True
        self.walk.hwi.turn_on_smooth.side_effect = RuntimeError("left_knee is not following")
        self.walk.try_recover()
        self.assertTrue(self.walk.fallen)

    def test_run_loop_stops_commanding_motors_after_fall(self):
        self.walk.paused = False
        self.walk.commands = False
        self.walk.save_obs = False
        self.walk.feet_contacts = types.SimpleNamespace(stop=Mock())
        self.walk.get_obs = Mock(return_value=numpy.zeros(100))
        self.walk.hwi.set_position_all = Mock()
        self.tip_over(90)
        self.walk.tilted_since = -1.0  # already tilted for a second
        clock = iter(numpy.arange(0, 100, 0.05))
        sleeps = []

        def fake_sleep(seconds):
            sleeps.append(seconds)
            if len(sleeps) > 20:
                raise KeyboardInterrupt

        with patch.object(self.module.time, "time", side_effect=lambda: next(clock)):
            with patch.object(self.module.time, "sleep", side_effect=fake_sleep):
                self.walk.run()

        self.assertTrue(self.walk.fallen)
        self.walk.get_obs.assert_not_called()
        self.walk.hwi.set_position_all.assert_not_called()
        # Once when the fall is detected, once more on exit.
        self.assertEqual(self.walk.hwi.turn_off.call_count, 2)


if __name__ == "__main__":
    unittest.main()
