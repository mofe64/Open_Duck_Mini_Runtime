import time
import pickle
import sys

import numpy as np
from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.onnx_infer import OnnxInfer

from mini_bdx_runtime.raw_imu import Imu
from mini_bdx_runtime.poly_reference_motion import PolyReferenceMotion
from mini_bdx_runtime.xbox_controller import XBoxController
from mini_bdx_runtime.feet_contacts import FeetContacts
from mini_bdx_runtime.eyes import Eyes
from mini_bdx_runtime.sounds import Sounds
from mini_bdx_runtime.antennas import Antennas
from mini_bdx_runtime.projector import Projector
from mini_bdx_runtime.rl_utils import make_action_dict, LowPassActionFilter
from mini_bdx_runtime.duck_config import DuckConfig

import os

HOME_DIR = os.path.expanduser("~")


class RLWalk:
    def __init__(
        self,
        onnx_model_path: str,
        duck_config_path: str = f"{HOME_DIR}/duck_config.json",
        serial_port: str = "/dev/ttyACM0",
        control_freq: float = 50,
        pid=[30, 0, 0],
        action_scale=0.25,
        commands=False,
        pitch_bias=0,
        save_obs=False,
        replay_obs=None,
        cutoff_frequency=None,
        freeze_head=False,
        fall_detection=True,
        fall_tilt_deg=45.0,
        fall_time=0.2,
        recover_tilt_deg=20.0,
        recover_ramp_seconds=3.0,
        autonomy=False,
        autonomy_bind="::",
        autonomy_port=5602,
        autonomy_forward_cap=0.10,
        autonomy_sideways_cap=0.05,
        autonomy_turn_cap=0.6,
        accel_offset=(0, 0, 0),
        stand_when_idle=False,
        idle_linear_deadband=0.01,
        idle_turn_deadband=0.05,
        idle_delay=0.5,
        idle_ramp_time=0.5,
        idle_contact_wait=1.0,
    ):
        offset = np.asarray(accel_offset, dtype=float)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("Acceleration offsets must contain three finite values.")
        for value in (idle_linear_deadband, idle_turn_deadband, idle_delay, idle_contact_wait):
            if not np.isfinite(value) or value < 0:
                raise ValueError("Idle limits and wait times must be finite and at least zero.")
        if not np.isfinite(idle_ramp_time) or idle_ramp_time <= 0:
            raise ValueError("The idle move time must be finite and greater than zero.")
        if stand_when_idle and replay_obs is not None:
            raise ValueError("Idle standing cannot use saved policy inputs.")
        if stand_when_idle and (not np.isfinite(control_freq) or control_freq <= 0):
            raise ValueError("Idle standing needs a control rate greater than zero.")
        if pitch_bias != 0:
            print("WARNING: --pitch_bias has no effect with the raw IMU. Use --accel-offset X Y Z.", flush=True)

        self.stand_when_idle = stand_when_idle
        self.idle_linear_deadband = idle_linear_deadband
        self.idle_turn_deadband = idle_turn_deadband
        self.idle_delay = idle_delay
        self.idle_ramp_time = idle_ramp_time
        self.idle_contact_wait = idle_contact_wait
        self.idle_state = "walking"
        self.idle_zero_since = None
        self.idle_last_t = None
        self.idle_contact_lost_since = None
        self.autonomy_enabled = autonomy
        self.autonomy_head_frozen = False
        if autonomy:
            if not commands or not fall_detection or replay_obs is not None:
                raise ValueError("Autonomy needs the controller and fall detection. Do not use observation replay.")
            caps = (autonomy_forward_cap, autonomy_sideways_cap, autonomy_turn_cap)
            if not all(np.isfinite(cap) and cap >= 0 for cap in caps):
                raise ValueError("Autonomy speed caps must be finite and non-negative.")
            if not 1 <= autonomy_port <= 65535:
                raise ValueError("Autonomy port is not valid.")
            if not np.isfinite(control_freq) or control_freq <= 2:
                raise ValueError("Autonomy needs more than two control steps per second.")
            if not freeze_head:
                print("WARNING: --freeze-head is off. Auto holds the last head pose. Manual mode can move the head.", flush=True)

        self.duck_config = DuckConfig(config_json_path=duck_config_path)

        self.commands = commands
        self.freeze_head = freeze_head
        self.pitch_bias = pitch_bias

        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(self.onnx_model_path, awd=True)

        self.num_dofs = 14
        self.max_motor_velocity = 5.24  # rad/s

        # Control
        self.control_freq = control_freq
        self.pid = pid

        self.save_obs = save_obs
        if self.save_obs:
            self.saved_obs = []

        self.replay_obs = replay_obs
        if self.replay_obs is not None:
            self.replay_obs = pickle.load(open(self.replay_obs, "rb"))

        self.action_filter = None
        self.action_filter_cutoff = cutoff_frequency
        if cutoff_frequency is not None:
            self.action_filter = LowPassActionFilter(
                self.control_freq, cutoff_frequency
            )

        self.hwi = HWI(self.duck_config, serial_port)

        self.imu = Imu(
            sampling_freq=int(self.control_freq),
            user_pitch_bias=self.pitch_bias,
            upside_down=self.duck_config.imu_upside_down,
            accel_offset=accel_offset,
        )

        self.feet_contacts = FeetContacts()

        # Scales
        self.action_scale = action_scale

        self.last_action = np.zeros(self.num_dofs)
        self.last_last_action = np.zeros(self.num_dofs)
        self.last_last_last_action = np.zeros(self.num_dofs)

        self.init_pos = list(self.hwi.init_pos.values())

        self.motor_targets = np.array(self.init_pos.copy())
        self.prev_motor_targets = np.array(self.init_pos.copy())

        self.last_commands = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.paused = self.duck_config.start_paused

        # Fall detection: compare the measured gravity direction with the one
        # recorded while standing, and cut torque if the duck stays tilted.
        self.fall_detection = fall_detection
        self.fall_tilt_deg = fall_tilt_deg
        self.fall_time = fall_time
        self.recover_tilt_deg = recover_tilt_deg
        self.recover_ramp_seconds = recover_ramp_seconds
        self.fallen = False
        self.tilted_since = None
        self.upright_gravity = None

        self.command_freq = 20  # hz
        if self.commands:
            if self.autonomy_enabled:
                self.xbox_controller = XBoxController(self.command_freq, track_sticks=True)
            else:
                self.xbox_controller = XBoxController(self.command_freq)

        # Reference motion, but we only really need the length of one phase
        # TODO
        self.PRM = PolyReferenceMotion("./polynomial_coefficients.pkl")
        self.imitation_i = 0
        self.imitation_phase = np.array([0, 0])
        self.phase_frequency_factor = 1.0
        self.phase_frequency_factor_offset = (
            self.duck_config.phase_frequency_factor_offset
        )

        # Optional expression features
        if self.duck_config.eyes:
            self.eyes = Eyes()
        if self.duck_config.projector:
            self.projector = Projector()
        if self.duck_config.speaker:
            self.sounds = Sounds(
                volume=1.0, sound_directory="../mini_bdx_runtime/assets/"
            )
        if self.duck_config.antennas:
            self.antennas = Antennas()

        # Complete sensor and controller setup before enabling the motors.
        if self.autonomy_enabled:
            self.configure_autonomy(autonomy_bind, autonomy_port, *caps)
        try:
            self.start()
            if self.fall_detection:
                self.upright_gravity = self.capture_upright_gravity()
            if self.autonomy_enabled:
                self.autonomy_link.discard_waiting()
                self.autonomy_safety.expire_commands()
        except BaseException:
            if self.autonomy_enabled:
                try:
                    self.hwi.turn_off()
                finally:
                    self.autonomy_link.close()
            raise

    def configure_autonomy(self, bind, port, forward_cap, sideways_cap, turn_cap):
        """Load the optional command link before enabling motor torque."""
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if root not in sys.path:
            sys.path.insert(0, root)
        from autonomy.command_link import CommandLink
        from autonomy.safety import SafetyControl

        self.autonomy_safety = SafetyControl(forward_cap, sideways_cap, turn_cap,
                                            control_period=1 / self.control_freq)
        self.autonomy_link = CommandLink(bind, port)

    def update_autonomy(self):
        """Select walking speeds before the policy reads them."""
        now = time.monotonic()
        self.autonomy_link.poll(self.autonomy_safety, now)
        decision = self.autonomy_safety.step(
            now, manual=tuple(self.last_commands[:3]),
            sticks=self.xbox_controller.get_stick_axes(),
            rb=self.buttons.RB.triggered, a=self.buttons.A.triggered,
            paused=self.paused, fallen=self.fallen,
            link_ok=self.autonomy_link.healthy,
        )
        # Copy the command array so the controller keeps its manual values.
        self.last_commands = np.array(self.last_commands, dtype=float, copy=True)
        self.last_commands[:3] = decision.applied
        self.autonomy_head_frozen = decision.mode.name == "AUTO"
        if self.autonomy_head_frozen:
            self.last_commands[3:] = 0.0
            if decision.changed:
                # The final target includes the last manual head offsets.
                self.autonomy_head_targets = self.motor_targets[5:9].copy()
        if decision.changed:
            print(f"Command mode: {decision.mode.name.lower()}; {decision.reason}", flush=True)
            if self.duck_config.speaker and (decision.previous_mode.name == "AUTO" or self.autonomy_head_frozen):
                self.sounds.play("beep1.wav", maxtime_ms=150)
        if decision.notice:
            print(decision.notice, flush=True)
        imu_data = self.imu.get_data()
        self.autonomy_link.send_state(
            decision.mode, decision.applied, imu_data["gyro"], imu_data["accelero"],
            decision.reason, now,
        )

    def get_obs(self):

        imu_data = self.imu.get_data()

        dof_pos = self.hwi.get_present_positions(
            ignore=[
                "left_antenna",
                "right_antenna",
            ]
        )  # rad

        if dof_pos is None:
            return None

        dof_vel = self.hwi.get_present_velocities(
            ignore=[
                "left_antenna",
                "right_antenna",
            ]
        )  # rad/s

        if dof_vel is None:
            return None

        if len(dof_pos) != self.num_dofs:
            print(f"ERROR len(dof_pos) != {self.num_dofs}")
            return None

        if len(dof_vel) != self.num_dofs:
            print(f"ERROR len(dof_vel) != {self.num_dofs}")
            return None

        cmds = np.array(self.last_commands, copy=True)
        if self.freeze_head or getattr(self, "autonomy_head_frozen", False):
            cmds[3:] = 0.0

        if getattr(self, "stand_when_idle", False):
            feet_contacts = self.read_idle_contacts()
            if feet_contacts is None:
                feet_contacts = [False, False]
        else:
            feet_contacts = self.feet_contacts.get()

        obs = np.concatenate(
            [
                imu_data["gyro"],
                imu_data["accelero"],
                cmds,
                dof_pos - self.init_pos,
                dof_vel * 0.05,
                self.last_action,
                self.last_last_action,
                self.last_last_last_action,
                self.motor_targets,
                feet_contacts,
                self.imitation_phase,
            ]
        )

        return obs

    def start(self):
        kps = [self.pid[0]] * 14
        kds = [self.pid[2]] * 14

        # lower head kps
        kps[5:9] = [8, 8, 8, 8]
        self.walk_kps = kps

        self.hwi.set_kds(kds)
        self.hwi.turn_on_smooth(kps)

    def capture_upright_gravity(self, seconds=0.5):
        """Average the accelerometer while standing to get the upright reference."""
        samples = []
        end = time.time() + seconds
        while time.time() < end:
            accelero = np.asarray(self.imu.get_data()["accelero"], dtype=float)
            if np.linalg.norm(accelero) > 5:  # skip empty or garbage readings
                samples.append(accelero)
            time.sleep(1 / self.control_freq)
        if not samples:
            raise RuntimeError("No IMU accelerometer data; cannot set up fall detection")
        mean = np.mean(samples, axis=0)
        print(f"Fall detection: upright gravity reference {np.around(mean, 2)}")
        return mean / np.linalg.norm(mean)

    def get_tilt_deg(self):
        """Angle between the current gravity direction and the upright one."""
        accelero = np.asarray(self.imu.get_data()["accelero"], dtype=float)
        norm = np.linalg.norm(accelero)
        if norm < 1e-6:
            return None
        cos = np.dot(accelero / norm, self.upright_gravity)
        return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))

    def check_fall(self, t):
        """True once the tilt has stayed above the limit for fall_time seconds."""
        tilt = self.get_tilt_deg()
        if tilt is None or tilt < self.fall_tilt_deg:
            self.tilted_since = None
            return False
        if self.tilted_since is None:
            self.tilted_since = t
        if t - self.tilted_since >= self.fall_time:
            print(f"FALL DETECTED (tilt {tilt:.0f} deg): motor torque off.")
            if self.commands:
                print("Stand the duck up and press A to resume walking.")
            else:
                print("No controller: stop the script and restart it.")
            return True
        return False

    def handle_fall(self):
        self.hwi.turn_off()
        self.fallen = True
        self.tilted_since = None
        if self.duck_config.speaker:
            self.sounds.play_random_sound()

    def try_recover(self):
        """Ease back into the standing pose after the duck was stood up by hand."""
        tilt = self.get_tilt_deg()
        if tilt is None or tilt > self.recover_tilt_deg:
            shown = "unknown" if tilt is None else f"{tilt:.0f}"
            print(
                f"Still tilted ({shown} deg, limit {self.recover_tilt_deg:.0f}); "
                "stand the duck upright and press A again."
            )
            return
        print("Recovering: easing joints back to the standing pose")
        try:
            self.hwi.turn_on_smooth(
                self.walk_kps,
                ramp_seconds=self.recover_ramp_seconds,
                gain_step_seconds=0.5,
            )
        except RuntimeError as exc:
            # turn_on_smooth has already switched torque off.
            print(f"Recovery failed ({exc}); torque is off. Press A to try again.")
            return

        self.reset_walk_state()
        self.fallen = False
        self.paused = False
        print("Recovered: walking")

    def reset_walk_state(self):
        """Clear saved actions and targets before walking starts again."""
        self.last_action = np.zeros(self.num_dofs)
        self.last_last_action = np.zeros(self.num_dofs)
        self.last_last_last_action = np.zeros(self.num_dofs)
        self.motor_targets = np.array(self.init_pos.copy())
        self.prev_motor_targets = np.array(self.init_pos.copy())
        self.imitation_i = 0
        if self.action_filter is not None:
            self.action_filter = LowPassActionFilter(
                self.control_freq, self.action_filter_cutoff
            )
        self.walk_start_t = time.time()
        if getattr(self, "stand_when_idle", False):
            self.imitation_phase = np.zeros(2)
        self.idle_state = "walking"
        self.idle_zero_since = None
        self.idle_last_t = None
        self.idle_contact_lost_since = None

    def read_idle_contacts(self):
        """Read both foot switches. Return None if their data is unavailable."""
        try:
            contacts = np.asarray(self.feet_contacts.get())
            if contacts.shape == (2,) and np.all(np.isin(contacts, [False, True])):
                return contacts.astype(bool)
        except (OSError, RuntimeError, TypeError, ValueError):
            pass
        return None

    def update_idle(self, now):
        """Stop policy calls after a zero command. Keep safety checks in the main loop."""
        speeds = np.asarray(self.last_commands[:3])
        limits = [self.idle_linear_deadband, self.idle_linear_deadband, self.idle_turn_deadband]
        zero = np.all(np.isfinite(speeds)) and np.all(np.abs(speeds) <= limits)
        if not zero:
            self.idle_zero_since = None
            if self.idle_state != "walking":
                self.reset_walk_state()
                print("Walking: the policy has a fresh state.", flush=True)
            return False

        contacts = self.read_idle_contacts()
        both_feet = contacts is not None and bool(np.all(contacts))
        if self.idle_state == "walking":
            if self.idle_zero_since is None:
                self.idle_zero_since = now
            zero_time = now - self.idle_zero_since
            if zero_time < self.idle_delay:
                return False
            if not both_feet and zero_time - self.idle_delay < self.idle_contact_wait:
                return False
            self.idle_state = "moving"
            self.idle_start_targets = self.motor_targets.copy()
            self.idle_ramp_elapsed = 0.0
            self.idle_last_t = now
            self.idle_use_contacts = both_feet
            self.idle_contact_lost_since = None
            detail = "both feet touch the ground" if both_feet else "the foot contact wait has expired"
            print(f"Idle: move to the standing pose; {detail}.", flush=True)

        elapsed = 0.0 if self.idle_last_t is None else max(0.0, now - self.idle_last_t)
        self.idle_last_t = now
        standing_targets = np.array(self.init_pos, dtype=float)
        if self.freeze_head:
            standing_targets[5:9] = self.init_pos[5:9]
        elif self.autonomy_head_frozen:
            standing_targets[5:9] = self.autonomy_head_targets
        else:
            standing_targets[5:9] += self.last_commands[3:]

        if self.idle_state == "moving":
            if self.idle_use_contacts:
                if both_feet:
                    self.idle_contact_lost_since = None
                else:
                    if self.idle_contact_lost_since is None:
                        self.idle_contact_lost_since = now
                    elapsed = 0.0
                    if now - self.idle_contact_lost_since >= self.idle_contact_wait:
                        self.idle_use_contacts = False
            self.idle_ramp_elapsed += elapsed
            fraction = min(1.0, self.idle_ramp_elapsed / self.idle_ramp_time)
            # This curve starts and ends the move with zero target speed.
            weight = fraction * fraction * (3 - 2 * fraction)
            self.motor_targets = self.idle_start_targets + weight * (standing_targets - self.idle_start_targets)
            if fraction >= 1.0:
                self.idle_state = "standing"
        else:
            self.motor_targets = standing_targets

        if self.freeze_head:
            self.motor_targets[5:9] = self.init_pos[5:9]
        elif self.autonomy_head_frozen:
            self.motor_targets[5:9] = self.autonomy_head_targets
        self.prev_motor_targets = self.motor_targets.copy()
        return True

    def get_phase_frequency_factor(self, x_velocity):

        max_phase_frequency = 1.2
        min_phase_frequency = 1.0

        # Perform linear interpolation
        freq = min_phase_frequency + (abs(x_velocity) / 0.15) * (
            max_phase_frequency - min_phase_frequency
        )

        return freq

    def run(self):
        i = 0
        last_paused_feedback_check = 0
        run_failed = False
        try:
            print("Starting")
            self.walk_start_t = time.time()
            while True:
                left_trigger = 0
                right_trigger = 0
                t = time.time()

                if self.commands:
                    self.last_commands, self.buttons, left_trigger, right_trigger = (
                        self.xbox_controller.get_last_command()
                    )
                    if self.buttons.dpad_up.triggered:
                        self.phase_frequency_factor_offset += 0.05
                        print(
                            f"Phase frequency factor offset {round(self.phase_frequency_factor_offset, 3)}"
                        )

                    if self.buttons.dpad_down.triggered:
                        self.phase_frequency_factor_offset -= 0.05
                        print(
                            f"Phase frequency factor offset {round(self.phase_frequency_factor_offset, 3)}"
                        )

                    if self.buttons.LB.is_pressed:
                        self.phase_frequency_factor = 1.3
                    else:
                        self.phase_frequency_factor = 1.0

                    if self.buttons.X.triggered:
                        if self.duck_config.projector:
                            self.projector.switch()

                    if self.buttons.B.triggered:
                        if self.duck_config.speaker:
                            self.sounds.play_random_sound()

                    if self.duck_config.antennas:
                        self.antennas.set_position_left(right_trigger)
                        self.antennas.set_position_right(left_trigger)

                    if self.buttons.A.triggered:
                        if self.fallen:
                            self.try_recover()
                            if getattr(self, "autonomy_enabled", False):
                                self.autonomy_link.discard_waiting()
                                self.autonomy_safety.expire_commands()
                        else:
                            self.paused = not self.paused
                            if self.paused:
                                print("PAUSE")
                            else:
                                print("UNPAUSE")

                if self.fall_detection and not self.fallen and self.check_fall(t):
                    self.handle_fall()

                if getattr(self, "autonomy_enabled", False):
                    self.update_autonomy()

                if getattr(self, "stand_when_idle", False) and (self.fallen or self.paused):
                    self.idle_zero_since = None
                    self.idle_last_t = None
                    self.idle_contact_lost_since = None

                if self.fallen:
                    time.sleep(0.1)
                    continue

                if self.paused:
                    if t - last_paused_feedback_check >= 1:
                        if self.hwi.get_present_positions() is None:
                            raise RuntimeError("Motor feedback lost while paused; stopping walk")
                        last_paused_feedback_check = t
                    time.sleep(0.1)
                    continue

                idle = (self.update_idle(time.monotonic())
                        if getattr(self, "stand_when_idle", False) else False)
                obs = self.get_obs()
                if obs is None:
                    raise RuntimeError("Motor feedback lost; stopping walk")

                if idle:
                    self.hwi.set_position_all(make_action_dict(
                        self.motor_targets, list(self.hwi.joints.keys())))
                    time.sleep(max(0, 1 / self.control_freq - (time.time() - t)))
                    continue

                self.imitation_i += 1 * (
                    self.phase_frequency_factor + self.phase_frequency_factor_offset
                )
                self.imitation_i = self.imitation_i % self.PRM.nb_steps_in_period
                self.imitation_phase = np.array(
                    [
                        np.cos(
                            self.imitation_i / self.PRM.nb_steps_in_period * 2 * np.pi
                        ),
                        np.sin(
                            self.imitation_i / self.PRM.nb_steps_in_period * 2 * np.pi
                        ),
                    ]
                )

                if self.save_obs:
                    self.saved_obs.append(obs)

                if self.replay_obs is not None:
                    if i < len(self.replay_obs):
                        obs = self.replay_obs[i]
                    else:
                        print("BREAKING ")
                        break

                action = self.policy.infer(obs)
                if self.freeze_head or getattr(self, "autonomy_head_frozen", False):
                    # Keep the action history consistent with what we send to the motors.
                    action = action.copy()
                    action[5:9] = 0.0

                self.last_last_last_action = self.last_last_action.copy()
                self.last_last_action = self.last_action.copy()
                self.last_action = action.copy()

                # action = np.zeros(10)

                self.motor_targets = self.init_pos + action * self.action_scale

                # self.motor_targets = np.clip(
                #     self.motor_targets,
                #     self.prev_motor_targets
                #     - self.max_motor_velocity * (1 / self.control_freq),  # control dt
                #     self.prev_motor_targets
                #     + self.max_motor_velocity * (1 / self.control_freq),  # control dt
                # )

                if self.action_filter is not None:
                    self.action_filter.push(self.motor_targets)
                    filtered_motor_targets = self.action_filter.get_filtered_action()
                    if (
                        time.time() - self.walk_start_t > 1
                    ):  # give time to the filter to stabilize
                        self.motor_targets = filtered_motor_targets

                self.prev_motor_targets = self.motor_targets.copy()

                if self.freeze_head:
                    self.motor_targets[5:9] = self.init_pos[5:9]
                elif getattr(self, "autonomy_head_frozen", False):
                    self.motor_targets[5:9] = self.autonomy_head_targets
                else:
                    self.motor_targets[5:9] += self.last_commands[3:]

                action_dict = make_action_dict(
                    self.motor_targets, list(self.hwi.joints.keys())
                )

                self.hwi.set_position_all(action_dict)

                i += 1

                took = time.time() - t
                # print("Full loop took", took, "fps : ", np.around(1 / took, 2))
                if (1 / self.control_freq - took) < 0:
                    print(
                        "Policy control budget exceeded by",
                        np.around(took - 1 / self.control_freq, 3),
                    )
                time.sleep(max(0, 1 / self.control_freq - took))

        except KeyboardInterrupt:
            pass
        except BaseException:
            run_failed = True
            raise
        finally:
            torque_error = None
            try:
                self.hwi.turn_off()
            except Exception as exc:
                torque_error = exc
                print(
                    f"Could not disable motor torque over the bus ({exc}); "
                    "switch off motor power.",
                    file=sys.stderr,
                    flush=True,
                )
            finally:
                if getattr(self, "autonomy_enabled", False):
                    self.autonomy_link.close()
                if self.duck_config.antennas:
                    self.antennas.stop()
                if self.duck_config.eyes:
                    self.eyes.stop()
                if self.duck_config.projector:
                    self.projector.stop()
                self.feet_contacts.stop()
                if self.save_obs:
                    pickle.dump(self.saved_obs, open("robot_saved_obs.pkl", "wb"))
                print("TURNING OFF")
            if torque_error is not None and not run_failed:
                raise RuntimeError("Motor torque could not be disabled") from torque_error


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model_path", type=str, required=True)
    parser.add_argument(
        "--duck_config_path",
        type=str,
        required=False,
        default=f"{HOME_DIR}/duck_config.json",
    )
    parser.add_argument("-a", "--action_scale", type=float, default=0.25)
    parser.add_argument("-p", type=int, default=30)
    parser.add_argument("-i", type=int, default=0)
    parser.add_argument("-d", type=int, default=0)
    parser.add_argument("-c", "--control_freq", type=int, default=50)
    parser.add_argument("--pitch_bias", type=float, default=0,
                        help="This setting has no effect with the raw IMU. IMU means motion sensor.")
    parser.add_argument("--accel-offset", type=float, nargs=3, metavar=("X", "Y", "Z"), default=(0, 0, 0),
                        help="Subtract these X, Y, Z values from acceleration in metres per second squared. The default is 0 0 0.")
    parser.add_argument("--stand-when-idle", action="store_true",
                        help="Stop stepping after a zero walking command. Move smoothly to the standing pose.")
    parser.add_argument("--idle-linear-deadband", type=float, default=0.01,
                        help="Treat forward and sideways speeds within this limit as zero, in metres per second. The default is 0.01.")
    parser.add_argument("--idle-turn-deadband", type=float, default=0.05,
                        help="Treat turn speeds within this limit as zero, in radians per second. The default is 0.05.")
    parser.add_argument("--idle-delay", type=float, default=0.5,
                        help="Wait this many seconds at zero speed before idle standing. The default is 0.5.")
    parser.add_argument("--idle-ramp-time", type=float, default=0.5,
                        help="Move to the standing pose over this many seconds. The default is 0.5.")
    parser.add_argument("--idle-contact-wait", type=float, default=1.0,
                        help="Wait at most this many seconds for both feet before using the smooth move alone. The default is 1.")
    parser.add_argument(
        "--commands",
        action="store_true",
        default=True,
        help="external commands, keyboard or gamepad. Launch control_server.py on host computer",
    )
    parser.add_argument(
        "--save_obs",
        type=str,
        required=False,
        default=False,
        help="save the run's observations",
    )
    parser.add_argument(
        "--replay_obs",
        type=str,
        required=False,
        default=None,
        help="replay the observations from a previous run (can be from the robot or from mujoco)",
    )
    parser.add_argument("--cutoff_frequency", type=float, default=None)
    parser.add_argument(
        "--freeze-head",
        action="store_true",
        help="Hold the neck and head at the walking start pose during the walk",
    )

    parser.add_argument(
        "--no-fall-detection",
        action="store_true",
        help="Keep walking even when the duck has fallen over",
    )
    parser.add_argument(
        "--fall-tilt-deg",
        type=float,
        default=45.0,
        help="Tilt from upright, in degrees, that counts as a fall",
    )
    parser.add_argument(
        "--fall-time",
        type=float,
        default=0.2,
        help="Seconds the tilt must last before torque is cut",
    )

    parser.add_argument("--autonomy", action="store_true", help="Enable Mac commands and the duck safety checks.")
    parser.add_argument("--autonomy-bind", default="::", help="Listen at this address. Use :: for IPv4 and IPv6.")
    parser.add_argument("--autonomy-port", type=int, default=5602, help="Use this UDP command port. UDP sends separate network messages.")
    parser.add_argument("--autonomy-forward-cap", type=float, default=0.10, help="Limit autonomous forward and backward speed in metres per second.")
    parser.add_argument("--autonomy-sideways-cap", type=float, default=0.05, help="Limit autonomous sideways speed in metres per second.")
    parser.add_argument("--autonomy-turn-cap", type=float, default=0.6, help="Limit autonomous turning in radians per second.")

    args = parser.parse_args()
    pid = [args.p, args.i, args.d]

    print("Done parsing args")
    rl_walk = RLWalk(
        args.onnx_model_path,
        duck_config_path=args.duck_config_path,
        action_scale=args.action_scale,
        pid=pid,
        control_freq=args.control_freq,
        commands=args.commands,
        pitch_bias=args.pitch_bias,
        accel_offset=args.accel_offset,
        stand_when_idle=args.stand_when_idle,
        idle_linear_deadband=args.idle_linear_deadband,
        idle_turn_deadband=args.idle_turn_deadband,
        idle_delay=args.idle_delay,
        idle_ramp_time=args.idle_ramp_time,
        idle_contact_wait=args.idle_contact_wait,
        save_obs=args.save_obs,
        replay_obs=args.replay_obs,
        cutoff_frequency=args.cutoff_frequency,
        freeze_head=args.freeze_head,
        fall_detection=not args.no_fall_detection,
        fall_tilt_deg=args.fall_tilt_deg,
        fall_time=args.fall_time,
        autonomy=args.autonomy,
        autonomy_bind=args.autonomy_bind,
        autonomy_port=args.autonomy_port,
        autonomy_forward_cap=args.autonomy_forward_cap,
        autonomy_sideways_cap=args.autonomy_sideways_cap,
        autonomy_turn_cap=args.autonomy_turn_cap,
    )
    print("Done instantiating RLWalk")
    rl_walk.run()
