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
    ):

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
        self.start()
        if self.fall_detection:
            self.upright_gravity = self.capture_upright_gravity()

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
        if self.freeze_head:
            cmds[3:] = 0.0

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

        # Start the policy from a clean state, as at the beginning of a walk.
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
        self.fallen = False
        self.paused = False
        print("Recovered: walking")

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
                        else:
                            self.paused = not self.paused
                            if self.paused:
                                print("PAUSE")
                            else:
                                print("UNPAUSE")

                if self.fall_detection and not self.fallen and self.check_fall(t):
                    self.handle_fall()

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

                obs = self.get_obs()
                if obs is None:
                    raise RuntimeError("Motor feedback lost; stopping walk")

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
                if self.freeze_head:
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
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
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
        save_obs=args.save_obs,
        replay_obs=args.replay_obs,
        cutoff_frequency=args.cutoff_frequency,
        freeze_head=args.freeze_head,
        fall_detection=not args.no_fall_detection,
        fall_tilt_deg=args.fall_tilt_deg,
        fall_time=args.fall_time,
    )
    print("Done instantiating RLWalk")
    rl_walk.run()
