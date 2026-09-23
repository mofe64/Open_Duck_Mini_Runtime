import time

import numpy as np
import rustypot
from mini_bdx_runtime.duck_config import DuckConfig


class HWI:
    def __init__(self, duck_config: DuckConfig, usb_port: str = "/dev/ttyACM0"):

        self.duck_config = duck_config

        # Order matters here. On this build, all leg motor IDs are mounted
        # on the opposite legs from the original wiring map.
        self.joints = {
            "left_hip_yaw": 10,
            "left_hip_roll": 11,
            "left_hip_pitch": 12,
            "left_knee": 13,
            "left_ankle": 14,
            "neck_pitch": 30,
            "head_pitch": 31,
            "head_yaw": 32,
            "head_roll": 33,
            # "left_antenna": None,
            # "right_antenna": None,
            "right_hip_yaw": 20,
            "right_hip_roll": 21,
            "right_hip_pitch": 22,
            "right_knee": 23,
            "right_ankle": 24,
        }

        self.zero_pos = {
            "left_hip_yaw": 0,
            "left_hip_roll": 0,
            "left_hip_pitch": 0,
            "left_knee": 0,
            "left_ankle": 0,
            "neck_pitch": 0,
            "head_pitch": 0,
            "head_yaw": 0,
            "head_roll": 0,
            # "left_antenna":0,
            # "right_antenna":0,
            "right_hip_yaw": 0,
            "right_hip_roll": 0,
            "right_hip_pitch": 0,
            "right_knee": 0,
            "right_ankle": 0,
        }

        self.init_pos = {
            "left_hip_yaw": 0.002,
            "left_hip_roll": 0.053,
            "left_hip_pitch": -0.63,
            "left_knee": 1.368,
            "left_ankle": -0.784,
            "neck_pitch": 0.0,
            "head_pitch": 0.0,
            "head_yaw": 0,
            "head_roll": 0,
            # "left_antenna": 0,
            # "right_antenna": 0,
            "right_hip_yaw": -0.003,
            "right_hip_roll": -0.065,
            "right_hip_pitch": 0.635,
            "right_knee": 1.379,
            "right_ankle": -0.796,
        }

        self.joints_offsets = self.duck_config.joints_offset

        self.kps = np.ones(len(self.joints)) * 32  # default kp
        self.kds = np.ones(len(self.joints)) * 0  # default kd
        self.low_torque_kps = np.ones(len(self.joints)) * 2

        self.io = rustypot.feetech(usb_port, 1000000)

    def set_kps(self, kps):
        self.kps = kps
        self.io.set_kps(list(self.joints.values()), self.kps)

    def set_kds(self, kds):
        self.kds = kds
        self.io.set_kds(list(self.joints.values()), self.kds)

    def set_kp(self, id, kp):
        self.io.set_kps([id], [kp])

    def turn_on(self):
        self.io.set_kps(list(self.joints.values()), self.low_torque_kps)
        print("turn on : low KPS set")
        time.sleep(1)

        self.set_position_all(self.init_pos)
        print("turn on : init pos set")

        time.sleep(1)

        self.io.set_kps(list(self.joints.values()), self.kps)
        print("turn on : high kps")

    def turn_on_smooth(self, final_kps, ramp_seconds=20.0):
        """Move to the walking start pose before raising motor gains."""
        names = list(self.joints)
        ids = list(self.joints.values())
        if len(final_kps) != len(ids):
            raise ValueError("Expected one P gain per joint")
        if ramp_seconds <= 0:
            raise ValueError("ramp_seconds must be positive")

        positions = self.get_present_positions()
        if positions is None or len(positions) != len(ids):
            raise RuntimeError("Could not read all motor positions before startup")

        start = np.asarray(positions, dtype=float)
        target = np.array([self.init_pos[name] for name in names], dtype=float)
        self.kps = list(final_kps)
        initial_kp = min(8, min(self.kps))
        steps = max(1, round(ramp_seconds / 0.05))

        def set_fraction(fraction):
            values = start + (target - start) * fraction
            self.set_position_all(
                {name: float(value) for name, value in zip(names, values)}
            )

        def check_position(fraction, limit):
            actual = self.get_present_positions()
            if actual is None or len(actual) != len(ids):
                raise RuntimeError("Lost motor position readings during startup")
            wanted = start + (target - start) * fraction
            errors = np.abs(actual - wanted)
            worst = int(np.argmax(errors))
            print(f"Startup {fraction:.0%}: {names[worst]} error {errors[worst]:.3f} rad")
            if errors[worst] > limit:
                raise RuntimeError(f"{names[worst]} is not following its target")

        try:
            self.io.set_kps(ids, [initial_kp] * len(ids))
            set_fraction(0)
            self.io.enable_torque(ids)

            for step in range(1, steps + 1):
                fraction = step / steps
                set_fraction(fraction)
                time.sleep(0.05)
                if step % 20 == 0 or step == steps:
                    check_position(fraction, 0.25)

            for fraction in (0.2, 0.4, 0.6, 0.8, 1.0):
                gains = [
                    int(round(initial_kp + (kp - initial_kp) * fraction))
                    for kp in self.kps
                ]
                self.io.set_kps(ids, gains)
                time.sleep(2)
                check_position(1, 0.15)
        except BaseException:
            self.turn_off()
            raise

    def turn_off(self):
        self.io.disable_torque(list(self.joints.values()))

    def set_position(self, joint_name, pos):
        """
        pos is in radians
        """
        id = self.joints[joint_name]
        pos = pos + self.joints_offsets[joint_name]
        self.io.write_goal_position([id], [pos])

    def set_position_all(self, joints_positions):
        """
        joints_positions is a dictionary with joint names as keys and joint positions as values
        Warning: expects radians
        """
        ids_positions = {
            self.joints[joint]: position + self.joints_offsets[joint]
            for joint, position in joints_positions.items()
        }

        self.io.write_goal_position(
            list(self.joints.values()), list(ids_positions.values())
        )

    def get_present_positions(self, ignore=[]):
        """
        Returns the present positions in radians
        """

        try:
            present_positions = self.io.read_present_position(
                list(self.joints.values())
            )
        except Exception as e:
            print(e)
            return None

        present_positions = [
            pos - self.joints_offsets[joint]
            for joint, pos in zip(self.joints.keys(), present_positions)
            if joint not in ignore
        ]
        return np.array(np.around(present_positions, 3))

    def get_present_velocities(self, rad_s=True, ignore=[]):
        """
        Returns the present velocities in rad/s (default) or rev/min
        """
        try:
            present_velocities = self.io.read_present_velocity(
                list(self.joints.values())
            )
        except Exception as e:
            print(e)
            return None

        present_velocities = [
            vel
            for joint, vel in zip(self.joints.keys(), present_velocities)
            if joint not in ignore
        ]

        return np.array(np.around(present_velocities, 3))
