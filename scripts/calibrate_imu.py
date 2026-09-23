import argparse

from mini_bdx_runtime.raw_imu import Imu
from mini_bdx_runtime.imu_i2c import DEFAULT_IMU_I2C_BUS

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--i2c-bus", type=int, default=DEFAULT_IMU_I2C_BUS)
    parser.add_argument("--upside-down", action="store_true")
    args = parser.parse_args()
    imu = Imu(
        50,
        calibrate=True,
        upside_down=args.upside_down,
        i2c_bus=args.i2c_bus,
    )
