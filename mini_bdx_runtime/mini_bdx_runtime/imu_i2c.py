"""Open the I2C bus used by the BNO055."""

DEFAULT_IMU_I2C_BUS = 8


def open_imu_i2c(bus_id=DEFAULT_IMU_I2C_BUS):
    if bus_id is None:
        import board
        import busio

        return busio.I2C(board.SCL, board.SDA)

    if type(bus_id) is not int or bus_id < 0:
        raise ValueError("I2C bus must be a non-negative integer or None")

    try:
        from adafruit_extended_bus import ExtendedI2C
    except ImportError as exc:
        raise RuntimeError(
            "Software I2C requires adafruit-extended-bus in the runtime environment"
        ) from exc

    return ExtendedI2C(bus_id)
