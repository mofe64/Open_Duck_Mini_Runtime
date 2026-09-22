import time

LEFT_ANTENNA_PIN = 15
RIGHT_ANTENNA_PIN = 14
LEFT_SIGN = 1
RIGHT_SIGN = -1
MIN_UPDATE_INTERVAL = 1 / 50  # 20ms


def value_to_pulse_width(v):
    return round(1500 + v * 100)


class Antennas:
    def __init__(self):
        import pigpio

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.pi.stop()
            raise RuntimeError("Could not connect to the local pigpio service")

        try:
            self.pi.set_servo_pulsewidth(LEFT_ANTENNA_PIN, 1500)
            self.pi.set_servo_pulsewidth(RIGHT_ANTENNA_PIN, 1500)
        except Exception:
            self.pi.set_servo_pulsewidth(LEFT_ANTENNA_PIN, 0)
            self.pi.stop()
            raise

    def set_position_left(self, position):
        self.set_position(LEFT_ANTENNA_PIN, position, LEFT_SIGN)

    def set_position_right(self, position):
        self.set_position(RIGHT_ANTENNA_PIN, position, RIGHT_SIGN)

    def set_position(self, pin, value, sign=1):
        if -1 <= value <= 1:
            self.pi.set_servo_pulsewidth(pin, value_to_pulse_width(value * sign))
        else:
            print("Invalid input! Enter a value between -1 and 1.")

    def stop(self):
        time.sleep(MIN_UPDATE_INTERVAL)
        self.set_position_left(0)
        self.set_position_right(0)
        time.sleep(MIN_UPDATE_INTERVAL)
        self.pi.set_servo_pulsewidth(LEFT_ANTENNA_PIN, 0)
        self.pi.set_servo_pulsewidth(RIGHT_ANTENNA_PIN, 0)
        self.pi.stop()
