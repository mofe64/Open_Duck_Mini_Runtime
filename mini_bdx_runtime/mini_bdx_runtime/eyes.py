import board
import digitalio
import random
import time
from threading import Thread, Event

EYE_GPIO27_PIN = board.D27
EYE_GPIO22_PIN = board.D22


class Eyes:
    def __init__(self, blink_duration=0.1, min_interval=1.0, max_interval=4.0):
        self.eye_gpio27 = digitalio.DigitalInOut(EYE_GPIO27_PIN)
        self.eye_gpio27.direction = digitalio.Direction.OUTPUT

        self.eye_gpio22 = digitalio.DigitalInOut(EYE_GPIO22_PIN)
        self.eye_gpio22.direction = digitalio.Direction.OUTPUT

        self.blink_duration = blink_duration
        self.min_interval = min_interval
        self.max_interval = max_interval

        self._stop_event = Event()
        self._thread = Thread(target=self.run, daemon=True)
        self._thread.start()

    def _set_eyes(self, state):
        self.eye_gpio27.value = state
        self.eye_gpio22.value = state

    def run(self):
        try:
            while not self._stop_event.is_set():
                self._set_eyes(False)
                time.sleep(self.blink_duration)
                self._set_eyes(True)
                next_blink = random.uniform(self.min_interval, self.max_interval)
                time.sleep(next_blink)
        except Exception as err:
            print(f"Error in eye thread: {err}")
            self._stop_event.set()

    def stop(self):
        self._stop_event.set()
        self._thread.join()
        self._set_eyes(False)
        self.eye_gpio27.deinit()
        self.eye_gpio22.deinit()


if __name__ == "__main__":
    e = Eyes()
    try:
        while True:
            time.sleep(1)
    finally:
        e.stop()
