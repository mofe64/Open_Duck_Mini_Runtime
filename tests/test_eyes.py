"""Regression check for the two mapped LED outputs."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "mini_bdx_runtime"
    / "mini_bdx_runtime"
    / "eyes.py"
)


class FakePin:
    def __init__(self, pin):
        self.pin = pin
        self.value = False
        self.deinitialized = False

    def deinit(self):
        self.deinitialized = True


class FakeThread:
    def __init__(self, **kwargs):
        self.started = False
        self.joined = False

    def start(self):
        self.started = True

    def join(self):
        self.joined = True


class EyesTest(unittest.TestCase):
    def test_both_checked_gpio_outputs_switch_together(self):
        board = types.SimpleNamespace(D27=27, D22=22)
        digitalio = types.SimpleNamespace(
            DigitalInOut=FakePin,
            Direction=types.SimpleNamespace(OUTPUT="output"),
        )
        with patch.dict(sys.modules, {"board": board, "digitalio": digitalio}):
            spec = importlib.util.spec_from_file_location("eyes_test", MODULE_PATH)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.Thread = FakeThread

            eyes = module.Eyes()
            self.assertEqual((eyes.eye_gpio27.pin, eyes.eye_gpio22.pin), (27, 22))
            self.assertEqual((eyes.eye_gpio27.direction, eyes.eye_gpio22.direction), ("output", "output"))

            eyes._set_eyes(True)
            self.assertEqual((eyes.eye_gpio27.value, eyes.eye_gpio22.value), (True, True))
            eyes.stop()
            self.assertEqual((eyes.eye_gpio27.value, eyes.eye_gpio22.value), (False, False))
            self.assertTrue(eyes.eye_gpio27.deinitialized)
            self.assertTrue(eyes.eye_gpio22.deinitialized)


if __name__ == "__main__":
    unittest.main()
