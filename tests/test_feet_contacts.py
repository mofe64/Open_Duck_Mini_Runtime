"""Regression check for this build's active-low foot wiring."""

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
    / "feet_contacts.py"
)


class FakePin:
    def __init__(self, pin):
        self.pin = pin
        self.value = True
        self.deinitialized = False

    def deinit(self):
        self.deinitialized = True


class FeetContactsTest(unittest.TestCase):
    def test_checked_pins_pullups_and_active_low_reading(self):
        board = types.SimpleNamespace(D4=4, D17=17)
        digitalio = types.SimpleNamespace(
            DigitalInOut=FakePin,
            Direction=types.SimpleNamespace(INPUT="input"),
            Pull=types.SimpleNamespace(UP="up"),
        )
        with patch.dict(sys.modules, {"board": board, "digitalio": digitalio}):
            spec = importlib.util.spec_from_file_location("feet_contacts_test", MODULE_PATH)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            feet = module.FeetContacts()
            self.assertEqual((feet.left_foot.pin, feet.right_foot.pin), (4, 17))
            self.assertEqual((feet.left_foot.direction, feet.right_foot.direction), ("input", "input"))
            self.assertEqual((feet.left_foot.pull, feet.right_foot.pull), ("up", "up"))
            self.assertEqual(feet.get(), [False, False])

            feet.left_foot.value = False
            self.assertEqual(feet.get(), [True, False])
            feet.right_foot.value = False
            self.assertEqual(feet.get(), [True, True])

            feet.stop()
            self.assertTrue(feet.left_foot.deinitialized)
            self.assertTrue(feet.right_foot.deinitialized)


if __name__ == "__main__":
    unittest.main()
