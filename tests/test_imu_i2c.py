import importlib.util
import inspect
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "mini_bdx_runtime"
    / "mini_bdx_runtime"
    / "imu_i2c.py"
)
spec = importlib.util.spec_from_file_location("imu_i2c", MODULE_PATH)
imu_i2c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imu_i2c)

class OpenImuI2cTests(unittest.TestCase):
    def test_explicit_none_uses_hardware_pins(self):
        calls = []
        board = types.SimpleNamespace(SCL=object(), SDA=object())
        busio = types.SimpleNamespace(
            I2C=lambda scl, sda: calls.append((scl, sda)) or "hardware"
        )
        with patch.dict(sys.modules, {"board": board, "busio": busio}):
            self.assertEqual(imu_i2c.open_imu_i2c(None), "hardware")
        self.assertEqual(calls, [(board.SCL, board.SDA)])

    def test_default_uses_software_bus_eight(self):
        calls = []
        extended = types.SimpleNamespace(
            ExtendedI2C=lambda bus_id: calls.append(bus_id) or "software"
        )
        with patch.dict(sys.modules, {"adafruit_extended_bus": extended}):
            self.assertEqual(imu_i2c.open_imu_i2c(), "software")
        self.assertEqual(calls, [8])

    def test_other_bus_number_uses_extended_i2c(self):
        calls = []
        extended = types.SimpleNamespace(
            ExtendedI2C=lambda bus_id: calls.append(bus_id) or "software"
        )
        with patch.dict(sys.modules, {"adafruit_extended_bus": extended}):
            self.assertEqual(imu_i2c.open_imu_i2c(9), "software")
        self.assertEqual(calls, [9])

    def test_rejects_invalid_bus_number(self):
        for value in (-1, True, "8"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    imu_i2c.open_imu_i2c(value)


class ReaderDefaultTests(unittest.TestCase):
    def test_raw_reader_uses_software_bus_eight_by_default(self):
        path = MODULE_PATH.with_name("raw_imu.py")
        reader_spec = importlib.util.spec_from_file_location("raw_imu", path)
        reader = importlib.util.module_from_spec(reader_spec)
        with patch.dict(
            sys.modules,
            {"adafruit_bno055": types.ModuleType("adafruit_bno055"), "imu_i2c": imu_i2c},
        ):
            reader_spec.loader.exec_module(reader)
        self.assertEqual(
            inspect.signature(reader.Imu).parameters["i2c_bus"].default, 8
        )


if __name__ == "__main__":
    unittest.main()
