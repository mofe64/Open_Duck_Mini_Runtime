import importlib.util
import json
from pathlib import Path
import sys
import tempfile
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

config_spec = importlib.util.spec_from_file_location(
    "duck_config", MODULE_PATH.with_name("duck_config.py")
)
duck_config = importlib.util.module_from_spec(config_spec)
config_spec.loader.exec_module(duck_config)


class OpenImuI2cTests(unittest.TestCase):
    def test_default_uses_hardware_pins(self):
        calls = []
        board = types.SimpleNamespace(SCL=object(), SDA=object())
        busio = types.SimpleNamespace(
            I2C=lambda scl, sda: calls.append((scl, sda)) or "hardware"
        )
        with patch.dict(sys.modules, {"board": board, "busio": busio}):
            self.assertEqual(imu_i2c.open_imu_i2c(), "hardware")
        self.assertEqual(calls, [(board.SCL, board.SDA)])

    def test_bus_number_uses_extended_i2c(self):
        calls = []
        extended = types.SimpleNamespace(
            ExtendedI2C=lambda bus_id: calls.append(bus_id) or "software"
        )
        with patch.dict(sys.modules, {"adafruit_extended_bus": extended}):
            self.assertEqual(imu_i2c.open_imu_i2c(8), "software")
        self.assertEqual(calls, [8])

    def test_rejects_invalid_bus_number(self):
        for value in (-1, True, "8"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    imu_i2c.open_imu_i2c(value)


class DuckConfigI2cTests(unittest.TestCase):
    def test_bus_selection_defaults_to_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duck_config.json"
            path.write_text("{}")
            self.assertIsNone(duck_config.DuckConfig(str(path)).imu_i2c_bus)

    def test_bus_selection_accepts_eight(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duck_config.json"
            path.write_text(json.dumps({"imu_i2c_bus": 8}))
            self.assertEqual(duck_config.DuckConfig(str(path)).imu_i2c_bus, 8)

    def test_bus_selection_rejects_invalid_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duck_config.json"
            for value in (-1, True, "8"):
                with self.subTest(value=value):
                    path.write_text(json.dumps({"imu_i2c_bus": value}))
                    with self.assertRaises(ValueError):
                        duck_config.DuckConfig(str(path))


if __name__ == "__main__":
    unittest.main()
