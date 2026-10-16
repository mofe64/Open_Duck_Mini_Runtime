"""Check the short mode sound without starting an audio device."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


class ModeSoundTests(unittest.TestCase):
    def test_short_sound_limits_play_time_and_keeps_default_play_call(self):
        source = Path(__file__).resolve().parents[1] / "mini_bdx_runtime/mini_bdx_runtime/sounds.py"
        spec = importlib.util.spec_from_file_location("mode_sound_test", source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"pygame": types.ModuleType("pygame")}):
            spec.loader.exec_module(module)
        sounds = module.Sounds.__new__(module.Sounds)
        sound = Mock()
        sounds.ok, sounds.sounds = True, {"beep1.wav": sound}
        sounds.play("beep1.wav", maxtime_ms=150)
        sound.play.assert_called_once_with(maxtime=150)
        sound.play.reset_mock()
        sounds.play("beep1.wav")
        sound.play.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
