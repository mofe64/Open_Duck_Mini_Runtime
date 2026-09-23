import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

import numpy  # Keep it loaded while mocked imports are restored.


ROOT = Path(__file__).resolve().parents[1]


class ControllerStartupTests(unittest.TestCase):
    def test_background_input_is_enabled_before_pygame_init(self):
        pygame = types.ModuleType("pygame")
        joystick = Mock()
        joystick.get_numaxes.return_value = 6
        pygame.joystick = types.SimpleNamespace(Joystick=Mock(return_value=joystick))
        pygame.init = Mock()

        package = types.ModuleType("mini_bdx_runtime")
        package.__path__ = []
        buttons = types.ModuleType("mini_bdx_runtime.buttons")
        buttons.Buttons = Mock
        spec = importlib.util.spec_from_file_location(
            "controller_test_module",
            ROOT / "mini_bdx_runtime" / "mini_bdx_runtime" / "xbox_controller.py",
        )
        module = importlib.util.module_from_spec(spec)
        with patch.dict(
            sys.modules,
            {
                "pygame": pygame,
                "mini_bdx_runtime": package,
                "mini_bdx_runtime.buttons": buttons,
            },
        ):
            spec.loader.exec_module(module)

        module.Thread = Mock()  # Do not start the command polling thread.
        environment = {}
        with patch.object(module.os, "environ", environment):
            pygame.init.side_effect = lambda: self.assertEqual(
                environment["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"], "1"
            )
            module.XBoxController(20)

        pygame.init.assert_called_once()
        joystick.init.assert_called_once()


if __name__ == "__main__":
    unittest.main()
