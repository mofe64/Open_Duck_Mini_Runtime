import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

import numpy  # Keep it loaded while mocked imports are restored.


ROOT = Path(__file__).resolve().parents[1]


class ControllerStartupTests(unittest.TestCase):
    def load_controller(self):
        pygame = types.ModuleType("pygame")
        joystick = Mock()
        joystick.get_numaxes.return_value = 6
        pygame.joystick = types.SimpleNamespace(Joystick=Mock(return_value=joystick))
        pygame.init = Mock()
        pygame.event = types.SimpleNamespace(get=Mock(return_value=[]), pump=Mock())
        joystick.get_hat.return_value = (0, 0)
        joystick.get_button.return_value = 0

        package = types.ModuleType("mini_bdx_runtime")
        package.__path__ = []
        button_spec = importlib.util.spec_from_file_location(
            "mini_bdx_runtime.buttons", ROOT / "mini_bdx_runtime/mini_bdx_runtime/buttons.py")
        buttons = importlib.util.module_from_spec(button_spec)
        button_spec.loader.exec_module(buttons)
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
        return module, pygame, joystick

    def test_background_input_is_enabled_before_pygame_init(self):
        module, pygame, joystick = self.load_controller()
        environment = {}
        with patch.object(module.os, "environ", environment):
            pygame.init.side_effect = lambda: self.assertEqual(
                environment["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"], "1"
            )
            module.XBoxController(20)

        pygame.init.assert_called_once()
        joystick.init.assert_called_once()

    def test_raw_left_stick_remains_available_in_head_control_mode(self):
        module, _pygame, joystick = self.load_controller()
        values = [-0.3, 0, 0, 0, -1, -1]
        joystick.get_axis.side_effect = lambda axis: values[axis]
        controller = module.XBoxController(20, only_head_control=True, track_sticks=True)
        self.assertIsNone(controller.get_stick_axes())
        command = controller.get_commands()
        numpy.testing.assert_array_equal(command[0][:3], [0, 0, 0])
        self.assertEqual(controller.get_stick_axes(), (0.3, 0, 0, 0))
        self.assertEqual(len(command), 10)
        self.assertEqual(len(controller.get_last_command()), 4)

    def test_raw_right_vertical_stick_is_available_for_override(self):
        module, _pygame, joystick = self.load_controller()
        values = [0, 0, 0, 0.25, -1, -1]
        joystick.get_axis.side_effect = lambda axis: values[axis]
        controller = module.XBoxController(20, track_sticks=True)
        controller.get_commands()
        self.assertEqual(controller.get_stick_axes(), (0, 0, 0, -0.25))

    def test_held_a_and_rb_do_not_repeat_between_controller_samples(self):
        for button_name, number in (("A", 0), ("RB", 7)):
            module, _pygame, joystick = self.load_controller()
            joystick.get_axis.side_effect = lambda axis: -1 if axis >= 4 else 0
            held = [False]
            joystick.get_button.side_effect = lambda button: held[0] and button == number
            with patch.object(module.time, "time", return_value=100):
                controller = module.XBoxController(20, track_sticks=True)
            held[0] = True
            with patch.object(module.time, "time", return_value=101):
                controller.cmd_queue.put(controller.get_commands())
                controller.get_last_command()
                self.assertTrue(getattr(controller.buttons, button_name).triggered)
            with patch.object(module.time, "time", return_value=101.01):
                controller.get_last_command()
                self.assertFalse(getattr(controller.buttons, button_name).triggered)
            with patch.object(module.time, "time", return_value=101.04):
                controller.cmd_queue.put(controller.get_commands())
                controller.get_last_command()
                self.assertFalse(getattr(controller.buttons, button_name).triggered)
            held[0] = False
            with patch.object(module.time, "time", return_value=101.05):
                controller.cmd_queue.put(controller.get_commands())
                controller.get_last_command()
            held[0] = True
            with patch.object(module.time, "time", return_value=101.08):
                controller.cmd_queue.put(controller.get_commands())
                controller.get_last_command()
                self.assertTrue(getattr(controller.buttons, button_name).triggered)


if __name__ == "__main__":
    unittest.main()
