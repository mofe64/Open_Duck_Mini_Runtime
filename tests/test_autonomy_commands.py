"""Test command messages and duck safety rules without hardware."""

import socket
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from autonomy import protocol
from autonomy.command_link import CommandLink
from autonomy.mac_send_command import main as sender_main, parse_args, send_commands
from autonomy.protocol import Command, Mode, State
from autonomy.safety import SafetyControl, ZERO_SPEEDS


class CommandProtocolTests(unittest.TestCase):
    def test_command_round_trip(self):
        command = Command(2 ** 63 + 3, 0.05, -0.02, 0.5, True)
        packet = protocol.pack_command(command)
        result = protocol.unpack_command(packet)
        self.assertEqual(len(packet), 29)
        self.assertEqual((result.sequence, result.stop), (command.sequence, True))
        for actual, expected in zip(result.speeds, command.speeds):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_state_round_trip_for_each_mode(self):
        for mode in Mode:
            state = State(17, mode, (0.1, -0.05, 0.6), (1, 2, 3), (4, 5, -9.81), "RB selected auto mode.")
            result = protocol.unpack_state(protocol.pack_state(state))
            self.assertEqual((result.sequence, result.mode, result.reason), (17, mode, state.reason))
            for field in ("applied", "gyro", "accelero"):
                for actual, expected in zip(getattr(result, field), getattr(state, field)):
                    self.assertAlmostEqual(actual, expected, places=5)

    def test_rejects_bad_command_headers_lengths_and_stop_flags(self):
        packet = protocol.pack_command(Command(1, 0, 0, 0))
        damaged = [b"", packet[:-1], packet + b"x", b"BAD!" + packet[4:],
                   packet[:4] + b"\x02" + packet[5:], packet[:-1] + b"\x02"]
        for message in damaged:
            with self.subTest(message=message), self.assertRaises(ValueError):
                protocol.unpack_command(message)

    def test_rejects_nonfinite_command_speeds(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            for axis in range(3):
                speeds = [0, 0, 0]
                speeds[axis] = value
                with self.assertRaises(ValueError):
                    protocol.pack_command(Command(1, *speeds))
                packet = protocol.PREFIX.pack(protocol.MAGIC, 1, protocol.COMMAND, protocol.COMMAND_SIZE)
                packet += protocol.COMMAND_FIELDS.pack(1, *speeds, 0)
                with self.assertRaises(ValueError):
                    protocol.unpack_command(packet)

    def test_rejects_bad_sequence_and_unrepresentable_values(self):
        for sequence in (-1, 2 ** 64, 1.5, True):
            with self.assertRaises(ValueError):
                protocol.pack_command(Command(sequence, 0, 0, 0))
        with self.assertRaises(ValueError):
            protocol.pack_command(Command(1, 1e300, 0, 0))
        with self.assertRaises(ValueError):
            protocol.pack_command(Command(1, 0, 0, 0, 2))

    def test_rejects_bad_state_values_lengths_mode_and_text(self):
        state = State(1, Mode.AUTO, (0, 0, 0), (0, 0, 0), (0, 0, 0), "Valid reason.")
        packet = protocol.pack_state(state)
        unknown = bytearray(packet)
        unknown[protocol.PREFIX.size + 8] = 99
        for message in (packet[:-1], packet + b"x", bytes(unknown), packet[:-1] + b"\xff"):
            with self.assertRaises(ValueError):
                protocol.unpack_state(message)
        with self.assertRaises(ValueError):
            protocol.pack_state(State(1, Mode.AUTO, (0, 0, 0), (float("nan"), 0, 0), (0, 0, 0), "Bad gyro."))
        with self.assertRaises(ValueError):
            protocol.pack_state(State(1, Mode.AUTO, (0, 0, 0), (0, 0, 0), (0, 0, 0), "x" * 97))


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.safety = SafetyControl()

    def enter_auto(self, command=None):
        self.safety.accept(command or Command(10, 1, -1, 1), 10.0)
        decision = self.safety.step(10.1, rb=True)
        self.assertEqual(decision.mode, Mode.AUTO)
        return decision

    def test_starts_in_manual_and_preserves_manual_speeds(self):
        decision = self.safety.step(0, manual=(0.15, 0.2, 1.0))
        self.assertEqual(decision.mode, Mode.MANUAL)
        self.assertEqual(decision.applied, (0.15, 0.2, 1.0))

    def test_rb_needs_a_recent_valid_command(self):
        self.assertIn("no valid command", self.safety.step(0, rb=True).notice)
        self.safety.accept(Command(1, 0.05, 0, 0), 1)
        self.assertEqual(self.safety.step(1.5, rb=True).mode, Mode.MANUAL)
        self.assertEqual(self.safety.step(1.499, rb=True).mode, Mode.MANUAL)
        self.assertEqual(self.safety.step(1.479, rb=True).mode, Mode.AUTO)

    def test_watchdog_exits_at_timeout_and_zeroes_that_step(self):
        self.enter_auto()
        self.assertEqual(self.safety.step(10.479).mode, Mode.AUTO)
        decision = self.safety.step(10.5, manual=(0.15, 0.2, 1))
        self.assertEqual((decision.mode, decision.applied), (Mode.MANUAL, ZERO_SPEEDS))
        self.assertIn("timeout", decision.reason)
        self.assertEqual(self.safety.step(10.52, manual=(0.15, 0.2, 1)).applied, (0.15, 0.2, 1))

    def test_watchdog_reserves_one_control_step(self):
        self.enter_auto()
        decision = self.safety.step(10.481, manual=(0.1, 0, 0))
        self.assertEqual((decision.mode, decision.applied), (Mode.MANUAL, ZERO_SPEEDS))
        self.assertLess(10.481 - self.safety.last_received, 0.5)

    def test_new_command_refreshes_watchdog_without_reentering_auto(self):
        self.enter_auto()
        self.assertTrue(self.safety.accept(Command(11, 0.03, 0, 0), 10.4))
        self.assertEqual(self.safety.step(10.6).applied, (0.03, 0, 0))
        self.assertEqual(self.safety.step(10.91).mode, Mode.MANUAL)
        self.safety.accept(Command(12, 0.03, 0, 0), 11)
        self.assertEqual(self.safety.step(11.01).mode, Mode.MANUAL)

    def test_rb_toggles_auto_off_with_zero_speeds(self):
        self.enter_auto()
        decision = self.safety.step(10.2, rb=True, manual=(0.1, 0, 0))
        self.assertEqual((decision.mode, decision.applied), (Mode.MANUAL, ZERO_SPEEDS))

    def test_caps_limit_both_signs_and_custom_values(self):
        self.assertEqual(self.enter_auto().applied, (0.1, -0.05, 0.6))
        self.safety.accept(Command(11, -9, 9, -9), 10.2)
        self.assertEqual(self.safety.step(10.2).applied, (-0.1, 0.05, -0.6))
        limited = SafetyControl(0.02, 0, 0.1)
        limited.accept(Command(1, 9, 9, 9), 0)
        self.assertEqual(limited.step(0, rb=True).applied, (0.02, 0, 0.1))

    def test_caps_reject_invalid_settings(self):
        for value in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                SafetyControl(forward_cap=value)

    def test_either_stick_overrides_in_both_directions(self):
        for axis in range(4):
            for value in (-0.201, 0.201):
                with self.subTest(axis=axis, value=value):
                    self.safety = SafetyControl()
                    self.enter_auto()
                    sticks = [0.0] * 4
                    sticks[axis] = value
                    decision = self.safety.step(10.2, sticks=sticks, manual=(0.1, 0, 0))
                    self.assertEqual((decision.mode, decision.applied), (Mode.MANUAL, ZERO_SPEEDS))

    def test_stick_threshold_and_diagonal_travel(self):
        self.enter_auto()
        self.assertEqual(self.safety.step(10.2, sticks=(0.2, 0, 0, 0)).mode, Mode.AUTO)
        self.assertEqual(self.safety.step(10.3, sticks=(0.15, 0.15, 0, 0)).mode, Mode.MANUAL)

    def test_unknown_or_invalid_sticks_block_auto(self):
        for sticks in (None, (0, 0), (float("nan"), 0, 0, 0), (1.1, 0, 0, 0)):
            self.safety = SafetyControl()
            self.safety.accept(Command(1, 0.05, 0, 0), 0)
            self.assertEqual(self.safety.step(0, rb=True, sticks=sticks).mode, Mode.MANUAL)

    def test_fall_pause_a_stop_and_bad_link_leave_auto(self):
        cases = [(dict(fallen=True), Mode.FALLEN), (dict(paused=True), Mode.PAUSED),
                 (dict(a=True), Mode.MANUAL), (dict(link_ok=False), Mode.MANUAL)]
        for flags, mode in cases:
            self.safety = SafetyControl()
            self.enter_auto()
            decision = self.safety.step(10.2, manual=(0.1, 0, 0), **flags)
            self.assertEqual((decision.mode, decision.applied), (mode, ZERO_SPEEDS))
        self.safety = SafetyControl()
        self.enter_auto()
        self.safety.accept(Command(11, 9, 9, 9, True), 10.2)
        self.assertEqual(self.safety.step(10.2).applied, ZERO_SPEEDS)
        self.assertEqual(self.safety.mode, Mode.MANUAL)

    def test_fall_wins_over_pause_rb_and_new_command(self):
        self.enter_auto()
        decision = self.safety.step(10.2, fallen=True, paused=True, rb=True)
        self.assertEqual((decision.mode, decision.applied), (Mode.FALLEN, ZERO_SPEEDS))
        self.assertEqual(self.safety.step(10.3, fallen=True, rb=True).mode, Mode.FALLEN)

    def test_recovery_and_unpause_need_another_rb_press(self):
        for state in (dict(fallen=True), dict(paused=True)):
            self.safety = SafetyControl()
            self.enter_auto()
            self.safety.step(10.2, **state)
            self.assertEqual(self.safety.step(10.3, a=True).mode, Mode.MANUAL)
            self.assertEqual(self.safety.step(10.4).mode, Mode.MANUAL)

    def test_stop_blocks_entry_until_a_new_movement_command(self):
        self.safety.accept(Command(1, 0.05, 0, 0, True), 0)
        self.assertEqual(self.safety.step(0, rb=True).mode, Mode.MANUAL)
        self.assertIn("stop", self.safety.step(0.1, rb=True).notice)
        self.safety.accept(Command(2, 0.05, 0, 0), 0.2)
        self.assertEqual(self.safety.step(0.2, rb=True).mode, Mode.AUTO)

    def test_stop_is_kept_when_a_newer_movement_command_follows(self):
        self.enter_auto()
        self.safety.accept(Command(11, 0, 0, 0, True), 10.2)
        self.safety.accept(Command(12, 0.05, 0, 0), 10.2)
        self.assertEqual(self.safety.step(10.2).mode, Mode.MANUAL)
        self.assertEqual(self.safety.applied, ZERO_SPEEDS)

    def test_old_repeated_and_malformed_commands_do_not_refresh_time(self):
        self.enter_auto()
        for command in (Command(10, 0.05, 0, 0), Command(9, 0, 0, 0, True),
                        Command(11, float("nan"), 0, 0), object()):
            self.assertFalse(self.safety.accept(command, 10.4))
        self.assertEqual(self.safety.last_received, 10.0)
        self.assertEqual(self.safety.step(10.5).mode, Mode.MANUAL)

    def test_sequence_wrap_and_recovery_expiry(self):
        self.assertTrue(self.safety.accept(Command(protocol.MAX_SEQUENCE, 0.05, 0, 0), 0))
        self.assertTrue(self.safety.accept(Command(0, 0.05, 0, 0), 0.1))
        self.assertFalse(self.safety.accept(Command(protocol.MAX_SEQUENCE, 0.05, 0, 0), 0.2))
        self.safety.expire_commands()
        self.assertEqual(self.safety.step(0.2, rb=True).mode, Mode.MANUAL)
        self.assertFalse(self.safety.accept(Command(0, 0.05, 0, 0), 0.2))

    def test_reason_tracks_the_last_mode_change(self):
        first = self.enter_auto()
        self.assertEqual(self.safety.step(10.2).reason, first.reason)
        stopped = self.safety.step(10.5)
        self.assertEqual(self.safety.step(10.6).reason, stopped.reason)


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.socket = Mock()
        self.socket.recvfrom.side_effect = BlockingIOError
        self.factory = patch("autonomy.command_link.socket.socket", return_value=self.socket)
        self.factory.start()
        self.addCleanup(self.factory.stop)
        self.link = CommandLink()
        self.safety = SafetyControl()

    def test_socket_never_waits_and_drains_newest_command(self):
        old, new = ("old", 1), ("new", 2)
        self.socket.recvfrom.side_effect = [
            (protocol.pack_command(Command(2, 0.02, 0, 0)), old),
            (b"bad", old), (protocol.pack_command(Command(4, 0.04, 0, 0)), new),
            (protocol.pack_command(Command(3, 0.03, 0, 0)), old), BlockingIOError,
        ]
        with patch("autonomy.command_link.time.monotonic", return_value=10):
            self.link.poll(self.safety, 10)
        self.assertEqual(self.safety.last_command.sequence, 4)
        self.assertEqual(self.link.subscriber, new)
        self.assertEqual(self.link.rejected, 2)
        self.assertTrue(self.link.healthy)
        self.socket.setblocking.assert_called_once_with(False)

    def test_state_contains_selected_values_and_uses_ten_hz_interval(self):
        self.link.subscriber = ("viewer", 1)
        self.assertTrue(self.link.send_state(Mode.AUTO, (0.1, 0, 0), (1, 2, 3), (4, 5, 6), "Auto.", 10))
        self.assertFalse(self.link.send_state(Mode.AUTO, (0.1, 0, 0), (1, 2, 3), (4, 5, 6), "Auto.", 10.05))
        self.assertTrue(self.link.send_state(Mode.AUTO, (0.1, 0, 0), (1, 2, 3), (4, 5, 6), "Auto.", 10.1))
        state = protocol.unpack_state(self.socket.sendto.call_args.args[0])
        self.assertEqual((state.sequence, state.gyro, state.accelero), (1, (1, 2, 3), (4, 5, 6)))

    def test_busy_queue_and_socket_error_leave_auto(self):
        self.safety.accept(Command(1, 0.05, 0, 0), 0)
        self.safety.step(0, rb=True)
        self.socket.recvfrom.return_value = (b"x", ("flood", 1))
        self.socket.recvfrom.side_effect = None
        with patch("autonomy.command_link.time.monotonic", return_value=0):
            self.link.poll(self.safety, 0.1)
        self.assertFalse(self.link.healthy)
        self.assertLessEqual(self.socket.recvfrom.call_count, 257)
        self.assertEqual(self.safety.step(0.1, link_ok=self.link.healthy).applied, ZERO_SPEEDS)
        self.socket.recvfrom.side_effect = OSError("Socket failed.")
        self.link.poll(self.safety, 0.2)
        self.assertFalse(self.link.healthy)

    def test_failed_state_send_does_not_delay_or_raise(self):
        self.link.subscriber = ("viewer", 1)
        self.socket.sendto.side_effect = BlockingIOError
        self.assertFalse(self.link.send_state(Mode.MANUAL, (0, 0, 0), (0, 0, 0), (0, 0, 0), "Manual.", 0))

    def test_queued_recovery_commands_are_discarded(self):
        self.socket.recvfrom.side_effect = [(b"queued", ("old", 1)), BlockingIOError]
        self.link.discard_waiting()
        self.assertIsNone(self.safety.last_command)


class SenderTests(unittest.TestCase):
    def socket_context(self):
        sock = Mock()
        sock.__enter__ = Mock(return_value=sock)
        sock.__exit__ = Mock(return_value=False)
        sock.recv.side_effect = BlockingIOError
        return sock

    def test_ctrl_c_sends_no_stop_packet(self):
        sock = self.socket_context()
        address = [(socket.AF_INET, socket.SOCK_DGRAM, 0, "", ("127.0.0.1", 5602))]
        with patch("autonomy.mac_send_command.socket.getaddrinfo", return_value=address), \
             patch("autonomy.mac_send_command.socket.socket", return_value=sock), \
             patch("autonomy.mac_send_command.time.monotonic", return_value=10), \
             patch("autonomy.mac_send_command.select.select", side_effect=KeyboardInterrupt):
            self.assertEqual(sender_main([]), 0)
        self.assertEqual(sock.send.call_count, 1)
        self.assertFalse(protocol.unpack_command(sock.send.call_args.args[0]).stop)

    def test_stop_option_sends_zero_stop_commands_for_one_second(self):
        sock = self.socket_context()
        clock = [10.0]
        def advance(_read, _write, _error, wait):
            clock[0] += max(wait, 0.000001)
            return [], [], []
        address = [(socket.AF_INET, socket.SOCK_DGRAM, 0, "", ("127.0.0.1", 5602))]
        with patch("autonomy.mac_send_command.socket.getaddrinfo", return_value=address), \
             patch("autonomy.mac_send_command.socket.socket", return_value=sock), \
             patch("autonomy.mac_send_command.time.monotonic", side_effect=lambda: clock[0]), \
             patch("autonomy.mac_send_command.select.select", side_effect=advance):
            send_commands(parse_args(["--stop", "--duration", "50"]))
        self.assertAlmostEqual(clock[0] - 10, 1.0, places=5)
        self.assertIn(sock.send.call_count, (10, 11))
        for call in sock.send.call_args_list:
            command = protocol.unpack_command(call.args[0])
            self.assertTrue(command.stop)
            self.assertEqual(command.speeds, ZERO_SPEEDS)


if __name__ == "__main__":
    unittest.main()
