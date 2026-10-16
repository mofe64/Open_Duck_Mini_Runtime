"""Select walking speeds without sockets or hardware access."""

from dataclasses import dataclass
import math

from .protocol import Command, Mode, newer_sequence, validate_command


ZERO_SPEEDS = (0.0, 0.0, 0.0)
COMMAND_TIMEOUT = 0.5
STICK_LIMIT = 0.2


@dataclass(frozen=True)
class Decision:
    """Describe the speeds and mode selected for one control step."""

    previous_mode: Mode
    mode: Mode
    applied: tuple
    reason: str
    notice: str = ""

    @property
    def changed(self):
        return self.mode != self.previous_mode


class SafetyControl:
    """Keep every autonomous mode change and speed limit on the duck."""

    def __init__(self, forward_cap=0.10, sideways_cap=0.05, turn_cap=0.6, control_period=0.02):
        self.caps = (forward_cap, sideways_cap, turn_cap)
        if not all(math.isfinite(value) and value >= 0 for value in self.caps):
            raise ValueError("Speed caps must be finite and non-negative.")
        if not math.isfinite(control_period) or not 0 < control_period < COMMAND_TIMEOUT:
            raise ValueError("Control period must be positive and below 0.5 seconds.")
        self.control_period = control_period
        self.mode = Mode.MANUAL
        self.reason = "Started in manual mode."
        self.applied = ZERO_SPEEDS
        self.last_command = None
        self.last_received = float("-inf")
        self._stop_pending = False

    def accept(self, command: Command, now: float) -> bool:
        """Keep only a newer valid command. Do not refresh time for rejected commands."""
        try:
            validate_command(command)
            if not math.isfinite(now):
                return False
        except (ValueError, TypeError, AttributeError):
            return False
        if self.last_command is not None and not newer_sequence(command.sequence, self.last_command.sequence):
            return False
        self.last_command, self.last_received = command, now
        # Keep a stop request even when a newer movement command follows it.
        self._stop_pending |= command.stop
        return True

    def _fresh(self, now, margin=0.0):
        return self.last_command is not None and 0 <= now - self.last_received < COMMAND_TIMEOUT - margin

    def expire_commands(self):
        """Require a new command after a recovery movement."""
        self.last_received = float("-inf")

    def step(self, now, manual=ZERO_SPEEDS, sticks=(0.0, 0.0, 0.0, 0.0),
             rb=False, a=False, paused=False, fallen=False, link_ok=True):
        """Apply fall, pause, stop, stick, link, and time checks before auto speeds."""
        if not math.isfinite(now):
            raise ValueError("Control time must be finite.")
        if len(manual) != 3 or not all(math.isfinite(value) for value in manual):
            raise ValueError("Manual speeds must contain three finite values.")
        previous = self.mode
        mode = previous
        reason = self.reason
        notice = ""
        stop = self._stop_pending
        self._stop_pending = False
        stick_valid = (sticks is not None and len(sticks) == 4
                       and all(math.isfinite(value) and abs(value) <= 1 for value in sticks))
        override = stick_valid and (math.hypot(*sticks[:2]) > STICK_LIMIT
                                    or math.hypot(*sticks[2:]) > STICK_LIMIT)
        applied = tuple(manual)

        if fallen:
            mode, reason, applied = Mode.FALLEN, "Fall detected.", ZERO_SPEEDS
        elif paused:
            mode, reason, applied = Mode.PAUSED, "Button A paused walking.", ZERO_SPEEDS
        elif a:
            mode, reason, applied = Mode.MANUAL, "Button A selected manual mode.", ZERO_SPEEDS
        elif stop:
            mode, reason, applied = Mode.MANUAL, "Mac stop command.", ZERO_SPEEDS
        elif not stick_valid:
            mode, reason, applied = Mode.MANUAL, "Controller stick data is unavailable.", ZERO_SPEEDS
            if rb:
                notice = "Auto refused: controller stick data is unavailable."
        elif not link_ok:
            mode, reason = Mode.MANUAL, "Command link cannot keep up."
            if rb:
                notice = "Auto refused: command link cannot keep up."
        elif override:
            mode, reason = Mode.MANUAL, "Controller stick override."
            if rb and previous != Mode.AUTO:
                notice = "Auto refused: release both sticks."
        elif previous in (Mode.PAUSED, Mode.FALLEN):
            mode, reason = Mode.MANUAL, "Walking resumed in manual mode."
        elif previous == Mode.AUTO:
            if rb:
                mode, reason = Mode.MANUAL, "RB selected manual mode."
            elif not self._fresh(now, self.control_period):
                # Reserve one step so speeds reach zero by the timeout.
                mode, reason = Mode.MANUAL, "Command timeout before 0.5 seconds."
        elif rb:
            if not self._fresh(now):
                notice = "Auto refused: no valid command in the last 0.5 seconds."
            elif self.last_command.stop:
                notice = "Auto refused: the latest command requests a stop."
            elif not self._fresh(now, self.control_period):
                notice = "Auto refused: the command is too close to its timeout."
            else:
                mode, reason = Mode.AUTO, "RB selected auto mode."

        if previous == Mode.AUTO and mode != Mode.AUTO:
            applied = ZERO_SPEEDS
        elif mode == Mode.AUTO:
            applied = tuple(max(-cap, min(cap, value))
                            for cap, value in zip(self.caps, self.last_command.speeds))
        if mode != previous:
            self.reason = reason
        self.mode, self.applied = mode, applied
        return Decision(previous, mode, applied, self.reason, notice)
