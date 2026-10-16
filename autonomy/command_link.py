"""Receive walking commands and send state without waiting for network traffic."""

import socket
import time

from .protocol import COMMAND_SIZE, MAX_SEQUENCE, State, pack_state, unpack_command


class CommandLink:
    """Keep the latest valid sender address and send state at ten messages per second."""

    def __init__(self, bind="::", port=5602):
        family = socket.AF_INET6 if ":" in bind else socket.AF_INET
        self.socket = socket.socket(family, socket.SOCK_DGRAM)
        try:
            if family == socket.AF_INET6:
                self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16384)
            self.socket.bind((bind, port))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.subscriber = None
        self.next_state = None
        self.state_sequence = 0
        self.healthy = True
        self.rejected = 0

    def poll(self, safety, now=None):
        """Drain queued messages. Leave auto if traffic exceeds the control step budget."""
        now = time.monotonic() if now is None else now
        deadline = time.monotonic() + 0.002
        self.healthy = True
        count = 0
        while True:
            try:
                packet, address = self.socket.recvfrom(COMMAND_SIZE + 1)
            except BlockingIOError:
                return
            except OSError:
                self.healthy = False
                return
            count += 1
            try:
                command = unpack_command(packet)
            except ValueError:
                self.rejected += 1
            else:
                if safety.accept(command, now):
                    self.subscriber = address
                else:
                    self.rejected += 1
            if count >= 256 or time.monotonic() >= deadline:
                try:
                    self.socket.recvfrom(1, socket.MSG_PEEK)
                except BlockingIOError:
                    return
                except OSError:
                    pass
                # Leave auto only when messages still wait beyond the budget.
                self.healthy = False
                return

    def send_state(self, mode, applied, gyro, accelero, reason, now=None):
        """Send a state message when its interval has elapsed."""
        now = time.monotonic() if now is None else now
        if self.subscriber is None or (self.next_state is not None and now < self.next_state):
            return False
        self.next_state = now + 0.1 if self.next_state is None else self.next_state + 0.1
        if self.next_state <= now:
            self.next_state = now + 0.1
        state = State(self.state_sequence, mode, tuple(applied), tuple(gyro), tuple(accelero), reason)
        try:
            self.socket.sendto(pack_state(state), self.subscriber)
        except (OSError, ValueError, TypeError):
            return False
        self.state_sequence = (self.state_sequence + 1) & MAX_SEQUENCE
        return True

    def discard_waiting(self):
        """Discard messages queued during a recovery movement."""
        for _ in range(256):
            try:
                self.socket.recvfrom(COMMAND_SIZE + 1)
            except BlockingIOError:
                return

    def close(self):
        self.socket.close()
