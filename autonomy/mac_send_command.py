"""Send a constant walking command at ten messages per second."""

import argparse
import math
import select
import socket
import time

from .protocol import Command, MAX_DATAGRAM, MAX_SEQUENCE, pack_command, unpack_state


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="bdxmini.local", help="Send commands to this duck host or address.")
    parser.add_argument("--port", type=int, default=5602, help="Use this UDP port. UDP sends separate network messages.")
    parser.add_argument("--forward", type=float, default=0.05, help="Set forward speed in metres per second.")
    parser.add_argument("--sideways", type=float, default=0, help="Set sideways speed in metres per second.")
    parser.add_argument("--turn", type=float, default=0, help="Set turn rate in radians per second.")
    parser.add_argument("--duration", type=float, default=0, help="Stop after this many seconds. Use zero to remove the time limit.")
    parser.add_argument("--stop", action="store_true", help="Send stop commands for one second.")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("Use a port from 1 to 65535.")
    if not all(math.isfinite(value) for value in (args.forward, args.sideways, args.turn, args.duration)):
        parser.error("Use finite speeds and duration.")
    if args.duration < 0:
        parser.error("Duration must be non-negative.")
    try:
        pack_command(Command(0, args.forward, args.sideways, args.turn))
    except ValueError as error:
        parser.error(str(error))
    return args


def send_commands(args):
    family, kind, _, _, address = socket.getaddrinfo(
        args.host, args.port, socket.AF_UNSPEC, socket.SOCK_DGRAM)[0]
    with socket.socket(family, kind) as sock:
        sock.connect(address)
        sock.setblocking(False)
        start = next_send = time.monotonic()
        duration = 1.0 if args.stop else args.duration
        end = start + duration if duration else float("inf")
        # Clock time gives a restarted Mac sender a newer sequence number.
        sequence = time.time_ns() & MAX_SEQUENCE
        sent = 0
        print(f"Command sender ready: {args.host}:{args.port}; stop={args.stop}.", flush=True)
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                speeds = (0.0, 0.0, 0.0) if args.stop else (args.forward, args.sideways, args.turn)
                command = Command(sequence, *speeds, stop=args.stop)
                try:
                    sock.send(pack_command(command))
                    sent += 1
                except OSError as error:
                    print(f"Command send failed: {error}", flush=True)
                sequence = (sequence + 1) & MAX_SEQUENCE
                next_send = now + 0.1
            wait = max(0, min(next_send, end) - time.monotonic())
            select.select([sock], [], [], wait)
            for _ in range(64):
                try:
                    packet = sock.recv(MAX_DATAGRAM)
                except (BlockingIOError, ConnectionRefusedError):
                    break
                try:
                    state = unpack_state(packet)
                except ValueError:
                    continue
                applied = ",".join(f"{value:.3f}" for value in state.applied)
                gyro = ",".join(f"{value:.3f}" for value in state.gyro)
                accelero = ",".join(f"{value:.3f}" for value in state.accelero)
                print(f"state={state.sequence}; mode={state.mode.name.lower()}; applied=({applied}); "
                      f"gyro=({gyro}); accelero=({accelero}); reason={state.reason}", flush=True)
        print(f"Command sender stopped: sent={sent}.", flush=True)
        return sent


def main(argv=None):
    args = parse_args(argv)
    try:
        send_commands(args)
    except KeyboardInterrupt:
        # Send no final packet. Let the duck's watchdog stop auto mode.
        print("Command sender stopped. No stop command was sent.", flush=True)
    except OSError as error:
        print(f"Command sender failed: {error}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
