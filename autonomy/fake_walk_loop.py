"""Test the command link at 50 steps per second without hardware imports."""

import argparse
import json
import math
from pathlib import Path
import time

from .command_link import CommandLink
from .protocol import Mode
from .safety import SafetyControl


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="::", help="Listen at this address. Use :: for IPv4 and IPv6.")
    parser.add_argument("--port", type=int, default=5602, help="Listen on this UDP port. UDP sends separate network messages.")
    parser.add_argument("--duration", type=float, default=15, help="Run for this many seconds.")
    parser.add_argument("--auto-on-command", action="store_true", help="Simulate one RB press after the first movement command.")
    parser.add_argument("--result-json", type=Path, help="Save measured results at this path.")
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535 or not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("Use a valid port and a positive, finite duration.")
    safety = SafetyControl()
    link = CommandLink(args.bind, args.port)
    start = deadline = time.monotonic()
    start_cpu = time.process_time()
    armed = False
    steps = states = overruns = 0
    maximum_step = 0
    watchdog_delays = []
    print(f"Fake loop ready: address={link.socket.getsockname()}; motors=absent.", flush=True)
    try:
        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            link.poll(safety, now)
            rb = (args.auto_on_command and not armed and safety.last_command is not None
                  and not safety.last_command.stop)
            if rb:
                armed = True
            decision = safety.step(now, rb=rb, link_ok=link.healthy)
            if decision.changed:
                print(f"Fake mode: {decision.mode.name.lower()}; {decision.reason}", flush=True)
                if decision.previous_mode == Mode.AUTO and "timeout" in decision.reason:
                    delay = now - safety.last_received
                    watchdog_delays.append(delay)
                    print(f"Watchdog stop: {delay:.6f} seconds after the last accepted command.", flush=True)
            states += link.send_state(decision.mode, decision.applied, (0, 0, 0),
                                      (0, 0, -9.81), decision.reason, now)
            steps += 1
            took = time.monotonic() - now
            maximum_step = max(maximum_step, took)
            overruns += took > 0.02
            deadline += 0.02
            time.sleep(max(0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    elapsed = time.monotonic() - start
    result = {"seconds": elapsed, "steps": steps, "steps_per_second": steps / elapsed,
              "state_messages": states, "cpu_percent_one_core": 100 * (time.process_time() - start_cpu) / elapsed,
              "maximum_step_ms": maximum_step * 1000, "overruns": overruns,
              "watchdog_delays_seconds": watchdog_delays, "rejected_messages": link.rejected,
              "final_mode": safety.mode.name.lower(), "final_applied": safety.applied}
    print(json.dumps(result), flush=True)
    if args.result_json:
        args.result_json.write_text(json.dumps(result, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
