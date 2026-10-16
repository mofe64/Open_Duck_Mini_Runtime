"""Show live depth images from the duck."""

import argparse
from dataclasses import dataclass
import select
import socket
import time
from typing import Optional

import numpy as np

from .depth import RANGE_MM, ZONE_NAMES, zone_distances
from .protocol import MAX_DATAGRAM, unpack_depth, pack_hello


@dataclass
class FrameStats:
    """Count missing frames. Ignore duplicate and late frames."""

    number: Optional[int] = None
    timestamp: float = -1
    last_received: float = float("-inf")
    dropped: int = 0
    received: int = 0

    def accept(self, frame, now):
        if self.number is not None and now - self.last_received < 3:
            if frame.timestamp <= self.timestamp:
                return False
            gap = (frame.number - self.number) & 0xFFFFFFFF
            if gap == 0:
                return False
            # A large backward step with newer Pi time marks a restarted streamer.
            if gap < 0x80000000:
                self.dropped += gap - 1
        self.number, self.timestamp = frame.number, frame.timestamp
        self.last_received = now
        self.received += 1
        return True


def distance_text(value):
    return "no valid depth" if value is None else f"{value:.0f} mm"


def render_image(cv2, frame, stats, fps, now, scale):
    age = now - stats.last_received
    stale = frame is None or age > 1
    depth = np.zeros((45, 60), dtype=np.uint16) if stale else frame.depth_mm
    colours = np.rint(255 * (1 - np.clip(depth, 0, RANGE_MM) / RANGE_MM)).astype(np.uint8)
    image = cv2.applyColorMap(colours, cv2.COLORMAP_TURBO)
    image[depth == 0] = 0
    height, width = depth.shape
    image = cv2.resize(image, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)
    view_height, view_width = image.shape[:2]
    canvas = np.full((view_height + 120, view_width, 3), 24, dtype=np.uint8)
    canvas[:view_height] = image
    left = 0
    for name, columns, distance in zip(ZONE_NAMES, np.array_split(np.arange(width), 3), zone_distances(depth)):
        right = (int(columns[-1]) + 1) * scale
        cv2.rectangle(canvas, (left, 0), (right - 1, view_height - 1), (240, 240, 240), 1)
        cv2.rectangle(canvas, (left + 1, 1), (right - 2, 52), (24, 24, 24), -1)
        cv2.putText(canvas, name, (left + 8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(canvas, distance_text(distance), (left + 8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        left = right
    age_text = "waiting" if frame is None else f"age={age:.1f}s"
    status = f"frames/s={fps:.1f}   dropped={stats.dropped}   {age_text}"
    cv2.putText(canvas, status, (10, view_height + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    state = "NO LIVE DEPTH" if stale else "Live depth"
    cv2.putText(canvas, state + "   Q: close", (10, view_height + 51), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (80, 140, 255) if stale else (180, 220, 180), 1)
    gradient = np.linspace(255, 0, view_width, dtype=np.uint8)[None, :]
    canvas[view_height + 65:view_height + 81] = cv2.applyColorMap(
        np.repeat(gradient, 16, axis=0), cv2.COLORMAP_TURBO)
    cv2.putText(canvas, "Near: 0 mm    Far: 2000 mm    Black: no valid depth",
                (10, view_height + 105), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
    return canvas


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="bdxmini.local", help="Receive images from this duck host.")
    parser.add_argument("--port", type=int, default=5601, help="Use this UDP port. UDP sends separate network messages.")
    parser.add_argument("--scale", type=int, default=12, help="Enlarge each depth pixel by this factor.")
    parser.add_argument("--headless", action="store_true", help="Print status without opening a window.")
    parser.add_argument("--duration", type=float, default=0, help="Stop after this many seconds. Use zero to run without a time limit.")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 6 <= args.scale <= 24:
        parser.error("Use a valid port and a scale from 6 to 24.")
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error("Use a finite, non-negative duration.")
    return args


def run_viewer(args, cv2=None):
    stats = FrameStats()
    frame = None
    previous_count = fps = 0
    title = "Duck depth"
    if cv2 is not None:
        cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
    try:
        family, kind, _, _, address = socket.getaddrinfo(
            args.host, args.port, socket.AF_UNSPEC, socket.SOCK_DGRAM)[0]
        with socket.socket(family, kind) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            sock.connect(address)
            sock.setblocking(False)
            start = last_status = next_hello = next_draw = time.monotonic()
            print(f"Viewer ready: {args.host}:{args.port}.", flush=True)
            while not args.duration or time.monotonic() - start < args.duration:
                now = time.monotonic()
                if now >= next_hello:
                    try:
                        sock.send(pack_hello())
                    except OSError:
                        pass
                    next_hello = now + 1
                select.select([sock], [], [], 0.03)
                # Drain queued images to show the newest valid frame.
                for _ in range(64):
                    try:
                        packet = sock.recv(MAX_DATAGRAM)
                    except BlockingIOError:
                        break
                    except ConnectionRefusedError:
                        break
                    try:
                        candidate = unpack_depth(packet)
                    except ValueError:
                        continue
                    if stats.accept(candidate, time.monotonic()):
                        frame = candidate
                now = time.monotonic()
                if now - last_status >= 1:
                    fps = (stats.received - previous_count) / (now - last_status)
                    previous_count, last_status = stats.received, now
                    if args.headless:
                        live = frame is not None and now - stats.last_received <= 1
                        values = zone_distances(frame.depth_mm) if live else (None,) * 3
                        zones = "; ".join(f"{name.lower()}={distance_text(value)}"
                                          for name, value in zip(ZONE_NAMES, values))
                        print(f"frames/s={fps:.1f}; dropped={stats.dropped}; {zones}", flush=True)
                if cv2 is not None:
                    if now >= next_draw:
                        cv2.imshow(title, render_image(cv2, frame, stats, fps, now, args.scale))
                        next_draw = now + 0.1
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                    if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                        break
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()
    print(f"Viewer stopped: received={stats.received}; dropped={stats.dropped}.", flush=True)
    return stats


def main(argv=None):
    args = parse_args(argv)
    try:
        cv2 = None
        if not args.headless:
            import cv2
        run_viewer(args, cv2)
    except KeyboardInterrupt:
        print("Viewer stopped.", flush=True)
    except (OSError, ImportError) as error:
        print(f"Viewer failed: {error}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
