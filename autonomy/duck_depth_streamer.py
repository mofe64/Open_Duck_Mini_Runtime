"""Send rotated depth images to the latest viewer."""

import argparse
from contextlib import contextmanager
import os
import re
import select
import signal
import socket
import subprocess
import time

# Limit NumPy worker threads before NumPy loads.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np

from .depth import RANGE_MM, centre_distance, downsample_depth, rotate_frames
from .protocol import MAX_DATAGRAM, pack_depth, unpack_hello


class Subscription:
    """Keep the latest viewer address until its hello messages stop."""

    def __init__(self, timeout=3.0):
        self.timeout = timeout
        self.address = None
        self.last_hello = float("-inf")

    def receive(self, packet, address, now):
        try:
            unpack_hello(packet)
        except ValueError:
            return
        self.address, self.last_hello = address, now

    def current(self, now):
        return self.address if now - self.last_hello < self.timeout else None


def receive_hellos(sock, subscription):
    # Limit each pass so incoming traffic cannot delay camera reads forever.
    for _ in range(32):
        try:
            packet, address = sock.recvfrom(MAX_DATAGRAM)
        except BlockingIOError:
            return
        subscription.receive(packet, address, time.monotonic())


def require_ok(result, operation):
    if result != 0:
        raise RuntimeError(f"{operation} failed: {result}")


def read_video_format(device):
    output = subprocess.check_output(
        ["v4l2-ctl", "-d", device, "--get-fmt-video"], text=True, timeout=5)
    size = re.search(r"Width/Height\s*:\s*(\d+)/(\d+)", output)
    pixel = re.search(r"Pixel Format\s*:\s*'([^']+)'", output)
    if not size or not pixel:
        raise RuntimeError("Cannot read the camera video format.")
    return int(size[1]), int(size[2]), pixel[1]


@contextmanager
def camera_session(ac, fps, device=0):
    """Use the tested start sequence. Restore the previous video format after use."""
    node = f"/dev/video{device}"
    previous = read_video_format(node)
    camera = ac.ArducamCamera()
    opened = started = False
    try:
        require_ok(camera.open(ac.Connection.CSI, device), "Camera open")
        opened = True
        require_ok(camera.start(ac.FrameType.DEPTH), "Camera start")
        started = True
        require_ok(camera.setControl(ac.Control.RANGE, RANGE_MM), "Set range")
        if camera.getControl(ac.Control.RANGE) != RANGE_MM:
            raise RuntimeError("Camera did not keep the 2000 mm range.")
        info = camera.getCameraInfo()
        if (info.width, info.height) != (240, 180):
            raise RuntimeError("Camera must produce 240 by 180 depth images.")
        rate_result = camera.setControl(ac.Control.FRAME_RATE, fps)
        if rate_result != 0:
            # This camera produces 30 depth frames per second before frame skips.
            divisor = max(1, 30 // fps)
            require_ok(camera.setControl(ac.Control.AUTO_FRAME_RATE, 0), "Disable automatic frame rate")
            require_ok(camera.setControl(ac.Control.SKIP_FRAME_LOOP, divisor), "Set frame skip loop")
            require_ok(camera.setControl(ac.Control.SKIP_FRAME, divisor - 1), "Set frame skip count")
            print(f"Camera rate control is unavailable. Keep one frame in {divisor} before depth processing.", flush=True)
        print(f"Camera ready: 240x180; range={RANGE_MM} mm; rate={fps} frames/s.", flush=True)
        yield camera
    finally:
        try:
            try:
                if started:
                    require_ok(camera.stop(), "Camera stop")
            finally:
                if opened:
                    require_ok(camera.close(), "Camera close")
                    print("Camera closed.", flush=True)
        finally:
            if read_video_format(node) != previous:
                width, height, pixel = previous
                subprocess.run([
                    "v4l2-ctl", "-d", node,
                    f"--set-fmt-video=width={width},height={height},pixelformat={pixel}",
                ], check=True, timeout=5)
                if read_video_format(node) != previous:
                    raise RuntimeError("Cannot restore the previous video format.")


def stream(camera, ac, sock, args):
    subscription = Subscription(args.hello_timeout)
    period = 1.0 / args.fps
    next_frame = last_status = time.monotonic()
    number = frames = sent = timeouts = 0
    centre = None
    while True:
        receive_hellos(sock, subscription)
        now = time.monotonic()
        if now < next_frame:
            select.select([sock], [], [], min(next_frame - now, 0.1))
            continue
        next_frame = now + period
        frame = camera.requestFrame(200)
        if frame is None:
            timeouts += 1
        else:
            timestamp = time.monotonic()
            try:
                if not isinstance(frame, ac.DepthData):
                    raise RuntimeError("Camera returned an unexpected frame type.")
                # Rotate both images before filtering or measuring any pixels.
                depth, confidence = rotate_frames(frame.depth_data, frame.confidence_data)
                if depth.shape != (180, 240):
                    raise RuntimeError("Camera returned an unexpected depth size.")
                reduced = downsample_depth(depth, confidence, args.min_confidence)
            finally:
                require_ok(camera.releaseFrame(frame), "Release frame")
            frames += 1
            centre = centre_distance(reduced)
            receive_hellos(sock, subscription)
            address = subscription.current(time.monotonic())
            if address is not None:
                try:
                    sock.sendto(pack_depth(number, timestamp, reduced), address)
                    sent += 1
                except OSError as error:
                    print(f"Frame send failed: {error}", flush=True)
            number = (number + 1) & 0xFFFFFFFF
        now = time.monotonic()
        if now - last_status >= args.status_interval:
            address = subscription.current(now)
            host = f"[{address[0]}]" if address and ":" in address[0] else (address[0] if address else "none")
            viewer = f"{host}:{address[1]}" if address else "none"
            distance = f"{centre:.0f} mm" if centre is not None else "no valid depth"
            print(f"frames/s={frames / (now - last_status):.1f}; sent={sent}; "
                  f"viewer={viewer}; centre={distance}; timeouts={timeouts}", flush=True)
            last_status, frames, sent, timeouts = now, 0, 0, 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="::", help="Listen at this address. Use :: for IPv4 and IPv6.")
    parser.add_argument("--port", type=int, default=5601, help="Use this UDP port. UDP sends separate network messages.")
    parser.add_argument("--fps", type=int, default=10, help="Set the camera and send rate in frames per second.")
    parser.add_argument("--min-confidence", type=float, default=30, help="Reject pixels below this confidence value.")
    parser.add_argument("--hello-timeout", type=float, default=3, help="Stop sending after this many seconds without a hello.")
    parser.add_argument("--status-interval", type=float, default=5, help="Print status after this many seconds.")
    parser.add_argument("--device", type=int, default=0, help="Use this camera device number.")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 1 <= args.fps <= 30 or args.device < 0:
        parser.error("Use a valid port, device, and frame rate from 1 to 30.")
    if not np.isfinite(args.min_confidence) or args.min_confidence < 0:
        parser.error("Use a finite, non-negative confidence value.")
    if not all(np.isfinite(value) and value > 0 for value in (args.hello_timeout, args.status_interval)):
        parser.error("Use positive, finite time intervals.")
    return args


def main(argv=None):
    args = parse_args(argv)

    def stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        import ArducamDepthCamera as ac

        family = socket.AF_INET6 if ":" in args.bind else socket.AF_INET
        with socket.socket(family, socket.SOCK_DGRAM) as sock:
            if family == socket.AF_INET6:
                sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
            sock.bind((args.bind, args.port))
            sock.setblocking(False)
            with camera_session(ac, args.fps, args.device) as camera:
                stream(camera, ac, sock, args)
    except KeyboardInterrupt:
        print("Streamer stopped.", flush=True)
        return 0
    except (OSError, RuntimeError, ValueError, ImportError, subprocess.SubprocessError) as error:
        print(f"Streamer failed: {error}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
