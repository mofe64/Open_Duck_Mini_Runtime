"""Test depth messages and image operations without camera hardware."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from autonomy import protocol
from autonomy.depth import centre_distance, downsample_depth, rotate_frames, zone_distances
from autonomy.duck_depth_streamer import Subscription, camera_session, stream
from autonomy.mac_depth_viewer import FrameStats


class ProtocolTests(unittest.TestCase):
    def test_depth_round_trip_and_wire_order(self):
        depth = np.array([[0, 1, 256], [500, 2000, 65535]], dtype=np.uint16)
        packet = protocol.pack_depth(42, 123.125, depth[:, ::-1])
        frame = protocol.unpack_depth(packet)
        self.assertEqual((frame.number, frame.timestamp), (42, 123.125))
        np.testing.assert_array_equal(frame.depth_mm, depth[:, ::-1])
        self.assertEqual(frame.depth_mm.dtype, np.uint16)
        self.assertEqual(packet[-6:], b"\xff\xff\x07\xd0\x01\xf4")

    def test_hello_round_trip(self):
        self.assertIsNone(protocol.unpack_hello(protocol.pack_hello()))

    def test_default_depth_fits_one_message(self):
        packet = protocol.pack_depth(0, 0, np.zeros((45, 60), dtype=np.uint16))
        self.assertEqual(len(packet), 5424)
        self.assertLess(len(packet), protocol.MAX_DATAGRAM)

    def test_rejects_large_or_wrong_depth_arrays(self):
        for depth in (np.zeros((180, 240), np.uint16), np.zeros((2, 3)),
                      np.zeros((0, 3), np.uint16), np.zeros(3, np.uint16)):
            with self.subTest(shape=depth.shape), self.assertRaises(ValueError):
                protocol.pack_depth(0, 0, depth)

    def test_rejects_damaged_messages(self):
        packet = protocol.pack_depth(3, 4, np.zeros((3, 3), dtype=np.uint16))
        changes = [b"", packet[:-1], packet + b"x", b"FAIL" + packet[4:]]
        for offset, value in ((4, 2), (5, protocol.HELLO), (7, 25), (20, 0), (21, 0)):
            changed = bytearray(packet)
            changed[offset] = value
            if offset == 20:
                changed[21] = 0
            if bytes(changed) != packet:
                changes.append(bytes(changed))
        for damaged in changes:
            with self.subTest(packet=damaged), self.assertRaises(ValueError):
                protocol.unpack_depth(damaged)
        for damaged in (b"", protocol.pack_hello() + b"x", packet):
            with self.assertRaises(ValueError):
                protocol.unpack_hello(damaged)

    def test_rejects_bad_numbers_and_times(self):
        depth = np.zeros((1, 3), np.uint16)
        for number, stamp in ((-1, 0), (2 ** 32, 0), (0, -1), (0, float("nan")), (0, float("inf"))):
            with self.subTest(number=number, stamp=stamp), self.assertRaises(ValueError):
                protocol.pack_depth(number, stamp, depth)
        packet = protocol.pack_depth(0, 0, depth)
        for stamp in (-1, float("nan"), float("inf")):
            damaged = packet[:8] + protocol.DEPTH_FIELDS.pack(0, stamp, 3, 1) + packet[24:]
            with self.assertRaises(ValueError):
                protocol.unpack_depth(damaged)


class DepthTests(unittest.TestCase):
    def test_rotation_keeps_depth_and_confidence_aligned(self):
        depth = np.arange(24).reshape(4, 6)
        confidence = depth + 100
        rotated, rotated_confidence = rotate_frames(depth, confidence)
        np.testing.assert_array_equal(rotated, depth[::-1, ::-1])
        np.testing.assert_array_equal(rotated_confidence, confidence[::-1, ::-1])
        np.testing.assert_array_equal(depth, np.arange(24).reshape(4, 6))

    def test_block_medians_ignore_invalid_pixels(self):
        depth = np.zeros((8, 8), dtype=np.float32)
        confidence = np.full((8, 8), 100, dtype=np.float32)
        depth[:4, :4] = np.arange(100, 1700, 100).reshape(4, 4)
        depth[:4, 4:] = np.array([
            [100, 300, np.nan, 0], [2001, -5, np.inf, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        depth[4:, 4:] = 700
        confidence[4:, 4:] = 29
        result = downsample_depth(depth, confidence)
        np.testing.assert_array_equal(result, [[850, 200], [0, 0]])
        self.assertEqual(result.dtype, np.uint16)

    def test_confidence_boundary_and_range_boundary(self):
        depth = np.full((4, 4), 2000, dtype=np.float32)
        confidence = np.full((4, 4), 30, dtype=np.float32)
        self.assertEqual(downsample_depth(depth, confidence)[0, 0], 2000)
        confidence[:] = np.nan
        self.assertEqual(downsample_depth(depth, confidence)[0, 0], 0)

    def test_reduction_matches_separate_block_medians(self):
        random = np.random.default_rng(42)
        depth = random.uniform(-100, 2200, (24, 32)).astype(np.float32)
        confidence = random.uniform(0, 100, depth.shape).astype(np.float32)
        depth[::3, ::5] = np.nan
        confidence[::2, ::7] = np.nan
        depth, confidence = rotate_frames(depth, confidence)
        expected = np.zeros((6, 8), np.uint16)
        for row in range(6):
            for column in range(8):
                area = (slice(row * 4, row * 4 + 4), slice(column * 4, column * 4 + 4))
                values, quality = depth[area], confidence[area]
                valid = (np.isfinite(values) & (values > 0) & (values <= 2000)
                         & np.isfinite(quality) & (quality >= 30))
                if valid.any():
                    expected[row, column] = np.rint(np.median(values[valid]))
        np.testing.assert_array_equal(downsample_depth(depth, confidence), expected)

    def test_rotation_precedes_reduction(self):
        depth = np.zeros((8, 12), np.float32)
        depth[:4, :4] = 100
        confidence = np.full(depth.shape, 100)
        result = downsample_depth(*rotate_frames(depth, confidence))
        np.testing.assert_array_equal(result, [[0, 0, 0], [0, 0, 100]])

    def test_zones_use_valid_10th_percentile(self):
        depth = np.array([[0, 100, 0, 0, 900, 1000], [200, 300, 0, 0, 1100, 1200]])
        self.assertEqual(zone_distances(depth), (120.0, None, 930.0))
        self.assertIsNone(centre_distance(depth))
        depth[:, 2:4] = [[0, 400], [600, 800]]
        self.assertEqual(centre_distance(depth), 600)

    def test_rejects_wrong_image_shapes(self):
        for shape in ((3, 4), (4, 3), (0, 4)):
            with self.assertRaises(ValueError):
                downsample_depth(np.zeros(shape), np.zeros(shape))
        with self.assertRaises(ValueError):
            downsample_depth(np.zeros((4, 4)), np.zeros((8, 8)))
        with self.assertRaises(ValueError):
            zone_distances(np.zeros((4, 2)))


class TransportTests(unittest.TestCase):
    def test_latest_hello_wins_and_expires(self):
        subscription = Subscription()
        first, second = ("127.0.0.1", 1000), ("127.0.0.1", 2000)
        self.assertIsNone(subscription.current(0))
        subscription.receive(protocol.pack_hello(), first, 0)
        subscription.receive(b"bad", second, 1)
        self.assertEqual(subscription.current(2), first)
        subscription.receive(protocol.pack_hello(), second, 2)
        self.assertEqual(subscription.current(4.9), second)
        self.assertIsNone(subscription.current(5))

    def test_missing_frames_duplicates_and_late_frames(self):
        stats = FrameStats()
        depth = np.zeros((1, 3), np.uint16)
        self.assertTrue(stats.accept(protocol.DepthFrame(10, 100, depth), 0))
        self.assertTrue(stats.accept(protocol.DepthFrame(13, 103, depth), 1))
        self.assertFalse(stats.accept(protocol.DepthFrame(13, 103, depth), 1.1))
        self.assertFalse(stats.accept(protocol.DepthFrame(12, 102, depth), 1.2))
        self.assertEqual((stats.received, stats.dropped), (2, 2))

    def test_frame_counter_wrap_and_streamer_restart(self):
        stats = FrameStats()
        depth = np.zeros((1, 3), np.uint16)
        for number, stamp, now in ((0xFFFFFFFF, 100, 0), (0, 101, 1), (5, 102, 2), (0, 103, 2.1)):
            self.assertTrue(stats.accept(protocol.DepthFrame(number, stamp, depth), now))
        self.assertEqual(stats.dropped, 4)
        self.assertTrue(stats.accept(protocol.DepthFrame(0, 1, depth), 6))
        self.assertEqual(stats.dropped, 4)


class CameraCleanupTests(unittest.TestCase):
    def setUp(self):
        self.camera = Mock()
        self.camera.open.return_value = self.camera.start.return_value = 0
        self.camera.setControl.return_value = self.camera.stop.return_value = self.camera.close.return_value = 0
        self.camera.getControl.return_value = 2000
        self.camera.getCameraInfo.return_value = SimpleNamespace(width=240, height=180)
        self.ac = SimpleNamespace(ArducamCamera=Mock(return_value=self.camera),
                                  Connection=SimpleNamespace(CSI=0), FrameType=SimpleNamespace(DEPTH=1),
                                  Control=SimpleNamespace(RANGE=0, FRAME_RATE=6, AUTO_FRAME_RATE=9,
                                                          SKIP_FRAME_LOOP=8, SKIP_FRAME=7))

    @patch("autonomy.duck_depth_streamer.read_video_format", return_value=(240, 180, "Y12P"))
    def test_unsupported_rate_uses_camera_frame_skips(self, _read):
        self.camera.setControl.side_effect = [0, -1, 0, 0, 0]
        with camera_session(self.ac, 10):
            pass
        self.assertEqual([call.args for call in self.camera.setControl.call_args_list],
                         [(0, 2000), (6, 10), (9, 0), (8, 3), (7, 2)])

    @patch("autonomy.duck_depth_streamer.read_video_format", return_value=(240, 180, "Y12P"))
    def test_interrupt_stops_and_closes_camera(self, _read):
        with self.assertRaises(KeyboardInterrupt):
            with camera_session(self.ac, 10):
                raise KeyboardInterrupt
        self.camera.stop.assert_called_once()
        self.camera.close.assert_called_once()

    @patch("autonomy.duck_depth_streamer.read_video_format", return_value=(240, 180, "Y12P"))
    def test_failed_stop_still_closes_camera(self, _read):
        self.camera.stop.return_value = -1
        with self.assertRaises(RuntimeError):
            with camera_session(self.ac, 10):
                pass
        self.camera.close.assert_called_once()

    @patch("autonomy.duck_depth_streamer.subprocess.run")
    @patch("autonomy.duck_depth_streamer.read_video_format", side_effect=[
        (240, 180, "Y12P"), (240, 180, "Y16 "), (240, 180, "Y12P")])
    def test_restores_video_format_after_start_failure(self, _read, restore):
        self.camera.start.return_value = -1
        with self.assertRaises(RuntimeError):
            with camera_session(self.ac, 10):
                pass
        self.camera.stop.assert_not_called()
        self.camera.close.assert_called_once()
        restore.assert_called_once()
        self.assertIn("--set-fmt-video=width=240,height=180,pixelformat=Y12P", restore.call_args.args[0])

    def test_processing_failure_releases_frame(self):
        frame = SimpleNamespace(depth_data=np.zeros((4, 4)), confidence_data=np.zeros((4, 4)))
        self.camera.requestFrame.return_value = frame
        self.camera.releaseFrame.return_value = 0
        self.ac.DepthData = SimpleNamespace
        args = SimpleNamespace(fps=10, hello_timeout=3)
        sock = Mock()
        sock.recvfrom.side_effect = BlockingIOError
        with self.assertRaises(RuntimeError):
            stream(self.camera, self.ac, sock, args)
        self.camera.releaseFrame.assert_called_once_with(frame)


if __name__ == "__main__":
    unittest.main()
