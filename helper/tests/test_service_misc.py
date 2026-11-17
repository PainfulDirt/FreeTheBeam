"""Regression: camera line finding and the driver client
(helper/lidar_camera.py, helper/lidar_service.py Middleware).
K1 bug hunt 2026-11-16:

- dark lines on a bright plate are measured like bright lines on a dark
  plate (skipped without numpy/PIL);
- a slow, non-idempotent driver command is not re-sent after the timeout.
"""
import os
import socket
import sys
import threading
import time
import unittest

try:
    import numpy as np
    import PIL  # noqa: F401
except ImportError:  # pragma: no cover (CI installs them)
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."),
                os.path.join(HERE, "..", "..", "klipper")]  # lidar.py
import lidar_service as svc  # noqa: E402
if np is not None:
    import lidar_camera as cam  # noqa: E402


def lines_img(plate, line, cols, w_px=23):
    img = np.full((1200, 1600), float(plate))
    for c in cols:
        img[:, int(c - w_px / 2):int(c + w_px / 2)] = line
    return img + np.random.default_rng(1).normal(0, 2, img.shape)


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class DarkLines(unittest.TestCase):
    def test_dark_lines_measured_like_bright(self):
        xs = [100.0 + 2.5 * i for i in range(9)]
        cols = [800 + (110.0 - x) / cam.MM_PX for x in xs]
        bright = lines_img(40, 200, cols)     # white PLA on black PEI
        dark = lines_img(200, 40, cols)       # dark PLA on a bright plate
        nb = len(cam.line_widths(bright, cam.find_lines(bright, xs, 110.0)[4]))
        nd = len(cam.line_widths(dark, cam.find_lines(dark, xs, 110.0)[4]))
        self.assertGreater(nb, 1000)
        self.assertGreater(nd, 1000, "dark line: no widths measured")


class MiddlewareRetry(unittest.TestCase):
    def test_slow_command_not_sent_twice(self):
        got = []
        timers = []
        srv = socket.socket()
        srv.bind(("127.0.0.1", 0))
        srv.listen(4)
        srv.settimeout(5.0)

        def bridge():         # a driver that answers after 1.5 s
            for _ in range(2):
                try:
                    c, _ = srv.accept()
                except OSError:
                    return
                got.append(c.recv(4096))
                t = threading.Timer(1.5, c.close)
                timers.append(t)
                t.start()
        th = threading.Thread(target=bridge, daemon=True)
        th.start()
        mw = svc.Middleware(srv.getsockname())
        try:
            mw.call({"control": "self_test"}, timeout=0.5)
        except RuntimeError:
            pass
        time.sleep(0.3)       # let a re-sent request reach the bridge
        try:
            self.assertEqual(len(got), 1, "non-idempotent command re-sent")
        finally:
            mw.close()
            srv.close()
            for t in timers:
                t.cancel()


if __name__ == "__main__":
    unittest.main(verbosity=2)
