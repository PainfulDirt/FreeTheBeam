"""Regression (K1 bug hunt 2026-11-16 #2, #7, #20): the printer <-> driver
and printer <-> helper request/reply channels (Lidar._call/_recv/_svc/
_svc_send/_frame_msg).

#2: _svc's connect must not freeze the Klipper reactor when the helper
    does not answer SYNs (non-blocking connect).
#7: two replies glued in one recv are split on ETX; a G-code error during
    LIDAR_TIMING does not leave the driver channel off by one; a bad
    forwarded frame does not put an error reply into the service stream.
#20: stream mode 1 ("direct-m1") still asks the driver for prefetched
    frames.
"""
import json
import os
import socket
import sys
import tempfile
import threading
import time
import unittest

import regress_fakes as F

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "helper"))
import lidar_service  # noqa: E402


def setUpModule():
    F.isolate_tmp()


def tearDownModule():
    F.restore_tmp()


class SvcConnect(unittest.TestCase):
    def test_svc_connect_does_not_freeze_reactor(self):
        srv = socket.socket()
        srv.bind(("127.0.0.1", 0))
        srv.listen(0)
        port = srv.getsockname()[1]
        fill = []
        try:
            for i in range(8):            # fill the accept queue
                c = socket.socket()
                c.setblocking(False)
                try:
                    c.connect(("127.0.0.1", port))
                except BlockingIOError:
                    pass
                fill.append(c)
            time.sleep(0.2)
            L = F.make_lidar(service="127.0.0.1:%d" % (port,))
            t0 = time.monotonic()
            try:
                L._svc({"cmd": "ping"}, timeout=30.0)
            except F.CmdError:
                pass
            blocked = time.monotonic() - t0
            L._svc_close()
            self.assertTrue(
                blocked < 0.5 or L.reactor.pauses > 0,
                "reactor frozen %.2f s in the service connect (%d reactor"
                " pauses)" % (blocked, L.reactor.pauses))
        finally:
            for c in fill:
                c.close()
            srv.close()


class DriverChannel(unittest.TestCase):
    def test_glued_two_replies_in_one_recv(self):
        L = F.make_lidar()
        s = F.FakeDriverSock(one_at_a_time=False)
        # a stale reply is still in the socket (aborted streaming command)
        s.sendall(b'{"control": "get_point_cloud", "prefetch": 1}\x03')
        L.sock = s
        try:
            r = L._call({"control": "get_sensor_temp"})
        except F.CmdError as e:
            self.fail("expected the get_sensor_temp reply, got: %s" % (e,))
        self.assertEqual(r["result"].get("temp"), 41.5, r)

    def test_timing_abort_keeps_channel_in_step(self):
        class Toolhead:
            def wait_moves(self):
                pass

            def get_last_move_time(self):
                return 0.0

        class Mcu:
            def estimated_print_time(self, t):
                return t

        class MR:
            trapqs = {"toolhead": object()}

        gcode = F.Gcode(fail_on="G1 ")       # a move error during the ramps
        L = F.make_lidar({"toolhead": Toolhead(), "mcu": Mcu(),
                          "motion_report": MR()}, gcode=gcode)
        L.sock = F.FakeDriverSock(one_at_a_time=True)
        try:
            L.cmd_LIDAR_TIMING(F.Gcmd(LEVELS="0"))
        except F.CmdError:
            pass
        # the next ordinary request must get its own reply
        r = L._call({"control": "get_sensor_temp"})
        self.assertEqual(r.get("control"), "get_sensor_temp",
                         "reply belongs to %r (channel off by one)"
                         % (r.get("control"),))


class ServiceFrameErrors(unittest.TestCase):
    def test_frame_error_does_not_desync_service(self):
        srv = lidar_service.Server(("127.0.0.1", 0), lidar_service.Handler)
        srv.svc = lidar_service.Service(None, tempfile.mkdtemp())
        srv.allow = set()
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            L = F.make_lidar(service="127.0.0.1:%d" % (srv.server_address[1],))
            self.assertTrue(L._svc({"cmd": "ping"})["ok"])
            head = '{"cmd": "frame", "name": %s, "row": %%d, "raw": ' % (
                json.dumps("scan"),)
            glued = (b'{"control": "get_point_cloud", "result": {"code": 0}}'
                     b'\x03{"control": "get_point_cloud", "result":'
                     b' {"code": 0}}')
            L._svc_send((head % (0,)).encode() + glued + b"}\n")
            replies = []
            for i in range(2):
                try:
                    replies.append(L._svc({"cmd": "ping", "seq": i}))
                except F.CmdError as e:
                    replies.append(str(e))
            self.assertTrue(
                all(isinstance(r, dict) and r.get("ok") for r in replies),
                "pings after a bad frame got: %r" % (replies,))
        finally:
            L._svc_close()
            srv.shutdown()
            srv.server_close()


class StreamModePrefetch(unittest.TestCase):
    def test_mode1_frames_are_streamed(self):
        class SV:
            allVariables = {"lidar_stream_mode": 1}

        L = F.make_lidar({"save_variables": SV()})
        msg = L._frame_msg()
        self.assertEqual(msg.get("prefetch"), 1,
                         "driver %r -> %r (no prefetch)" % (L._driver(), msg))


if __name__ == "__main__":
    unittest.main(verbosity=2)
