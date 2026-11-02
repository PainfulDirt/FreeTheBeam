"""helper/lidar_service.py talking to the driver: over the bridge (TCP) or,
on the same host, straight to the driver's unix socket."""
import os
import socket
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "..",
                                                        "klipper")]
import lidar_service  # noqa: E402


def serve_once(path):
    """A fake driver: answers every request with code 0."""
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(path)
    srv.listen(1)

    def run():
        c, _ = srv.accept()
        buf = b""
        while True:
            d = c.recv(4096)
            if not d:
                break
            buf += d
            while b"\x03" in buf:
                _, buf = buf.split(b"\x03", 1)
                c.sendall(b'{"result": {"code": 0}}\x03')
        c.close()
        srv.close()
    threading.Thread(target=run, daemon=True).start()


class TestServiceSocket(unittest.TestCase):
    def test_middleware_unix_socket(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "ai_server_uds")
            serve_once(path)
            mw = lidar_service.Middleware(path)
            r = mw.call({"control": "get_laser_exposure"}, timeout=5.0)
            self.assertEqual(r["result"]["code"], 0)
            mw.close()

    def test_args_exclusive(self):
        ap = lidar_service.build_parser()
        for argv in ([], ["--printer", "h:7130", "--socket", "/tmp/x"]):
            with self.assertRaises(SystemExit):
                ap.parse_args(argv)
        a = ap.parse_args(["--socket", "/tmp/ai_server_uds"])
        self.assertEqual((a.socket, a.printer), ("/tmp/ai_server_uds", None))


if __name__ == "__main__":
    unittest.main()
