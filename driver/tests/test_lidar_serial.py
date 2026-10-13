"""Offline tests for lidar_serial.py against a fake module that follows the
firmware's handlers (decompiled firmware): auth with a random challenge,
only 0x55/0xA0/0xA1/0xAA before auth, stream on/off, point cloud replies.
Run: python3 test_lidar_serial.py"""

import hashlib
import os
import random
import struct
import sys
import threading
import tty
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import lidar_serial as ls  # noqa: E402


class FakeModule(threading.Thread):
    PRE_AUTH = {0x55, 0xA0, 0xA1, 0xAA}

    def __init__(self, fd, points):
        threading.Thread.__init__(self, daemon=True)
        self.fd, self.points = fd, points
        self.authorized, self.stream, self.key = False, False, None
        self.level, self.exp, self.gain = 0, 1000, 1000
        self.parser = ls.FrameParser()
        self.seen = []
        self.noise = True

    def reply(self, cmd, payload):
        frame = ls.build_frame(cmd, payload, src=2)
        if self.noise:                       # garbage + split writes, like a real UART
            os.write(self.fd, b'\x00\xAF\x12')
            for i in range(0, len(frame), 7):
                os.write(self.fd, frame[i:i + 7])
        else:
            os.write(self.fd, frame)

    def run(self):
        while True:
            try:
                data = os.read(self.fd, 4096)
            except OSError:
                return
            for src, cmd, p in self.parser.feed(data):
                self.seen.append(cmd)
                if not self.authorized and cmd not in self.PRE_AUTH:
                    continue                 # firmware: not in the pre-auth table
                self.handle(cmd, p)

    def handle(self, cmd, p):
        if cmd == 0x55:
            self.reply(cmd, b'\x00')
        elif cmd == 0xA0:
            value = random.randint(0, 2**31 - 1)
            rnd = bytes(random.choice(b'0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
                        for _ in range(4))
            text = ('%d' % value).encode() + hashlib.md5(rnd).hexdigest().encode()
            self.key = hashlib.sha256(text).digest()
            raw = struct.pack('<I', value) + rnd
            self.reply(cmd, b'\x00' + bytes(-b & 0xFF for b in raw))
        elif cmd == 0xA1:
            ok = len(p) == 32 and p == self.key
            self.authorized = self.authorized or ok
            self.reply(cmd, b'\x00' if ok else b'\x01')
        elif cmd == 0x03:
            self.reply(cmd, b'\x01\x00\x0d')
        elif cmd == 0x1D:
            self.reply(cmd, b'\x00' + bytes(range(1, 13)))
        elif cmd in (0x0E, 0x13, 0x14):
            self.reply(cmd, b'\x00')
        elif cmd == 0x01:
            self.stream = True
            self.reply(cmd, b'\x00')
        elif cmd == 0x02:
            self.stream = False
            self.reply(cmd, b'\x00')
        elif cmd == 0x04:
            if not self.stream:
                self.reply(cmd, b'\x04')     # "please turn on stream"
            else:
                pts = self.points[::1 << self.level]
                self.reply(cmd, b'\x00' + b''.join(struct.pack('<3h', *q) for q in pts))
        elif cmd == 0x08:
            self.level = p[0]
            self.reply(cmd, b'\x00')
        elif cmd == 0x09:
            self.reply(cmd, bytes((self.level,)))
        elif cmd == 0x15:
            self.exp = int.from_bytes(p[:3], 'little')
            self.reply(cmd, b'\x00')
        elif cmd == 0x16:
            self.reply(cmd, self.exp.to_bytes(3, 'little'))
        elif cmd == 0x17:
            self.gain = int.from_bytes(p[:3], 'little')
            self.reply(cmd, b'\x00')
        elif cmd == 0x18:
            self.reply(cmd, self.gain.to_bytes(3, 'little'))


class FrameTests(unittest.TestCase):
    def test_build_matches_middleware(self):
        # FUN_0040bd4c: AF FF, len = payload + 2, src 1, cmd, payload, -sum
        f = ls.build_frame(0x13, b'\x01')
        self.assertEqual(f[:6], bytes((0xAF, 0xFF, 3, 0, 1, 0x13)))
        self.assertEqual(sum(f[:-2]) + struct.unpack('<H', f[-2:])[0] & 0xFFFF, 0)

    def test_parser_garbage_split_badcsum(self):
        p = ls.FrameParser()
        good = ls.build_frame(0x04, b'\x00' + b'\x01\x02' * 30, src=2)
        bad = bytearray(ls.build_frame(0x03, b'\x01\x00\x0d', src=2))
        bad[-1] ^= 0xFF
        stream = b'\xAF\x00junk' + bytes(bad) + good + b'\xAF'
        out = []
        for i in range(0, len(stream), 5):
            out += p.feed(stream[i:i + 5])
        self.assertEqual(out, [(2, 0x04, good[6:-2])])
        self.assertEqual(p.buf, bytearray(b'\xAF'))

    def test_points(self):
        data = struct.pack('<6h', -14400, 0, 25500, 14800, -3, 25480)
        self.assertEqual(ls.parse_points(data + b'\x07'), [[-14400, 0, 25500], [14800, -3, 25480]])

    def test_auth_key_matches_middleware_formula(self):
        # middleware FUN_0040c8f0/FUN_0040c7fc on the 8 reply bytes, written out
        raw = struct.pack('<I', 123456) + b'aZ9q'
        neg = bytes(-b & 0xFF for b in raw)
        num = '%d' % struct.unpack('<i', bytes(-b & 0xFF for b in neg[:4]))[0]
        rnd = bytes(-b & 0xFF for b in neg[4:])
        want = hashlib.sha256((num + hashlib.md5(rnd).hexdigest()).encode()).digest()
        self.assertEqual(ls.auth_key(neg), want)

    def test_unsafe_refused(self):
        d = ls.LidarSerial()
        for cmd in (0x05, 0x10, 0x12, 0x1B, 0x56, 0xAA):
            with self.assertRaises(ls.LidarError):
                d.request(cmd)


class DriverTests(unittest.TestCase):
    def setUp(self):
        master, slave = os.openpty()
        tty.setraw(master)
        self.points = [[x * 20 - 14400, 0, 25500 + x % 7] for x in range(1500)]
        self.fake = FakeModule(master, self.points)
        self.fake.start()
        self.d = ls.LidarSerial(port=os.ttyname(slave), timeout=1.0)
        self.d.open()
        self.master, self.slave = master, slave

    def tearDown(self):
        self.d.close()
        os.close(self.slave)
        os.close(self.master)

    def test_full_session(self):
        info = self.d.init()
        self.assertEqual(info['version'], '1.0.13')
        self.assertEqual(info['sn'], bytes(range(1, 13)).hex().upper())
        with self.assertRaises(ls.LidarError) as e:
            self.d.get_point_cloud()             # stream off -> status 4
        self.assertEqual(e.exception.code, 4)
        self.d.open_flow()
        self.assertEqual(self.d.get_point_cloud(), self.points)
        self.d.set_point_cloud_level(1)
        self.assertEqual(self.d.get_point_cloud_level(), 1)
        self.assertEqual(len(self.d.get_point_cloud()), 750)
        self.d.set_exposure(1234)
        self.assertEqual(self.d.get_exposure(), 1234)
        self.d.close_flow()

    def test_without_auth_module_stays_silent(self):
        with self.assertRaises(ls.LidarError):
            self.d.init(authenticate=False)      # 0x03 not allowed before auth
        self.assertIn(0x03, self.fake.seen)

    def test_not_ready(self):
        with self.assertRaises(ls.LidarError) as e:
            self.d.get_point_cloud()
        self.assertEqual(e.exception.code, ls.NOT_READY)


if __name__ == '__main__':
    unittest.main(verbosity=2)
