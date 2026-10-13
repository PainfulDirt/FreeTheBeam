"""Open driver for the Creality K1 Max LiDAR module (talks to it directly over
its USB serial port instead of Creality's cx_ai_middleware).

Protocol from reverse engineering cx_ai_middleware and the module firmware
(notes: docs/protocol.md):

    AF FF | len u16 LE (= payload + 2) | src | cmd | payload | csum u16 LE
    src 01 = host, 02 = module; csum = -(sum of all bytes before it) & 0xffff
    reply payload = status u8 (0 = ok) + data, except the getters in
    VALUE_REPLIES, which reply with the bare value (firmware handlers)

Python 3.8 (the K1's), standard library only.
"""

import hashlib
import os
import select
import struct
import termios
import time

PORT = '/dev/serial/by-id/creality-laser'
BAUD = termios.B1500000

# command bytes (names from the firmware's dispatch table)
STREAM_ON = 0x01
STREAM_OFF = 0x02
GET_VERSION = 0x03
GET_POINT_CLOUD = 0x04
SET_PC_MODE, GET_PC_MODE = 0x06, 0x07
SET_PC_LEVEL, GET_PC_LEVEL = 0x08, 0x09
SET_ROI, GET_ROI = 0x0A, 0x0B
SET_STREAM_MODE, GET_STREAM_MODE = 0x0C, 0x0D
SET_COOR, GET_COOR = 0x0E, 0x0F
RESET = 0x11
LASER_SWITCH = 0x13
LED_SWITCH = 0x14
SET_EXP, GET_EXP = 0x15, 0x16
SET_GAIN, GET_GAIN = 0x17, 0x18
GET_IR_SENSOR_TEMP = 0x1C
GET_SN = 0x1D
GET_STATUS = 0x55
GET_AUTH_STRING = 0xA0
SET_KEY = 0xA1

# replies without a status byte (checked against every firmware handler)
VALUE_REPLIES = {GET_VERSION, GET_PC_MODE, GET_PC_LEVEL, GET_ROI, GET_STREAM_MODE,
                 GET_COOR, GET_EXP, GET_GAIN, GET_STATUS, 0x59}

# never sent unless allow_unsafe: sticker calibration (stores geometry in the
# module), OTA, factory restore, baud change (saved to the module's flash, the
# stock middleware then can't talk to it), IR image capture (hangs the module),
# authorize-flag backdoor
UNSAFE = {0x05: 'calibration', 0x10: 'start_ota', 0x12: 'restore',
          0x19: 'set_ota_file_info', 0x1A: 'trans_ota_file',
          0x1B: 'point_cloud_uart_cfg', 0x56: 'capture_ir',
          0xAA: 'set_authorize_flag'}

# socket-API compatible codes (cx_ai_middleware: 101 not ready, 102 failed)
NOT_READY, FAILED = 101, 102


class LidarError(Exception):
    def __init__(self, msg, code=FAILED):
        Exception.__init__(self, msg)
        self.code = code


def checksum(data):
    return -sum(data) & 0xFFFF


def build_frame(cmd, payload=b'', src=1):
    body = b'\xAF\xFF' + struct.pack('<H', len(payload) + 2) + bytes((src, cmd)) + bytes(payload)
    return body + struct.pack('<H', checksum(body))


class FrameParser:
    """Collects bytes and returns complete, checked frames (src, cmd, payload).
    Garbage before a header and frames with a bad checksum are skipped."""

    MAX_LEN = 0x4000

    def __init__(self):
        self.buf = bytearray()
        self.dropped = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(b'\xAF\xFF')
            if i < 0:
                keep = 1 if self.buf.endswith(b'\xAF') else 0
                self.dropped += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                return frames
            if i:
                self.dropped += i
                del self.buf[:i]
            if len(self.buf) < 4:
                return frames
            n, = struct.unpack_from('<H', self.buf, 2)
            if n < 2 or n > self.MAX_LEN:
                self.dropped += 2
                del self.buf[:2]
                continue
            total = 4 + n + 2
            if len(self.buf) < total:
                return frames
            frame = bytes(self.buf[:total])
            if struct.unpack_from('<H', frame, total - 2)[0] != checksum(frame[:-2]):
                self.dropped += 2
                del self.buf[:2]
                continue
            del self.buf[:total]
            frames.append((frame[4], frame[5], frame[6:-2]))


def auth_key(challenge):
    """Key for SET_KEY from the GET_AUTH_STRING reply data (8 bytes after the
    status byte). The module negates the bytes; value = int32 LE, random = 4
    alphanumeric chars; key = SHA256('%d' % value + md5hex(random))."""
    raw = bytes(-b & 0xFF for b in challenge[:8])
    value, = struct.unpack('<i', raw[:4])
    rnd = raw[4:8].split(b'\0')[0]
    text = ('%d' % value).encode() + hashlib.md5(rnd).hexdigest().encode()
    return hashlib.sha256(text).digest()


def parse_points(data):
    """Point-cloud data (after the status byte): N x (x, y, z) int16 LE, µm."""
    n = len(data) // 6
    v = struct.unpack('<%dh' % (n * 3), data[:n * 6])
    return [[v[i], v[i + 1], v[i + 2]] for i in range(0, len(v), 3)]


class LidarSerial:
    def __init__(self, port=PORT, timeout=2.0, allow_unsafe=False, log=None):
        self.port = port
        self.timeout = timeout
        self.allow_unsafe = allow_unsafe
        self.log = log
        self.fd = None
        self.parser = FrameParser()
        self.ready = False
        self.info = {}

    # -- port --------------------------------------------------------------
    def open(self):
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attr = termios.tcgetattr(fd)
            attr[0] = 0                                       # iflag: raw
            attr[1] = 0                                       # oflag
            attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL   # 8N1
            attr[3] = 0                                       # lflag: raw
            attr[4] = attr[5] = BAUD
            attr[6][termios.VMIN] = 0
            attr[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attr)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        self.fd = fd
        self.parser = FrameParser()

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None
        self.ready = False

    # -- requests ----------------------------------------------------------
    def request(self, cmd, payload=b'', timeout=None):
        """Send one command, return the reply data (after the status byte for
        commands that have one). Raises LidarError on timeout or bad status."""
        if cmd in UNSAFE and not self.allow_unsafe:
            raise LidarError('refusing unsafe command 0x%02x (%s)' % (cmd, UNSAFE[cmd]))
        if self.fd is None:
            raise LidarError('port not open', NOT_READY)
        os.write(self.fd, build_frame(cmd, payload))
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise LidarError('timeout waiting for reply to 0x%02x' % cmd)
            r, _, _ = select.select([self.fd], [], [], left)
            if not r:
                continue
            data = os.read(self.fd, 16384)
            if not data:
                raise LidarError('port closed')
            for src, rcmd, body in self.parser.feed(data):
                if src != 2 or rcmd != cmd:
                    self._log('ignored frame src %d cmd 0x%02x' % (src, rcmd))
                    continue
                if not body:
                    raise LidarError('empty reply to 0x%02x' % cmd)
                if cmd in VALUE_REPLIES:
                    return body
                if body[0] != 0:
                    raise LidarError('0x%02x status %d' % (cmd, body[0]), body[0])
                return body[1:]

    def _log(self, msg):
        if self.log:
            self.log(msg)

    # -- module setup (same order as cx_ai_middleware + cx_ai_crypto) -------
    def authenticate(self):
        challenge = self.request(GET_AUTH_STRING)
        self.request(SET_KEY, auth_key(challenge))

    def init(self, authenticate=True):
        """Status check, auth, version, serial number, coordinate mode 1."""
        self.ready = False
        status = self.request(GET_STATUS)
        if status[0] != 0:
            # error bits (middleware): module, IR sensor I2C, laser I2C, IR MIPI, algorithm
            raise LidarError('module status 0x%02x' % status[0])
        if authenticate:
            self.authenticate()
        v = self.request(GET_VERSION)
        self.info['version'] = '%d.%d.%d' % tuple(v[:3])
        self.info['sn'] = self.request(GET_SN)[:12].hex().upper()
        self.request(SET_COOR, b'\x01')
        self.ready = True
        return self.info

    # -- socket-API equivalents ----------------------------------------------
    def open_flow(self):
        self.request(STREAM_ON)

    def close_flow(self):
        self.request(STREAM_OFF)

    def get_point_cloud(self):
        """One fresh frame (the module waits for the next capture, <= 1 s)."""
        if not self.ready:
            raise LidarError('module not initialised', NOT_READY)
        return parse_points(self.request(GET_POINT_CLOUD, timeout=max(self.timeout, 1.5)))

    def set_laser(self, on):
        self.request(LASER_SWITCH, b'\x01' if on else b'\x00')

    def set_fill_light(self, on):
        self.request(LED_SWITCH, b'\x01' if on else b'\x00')

    def set_point_cloud_level(self, level):
        self.request(SET_PC_LEVEL, bytes((level,)))

    def get_point_cloud_level(self):
        return self.request(GET_PC_LEVEL)[0]

    def set_exposure(self, value):
        self.request(SET_EXP, struct.pack('<I', value)[:3])

    def get_exposure(self):
        return int.from_bytes(self.request(GET_EXP)[:3], 'little')

    def set_gain(self, value):
        self.request(SET_GAIN, struct.pack('<I', value)[:3])

    def get_gain(self):
        return int.from_bytes(self.request(GET_GAIN)[:3], 'little')

    def set_roi(self, x_min, x_max):
        if not 0 <= x_min <= x_max <= 1600:
            raise LidarError('roi out of range', 103)
        self.request(SET_ROI, struct.pack('<HH', x_min, x_max))

    def get_roi(self):
        return struct.unpack('<HH', self.request(GET_ROI)[:4])

    def get_sensor_temp(self):
        """Raw 4 bytes (format not decoded yet)."""
        return self.request(GET_IR_SENSOR_TEMP)
