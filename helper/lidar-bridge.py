#!/usr/bin/env python3
# Forward TCP connections to the LiDAR driver's unix socket (lidar-direct.py,
# /tmp/ai_server_uds), so the k1-lidar service on a helper computer can talk
# to the LiDAR. Only copies bytes; started by lidar-start.sh.
#
# Single-threaded: every send has a timeout (SEND_TIMEOUT) and TCP keepalive
# drops a peer that vanished (helper off Wi-Fi), so a dead peer costs seconds,
# not the ~15 min until TCP gives up on its own.
#
#   lidar-bridge.py PORT ALLOWED_IP [ALLOWED_IP ...]
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import select
import socket
import sys

UDS = "/tmp/ai_server_uds"
SEND_TIMEOUT = 15.0     # s; an ir_photo reply (~150 KB) takes < 2 s on Wi-Fi
port = int(sys.argv[1])
allowed = set(sys.argv[2:])

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", port))
srv.listen(4)
peer = {}   # socket -> its partner


def close(s):
    p = peer.pop(s, None)
    for x in (s, p):
        if x is not None:
            peer.pop(x, None)
            try:
                x.close()
            except OSError:
                pass


while True:
    r, _, _ = select.select([srv] + list(peer), [], [])
    for s in r:
        if s is srv:
            c, addr = srv.accept()
            if addr[0] not in allowed:
                c.close()
                continue
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                c.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
            except (OSError, AttributeError):
                pass
            u = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                u.settimeout(SEND_TIMEOUT)
                u.connect(UDS)
            except OSError:
                u.close()
                c.close()
                continue
            # recv only runs after select() (never waits); sendall gives up
            # after SEND_TIMEOUT (socket.timeout is an OSError: both closed)
            c.settimeout(SEND_TIMEOUT)
            peer[c], peer[u] = u, c
            continue
        if s not in peer:
            continue
        try:
            d = s.recv(65536)
        except OSError:
            d = b""
        if not d:
            close(s)
            continue
        try:
            peer[s].sendall(d)
        except OSError:
            close(s)
