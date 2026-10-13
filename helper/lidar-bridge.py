#!/usr/bin/env python3
# Forward TCP connections to cx_ai_middleware's unix socket, so the k1-lidar
# service on a helper computer can talk to the LiDAR. Only copies bytes;
# started by lidar-start.sh.
#
#   lidar-bridge.py PORT ALLOWED_IP [ALLOWED_IP ...]
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import select
import socket
import sys

UDS = "/tmp/ai_server_uds"
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
            u = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                u.connect(UDS)
            except OSError:
                c.close()
                continue
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
