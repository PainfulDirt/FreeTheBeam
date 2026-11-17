"""A small simulated printer around klipper/lidar.py for the first-layer
check and the continuous-mesh rescan tests: toolhead moves as trapezoids
with a trapq that only answers from its finalized history, LiDAR frames
every PERIOD s that see an analytic bed over their exposure window, the
helper's Service in-process, and a minimal bed_mesh (bilinear ZMesh).
Needs numpy (callers skip without it)."""
import ast
import json
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(HERE, "..", "..", "helper"), os.path.join(HERE, "..")):
    p = os.path.abspath(p)
    if p not in sys.path:
        sys.path.insert(0, p)
import lidar             # noqa: E402
import lidar_service     # noqa: E402

ACCEL = 20000.0
PERIOD = 0.199
TRUE_LAG = 0.015          # the frame really looks 15 ms later than measured
FLUSH_AHEAD = 0.25        # a trapq move is finalized this close to now
X_OFFSET, Y_OFFSET = -36.0, -21.2
AXIS_MIN, AXIS_MAX = (-2.0, -2.0, -10.0), (306.5, 306.0, 250.0)
ZERO_REF = (150.0, 150.0)


class Err(Exception):
    pass


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Gcmd:
    def __init__(self, **p):
        self.p = dict((k.upper(), str(v)) for k, v in p.items())
        self.msgs = []

    def get(self, k, default=None):
        return self.p.get(k, default)

    def get_float(self, k, default=None, **kw):
        return float(self.p[k]) if k in self.p else default

    def get_int(self, k, default=None, **kw):
        return int(self.p[k]) if k in self.p else default

    def respond_info(self, m):
        self.msgs.append(m)

    def error(self, m):
        return Err(m)


# --- a minimal bed_mesh: lidar.py takes ZMesh / BedMeshError from the module
# of the bed_mesh object (sys.modules[type(bm).__module__]), i.e. this one
class BedMeshError(Exception):
    pass


class ZMesh:
    """Bilinear over the probed matrix (Klipper's is bicubic)."""

    def __init__(self, params, name):
        self.params, self.name = dict(params), name
        self.matrix, self.offset = None, 0.0

    def build_mesh(self, matrix):
        p = self.params
        if len(matrix) != p["y_count"] or any(
                len(r) != p["x_count"] for r in matrix):
            raise BedMeshError("mesh: matrix size")
        self.matrix = [[float(v) for v in r] for r in matrix]

    def get_mesh_params(self):
        return dict(self.params)

    def get_probed_matrix(self):
        return [list(r) for r in self.matrix]

    def set_zero_reference(self, x, y):
        self.offset = 0.0
        self.offset = self.calc_z(x, y)

    def calc_z(self, x, y):
        p, m = self.params, self.matrix
        nx, ny = p["x_count"], p["y_count"]
        fx = (x - p["min_x"]) / (p["max_x"] - p["min_x"]) * (nx - 1)
        fy = (y - p["min_y"]) / (p["max_y"] - p["min_y"]) * (ny - 1)
        fx, fy = min(max(fx, 0.0), nx - 1.001), min(max(fy, 0.0), ny - 1.001)
        i, j = int(fx), int(fy)
        tx, ty = fx - i, fy - j
        a = m[j][i] * (1 - tx) + m[j][i + 1] * tx
        b = m[j + 1][i] * (1 - tx) + m[j + 1][i + 1] * tx
        return a * (1 - ty) + b * ty - self.offset


class BedMesh:
    def __init__(self):
        self.z_mesh = None
        self.bmc = Obj(orig_config={"mesh_x_pps": 2, "mesh_y_pps": 2,
                                    "algo": "bicubic", "tension": 0.1},
                       zero_ref_pos=list(ZERO_REF))
        self.profiles = {}
        self.pmgr = Obj(get_profiles=lambda: self.profiles)

    def set_mesh(self, m):
        self.z_mesh = m

    def get_mesh(self):
        return self.z_mesh

    def save_profile(self, name):
        z = self.z_mesh
        self.profiles[name] = {"points": z.get_probed_matrix(),
                               "mesh_params": z.get_mesh_params()}


def bed(x, y):
    """The real surface (mm): a smooth warp of about +-0.1 mm."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    return (0.06 * np.sin(2 * np.pi * x / 240.0)
            + 0.04 * np.cos(2 * np.pi * y / 200.0)
            + 0.0003 * (x - 150.0) - 0.0002 * (y - 150.0))


class Sim:
    def __init__(self, surface):
        self.bed = surface
        self.t = 100.0
        self.end = 0.0
        self.pos = [150.0, 150.0, 10.0]
        self.segs = []          # (t0, t1, p0, p1, vmax, ta)
        self.subs = []          # trapq moves: (t0, dur, start, unit, v0, a)
        self.last_arrival = None
        self.rng = random.Random(3)

    # --- toolhead
    def last_move_time(self):
        if self.end < self.t + 0.05:
            self.end = self.t + 0.25
        return self.end

    def move(self, target, f):
        p0, p1 = list(self.pos), list(target)
        d = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
        if d < 1e-9:
            return
        v = f / 60.0
        if d < v * v / ACCEL:
            v = math.sqrt(d * ACCEL)
        ta = v / ACCEL
        dur = d / v + ta
        t0 = self.last_move_time()
        self.segs.append((t0, t0 + dur, p0, p1, v, ta))
        unit = [(b - a) / d for a, b in zip(p0, p1)]
        da = 0.5 * v * ta
        mid = [a + u * da for a, u in zip(p0, unit)]
        cruise = d - 2 * da
        dec = [a + u * (da + cruise) for a, u in zip(p0, unit)]
        self.subs += [(t0, ta, p0, unit, 0.0, ACCEL),
                      (t0 + ta, cruise / v, mid, unit, v, 0.0),
                      (t0 + ta + cruise / v, ta, dec, unit, v, -ACCEL)]
        self.end = t0 + dur
        self.pos = p1

    def dwell(self, s):
        self.end = self.last_move_time() + s

    def history_end(self):
        ends = [m[0] + m[1] for m in self.subs
                if m[0] + m[1] <= self.t + FLUSH_AHEAD]
        return max(ends) if ends else None

    def at_hist(self, t):
        """get_trapq_position: the newest finalized move starting before t,
        clamped to its end."""
        best = None
        for m in self.subs:
            if m[0] + m[1] <= self.t + FLUSH_AHEAD and m[0] < t:
                best = m
        if best is None:
            return None, None
        t0, dur, st, unit, v0, a = best
        u = max(0.0, min(dur, t - t0))
        dist = (v0 + 0.5 * a * u) * u
        return [s0 + e * dist for s0, e in zip(st, unit)], v0 + a * u

    def at(self, t):
        """(position, speed) at print time t."""
        last = None
        for s in self.segs:
            if s[0] <= t <= s[1]:
                t0, t1, p0, p1, v, ta = s
                d = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
                u, dur = t - t0, t1 - t0
                if u < ta:
                    dist, vel = 0.5 * ACCEL * u * u, ACCEL * u
                elif u > dur - ta:
                    r = dur - u
                    dist, vel = d - 0.5 * ACCEL * r * r, ACCEL * r
                else:
                    dist, vel = 0.5 * v * ta + v * (u - ta), v
                f = dist / d
                return [a + (b - a) * f for a, b in zip(p0, p1)], vel
            if s[1] < t:
                last = s
        if last is None:
            return (self.segs[0][2] if self.segs else self.pos), 0.0
        return list(last[3]), 0.0

    # --- LiDAR
    def frame(self, sent):
        """The next frame after a request at `sent` (frames back-to-back
        every PERIOD): raw reply bytes and the arrival time."""
        a = sent + 0.15
        if self.last_arrival is not None:
            a = max(a, self.last_arrival + PERIOD)
        self.last_arrival = a
        ts = np.linspace(a - 0.1968 + TRUE_LAG, a - 0.1633 + TRUE_LAG, 5)
        px = np.arange(-14.0, 14.01, 0.04)
        h = np.zeros_like(px)
        for t in ts:
            (nx, ny, nz), v = self.at(t)
            lx, ly = nx + X_OFFSET, ny + Y_OFFSET
            h += self.bed(lx - px, ly) - (nz - 3.0)     # x_dir -1
        h /= len(ts)
        dist = 25.6 - h - 0.041 * px + np.array(
            [self.rng.gauss(0, 0.004) for _ in px])
        pc = [[int(x * 1000), 0, int(d * 1000)] for x, d in zip(px, dist)]
        return (json.dumps({"t": a, "result": {"point_cloud": pc}})
                .encode()), a


def build(data_dir):
    """A homed printer with the LiDAR, helper service in-process (its files
    in data_dir). Returns Obj(sim, svc, L, bm, saved, log)."""
    sim = Sim(bed)
    full = [[i * 19 - 14440, 0, 25600] for i in range(1520)]
    svc = lidar_service.Service(Obj(call=lambda m, timeout=15.0: {
        "result": {"point_cloud": full}} if m.get("control") ==
        "get_point_cloud" else {"result": {}}), data_dir)
    saved = {"lidar_driver": "direct",
             "lidar_timing": {"direct": {"0": [0.1992, 0.1784, 0.1633,
                                               0.1968]}}}
    log = []
    bm = BedMesh()
    heaters = {"extruder": [150.0, 150.0], "heater_bed": [70.0, 70.0]}

    def heater(n):
        return Obj(get_status=lambda t: {"temperature": heaters[n][0],
                                         "target": heaters[n][1]})
    toolhead = Obj(
        get_status=lambda t: {"homed_axes": "xyz",
                              "axis_minimum": list(AXIS_MIN) + [0.0],
                              "axis_maximum": list(AXIS_MAX) + [0.0]},
        get_extruder=lambda: Obj(get_heater=lambda: heater("extruder")),
        get_last_move_time=sim.last_move_time,
        wait_moves=lambda: setattr(sim, "t", max(sim.t, sim.end)),
        get_position=lambda: list(sim.pos) + [0.0])
    objs = {
        "toolhead": toolhead, "bed_mesh": bm,
        "save_variables": Obj(allVariables=saved),
        "heater_bed": heater("heater_bed"),
        "print_stats": Obj(get_status=lambda t: {"state": "standby",
                                                 "filename": ""}),
        "gcode_move": Obj(get_status=lambda t: {"homing_origin": [0, 0, 0]}),
        "mcu": Obj(estimated_print_time=lambda t: t),
        "motion_report": Obj(trapqs={"toolhead": Obj(
            get_trapq_position=sim.at_hist, history_end=sim.history_end)}),
        "exclude_object": Obj(get_status=lambda t: {"objects": [
            {"name": "a", "polygon": [[120, 120], [180, 120], [180, 170],
                                      [120, 170]]}]}),
    }

    def pause(t):
        sim.t = max(sim.t, t)
        return sim.t
    reactor = Obj(monotonic=lambda: sim.t, pause=pause)
    printer = Obj(lookup_object=lambda n, d=None: objs.get(n, d),
                  command_error=Err, get_reactor=lambda: reactor)

    def kv(words):
        return dict((w.split("=", 1)[0].upper(), w.split("=", 1)[1])
                    for w in words if "=" in w)

    def run(script):
        for line in script.split("\n"):
            w = line.split()
            if not w:
                continue
            c = w[0].upper()
            log.append(line)
            if c in ("G0", "G1"):
                d = dict((a[0].upper(), float(a[1:])) for a in w[1:])
                tgt = [d.get("X", sim.pos[0]), d.get("Y", sim.pos[1]),
                       d.get("Z", sim.pos[2])]
                for i in range(3):
                    if not AXIS_MIN[i] <= tgt[i] <= AXIS_MAX[i]:
                        raise Err("Move out of range: %r" % (tgt,))
                sim.move(tgt, d.get("F", 6000.0))
            elif c == "G4":
                sim.dwell(float(w[1][1:]) / 1000.0)
            elif c == "SAVE_VARIABLE":
                p = dict(a.split("=", 1) for a in w[1:])
                saved[p["VARIABLE"]] = ast.literal_eval(
                    p["VALUE"].strip('"'))
            elif c == "SET_HEATER_TEMPERATURE":
                heaters["heater_bed"][1] = float(kv(w[1:])["TARGET"])
            elif c in ("M104", "M109"):
                heaters["extruder"][1] = float(w[1][1:])
            elif c in ("G90", "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE",
                       "M140", "M190", "SET_VELOCITY_LIMIT"):
                pass
            else:
                raise Err("sim: unknown command %s" % (line,))

    L = object.__new__(lidar.Lidar)
    L.printer, L.reactor = printer, reactor
    L.gcode = Obj(run_script_from_command=run)
    L.service, L.scan_z, L.x_dir, L.speed = "HELPER_IP:7131", 3.0, -1.0, 50.0
    L.x_offset, L.y_offset, L.offset_z = X_OFFSET, Y_OFFSET, 3.0
    L.x_per_z = L.y_per_z = 0.0
    L.mesh_min, L.mesh_max = (5.0, 5.0), (270.0, 265.0)
    L.scans, L.last_result, L.mesh_info = {}, {}, {}
    L.moving_lag, L.scan_level = 0.0, 0
    L.dwell_settle, L.dwell_margin = 0.03, 0.01
    L.frame_period, L.frame_latency, L.frame_spread = 0.235, 0.194, 0.05
    L.sock, L.fl, L.drv_buf = None, None, b""
    L.data_dir = data_dir
    queue = []

    def send(msg):
        queue.append(sim.t)

    def recv(timeout=10.0):
        raw, a = sim.frame(queue.pop(0))
        sim.t = max(sim.t, a)
        return raw, sim.t
    L._send, L._recv = send, recv

    def svc_call(req, timeout=30.0):
        rep = svc.handle(json.loads(json.dumps(req)))
        if not rep.get("ok"):
            raise Err("LiDAR service: %s" % (rep.get("error"),))
        return rep

    def svc_send(data, timeout=10.0):
        for line in data.decode().split("\n"):
            if line.strip():
                svc.handle(json.loads(line))
    L._svc, L._svc_send = svc_call, svc_send
    objs["lidar"] = L
    return Obj(sim=sim, svc=svc, L=L, bm=bm, saved=saved, log=log, bed=bed)


def truth_vs(env, prof, n=15, area=(20, 265, 25, 265)):
    """rms and max (um) of a profile against the real surface, constant
    removed."""
    z = env.L._zmesh(env.bm, prof)
    p = z.get_mesh_params()
    x0, x1 = max(area[0], p["min_x"]), min(area[1], p["max_x"])
    y0, y1 = max(area[2], p["min_y"]), min(area[3], p["max_y"])
    d = []
    for x in np.linspace(x0, x1, n):
        for y in np.linspace(y0, y1, n):
            d.append(z.calc_z(x, y) - float(env.bed(x, y)))
    d = np.array(d) - np.mean(d)
    return np.sqrt((d ** 2).mean()) * 1e3, np.abs(d).max() * 1e3


# --- the first-layer check on the simulator
FL_OBJECTS = [
    {"name": "plate_A", "polygon": [[120, 120], [180, 120], [180, 170],
                                    [120, 170], [120, 120]]},
    {"name": "small_B", "polygon": [[200, 60], [215, 60], [215, 75],
                                    [200, 75], [200, 60]]}]
LAYER = 0.23          # what really came out (sliced 0.20)
TILT_X = 0.0002       # mm/mm: +20 um per 100 mm across X


def texture(x, y, cell=0.25, amp=0.025):
    """Repeatable PEI-like texture: value noise on a 0.25 mm grid."""
    x = np.asarray(x, float) / cell
    y = np.asarray(y, float) / cell
    i, j = np.floor(x), np.floor(y)
    tx, ty = x - i, y - j

    def h(a, b):
        v = np.sin(a * 12.9898 + b * 78.233) * 43758.5453
        return (v - np.floor(v)) * 2.0 - 1.0
    a = h(i, j) * (1 - tx) + h(i + 1, j) * tx
    b = h(i, j + 1) * (1 - tx) + h(i + 1, j + 1) * tx
    return amp * (a * (1 - ty) + b * ty)


def _inside(x, y, poly):
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return (x >= min(xs)) & (x <= max(xs)) & (y >= min(ys)) & (y <= max(ys))


class World:
    """The bed with texture; once `printed`, the first layer on the objects
    (LAYER thick, tilted in X) and a small plate drift."""

    def __init__(self, base, objects):
        self.base, self.objects = base, objects
        self.printed = False

    def __call__(self, x, y):
        x = np.asarray(x, float)
        z = self.base(x, y) + texture(x, y)
        if self.printed:
            z = z + 0.006 + 0.00003 * (x - 150.0) - 0.00002 * (y - 150.0)
            t = LAYER + TILT_X * (x - 150.0) + 0.008 * np.sin(
                2 * np.pi * x / 0.45)
            m = np.zeros_like(x, dtype=bool)
            for o in self.objects:
                m = m | _inside(x, y, o["polygon"])
            z = z + np.where(m, t, 0.0)
        return z


def fl_setup(data_dir, learn=0, objects=FL_OBJECTS, state="printing"):
    """The simulator inside a print, for FIRST_LAYER_SCAN. Returns (env,
    world, print_stats dict)."""
    env = build(data_dir)
    world = World(env.sim.bed, objects)
    env.sim.bed = world
    L = env.L
    gm = {"homing_origin": [0.0, 0.0, 0.0],
          "gcode_position": [150.0, 145.0, 0.4, 0.0]}
    st = {"state": state, "filename": "cube.gcode"}
    extra = {
        "exclude_object": Obj(get_status=lambda t: {"objects": objects}),
        "gcode_move": Obj(get_status=lambda t: gm),
        "print_stats": Obj(get_status=lambda t: st),
        "output_pin First_Layer_Learn": Obj(
            get_status=lambda t: {"value": learn}),
    }
    orig = L.printer.lookup_object
    L.printer.lookup_object = lambda n, d=None: extra[n] if n in extra \
        else orig(n, d)
    run0 = L.gcode.run_script_from_command

    def run(script):
        # extruder-only commands are logged; prime moves lose their E
        keep = []
        for line in script.split("\n"):
            c = line.split()[0].upper() if line.split() else ""
            if c in ("M83", "M106") or c == "TEMPERATURE_WAIT":
                env.log.append(line)
                continue
            if c == "G1" and "E" in line and "X" not in line and \
                    "Y" not in line and "Z" not in line:
                env.log.append(line)
                continue
            if c == "G1" and " E" in line:
                env.log.append(line)
                line = " ".join(w for w in line.split()
                                if not w.upper().startswith("E"))
            keep.append(line)
        if keep:
            run0("\n".join(keep))
    L.gcode.run_script_from_command = run
    return env, world, st
