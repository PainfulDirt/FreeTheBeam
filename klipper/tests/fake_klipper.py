"""A fake Klipper around klipper/lidar.py for offline tests: a homed
toolhead with given travel limits, bed mesh, save_variables, optional load
cell probe / probe, and a G-code runner that logs every line."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import lidar  # noqa: E402


class Err(Exception):
    pass


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Gcmd:
    def __init__(self, **p):
        self.p = dict((k.upper(), str(v)) for k, v in p.items())
        self.info = []

    def get(self, k, default=None):
        return self.p.get(k, default)

    def get_float(self, k, default=None, **kw):
        return float(self.p[k]) if k in self.p else default

    def get_int(self, k, default=None, **kw):
        return int(self.p[k]) if k in self.p else default

    def respond_info(self, m):
        self.info.append(m)

    def error(self, m):
        return Err(m)


def make_lidar(axis_min=(-2.0, -2.0), axis_max=(306.5, 306.0),
               offset=(-36.0, -21.2), zero_ref=None, load_cell=False,
               probe_offsets=None, on_gcode=None, **attrs):
    """Returns (lidar object, log of G-code lines, objects by name)."""
    log = []
    looked_up = []
    saved = {}
    t = [0.0]
    lc = Obj(last_z_result=None)
    probe_status = {"last_z_result": None}
    objs = {
        "toolhead": Obj(get_status=lambda e: {
            "homed_axes": "xyz",
            "axis_minimum": [axis_min[0], axis_min[1], -10.0, 0.0],
            "axis_maximum": [axis_max[0], axis_max[1], 250.0, 0.0]}),
        "save_variables": Obj(allVariables=saved),
        "bed_mesh": Obj(get_mesh=lambda: None, set_mesh=lambda m: None,
                        bmc=Obj(zero_ref_pos=zero_ref)),
        "print_stats": Obj(get_status=lambda e: {"state": "standby"}),
        "gcode_move": Obj(get_status=lambda e: {"homing_origin": [0, 0, 0]}),
        "heater_bed": Obj(get_status=lambda e: {"temperature": 60.0,
                                                 "target": 0.0}),
    }
    if load_cell:
        objs["load_cell_probe"] = Obj(get_printer_probe=lambda: lc)
    if probe_offsets is not None:
        objs["probe"] = Obj(
            get_offsets=lambda: (tuple(probe_offsets) + (1.0,))[:3],
            get_status=lambda e: dict(probe_status))

    def run(script):
        for line in script.split("\n"):
            w = line.split()
            if not w:
                continue
            log.append(line)
            t[0] += 1.0
            if w[0] == "LOAD_CELL_PROBE":
                lc.last_z_result = 0.0
            elif w[0] == "PROBE":
                probe_status["last_z_result"] = 0.0
            if on_gcode is not None:
                on_gcode(line, objs)

    def lookup(name, default=None):
        looked_up.append(name)
        return objs.get(name, default)

    L = object.__new__(lidar.Lidar)
    L.printer = Obj(lookup_object=lookup, command_error=Err)
    L.reactor = Obj(monotonic=lambda: t[0], pause=lambda w: w)
    L.gcode = Obj(run_script_from_command=run)
    L.service, L.scan_z = "helper:7131", 3.0
    L.x_offset, L.y_offset, L.offset_z = offset[0], offset[1], 3.0
    L.x_per_z = L.y_per_z = 0.0
    L.twist_reading = lambda x, y, z=None: 0.0
    for k, v in attrs.items():
        setattr(L, k, v)
    objs["_looked_up"] = looked_up
    objs["_probe_status"] = probe_status
    return L, log, objs


K1 = dict(axis_min=(-2.0, -2.0), axis_max=(306.5, 306.0),
          offset=(-36.0, -21.2), mesh_min=(5.0, 5.0), mesh_max=(270.0, 265.0))
SMALL = dict(axis_min=(0.0, 0.0), axis_max=(230.0, 225.0),
             offset=(-36.0, -21.2))
