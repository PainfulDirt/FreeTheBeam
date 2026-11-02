# Keep the toolhead out of zones where it would hit something
#
# The Microprobe sits 20.3mm behind the nozzle; in the middle at the back it
# hits the Z rod housing. Every G-code move is checked: a move that would end
# in, or pass through, a zone is refused with an error before anything moves.
#
#   [keep_out]
#   zone1: x_min, y_min, x_max, y_max    (nozzle coordinates)
#
# Copy or symlink into klippy/plugins (see docs/install-k1max.md).
# Part of FreeTheBeam, https://github.com/PainfulDirt/FreeTheBeam
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class KeepOut:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.zones = []
        for i in range(1, 10):
            zone = config.getfloatlist("zone%d" % (i,), None, count=4)
            if zone is not None:
                x0, y0, x1, y1 = zone
                self.zones.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
        self.next_transform = None
        self.last = None
        self.printer.register_event_handler("klippy:connect", self._connect)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "KEEP_OUT_STATUS", self.cmd_KEEP_OUT_STATUS,
            desc="Show the zones the toolhead is kept out of")

    def _connect(self):
        gcode_move = self.printer.lookup_object("gcode_move")
        self.next_transform = gcode_move.set_move_transform(self, force=True)
        gcode_move.reset_last_position()

    def get_position(self):
        self.last = self.next_transform.get_position()
        return list(self.last)

    def _crosses(self, p0, p1, zone):
        # Liang-Barsky clip of the XY segment against the open rectangle
        x0, y0, x1, y1 = zone
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        t0, t1 = 0.0, 1.0
        for p, q in ((-dx, p0[0] - x0), (dx, x1 - p0[0]),
                     (-dy, p0[1] - y0), (dy, y1 - p0[1])):
            if p == 0.0:
                if q <= 0.0:
                    return False
                continue
            t = q / p
            if p < 0.0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
            if t0 >= t1:
                return False
        return True

    def _blocked(self, p0, p1):
        for zone in self.zones:
            if self._crosses(p0, p1, zone):
                return zone
        return None

    def _detour(self, start, newpos):
        safe_y = min(z[1] for z in self.zones) - 1.0
        z = max(start[2], newpos[2])
        pts = [[start[0], min(start[1], safe_y), z] + list(start[3:]),
               [newpos[0], min(newpos[1], safe_y), z] + list(start[3:])]
        path = [list(start)] + pts + [list(newpos)]
        for a, b in zip(path, path[1:]):
            if self._blocked(a, b):
                return None
        return pts

    def move(self, newpos, speed):
        start = self.last if self.last is not None else newpos
        zone = self._blocked(start, newpos)
        if zone is not None:
            travel = all(abs(a - b) < 1e-9 for a, b in zip(start[3:], newpos[3:]))
            pts = self._detour(start, newpos) if travel else None
            if pts is not None:
                for p in pts:
                    if any(abs(a - b) > 1e-9 for a, b in zip(p, self.last or start)):
                        self.next_transform.move(p, speed)
                        self.last = list(p)
                self.next_transform.move(newpos, speed)
                self.last = list(newpos)
                return
            raise self.printer.command_error(
                "Keep-out: move to X%.1f Y%.1f would enter the zone"
                " X%.0f-%.0f Y%.0f-%.0f (Microprobe would hit the Z rod"
                " housing)" % (newpos[0], newpos[1], zone[0], zone[2],
                               zone[1], zone[3]))
        self.next_transform.move(newpos, speed)
        self.last = list(newpos)

    def cmd_KEEP_OUT_STATUS(self, gcmd):
        for z in self.zones:
            gcmd.respond_info("Keep-out zone: X%.1f-%.1f Y%.1f-%.1f" % (z[0], z[2], z[1], z[3]))


def load_config(config):
    return KeepOut(config)
