# Axis twist calibration with the load cells instead of the paper test
#
# At spots along X, the Microprobe measures the bed and the nozzle touches the
# same spot with the load cells. The difference between the two varies along
# X by the carriage twist; that variation becomes [axis_twist_compensation].
# The compensation is for the Microprobe only: it is kept off the load cell
# touches (see _connect).
#
# Copy or symlink into klippy/plugins (see docs/install-k1max.md).
# Part of FreeTheBeam, https://github.com/PainfulDirt/FreeTheBeam
#
# This file may be distributed under the terms of the GNU GPLv3 license.


def _solve(a, b):
    """Solve a x = b (small dense system, Gauss-Jordan with pivoting)."""
    n = len(b)
    m = [list(row) + [v] for row, v in zip(a, b)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("singular")
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    return [m[i][n] / m[i][i] for i in range(n)]


def fit_twist(meas, n_spots, n_passes):
    """meas: (spot, pass, time, probe - nozzle) per measurement.

    The run starts right after heating, on a bed that may still be settling,
    so probe - nozzle drifts while it runs. Fit it as spot value + an offset
    per pass + one linear drift in time (the passes alternate direction, which
    keeps the drift apart from the spots). Returns (spot values, pass offsets,
    drift per second, residuals per spot)."""
    t0 = meas[0][2]
    nu = n_spots + (n_passes - 1) + (1 if n_passes > 1 else 0)
    rows, vals = [], []
    for spot, p, t, d in meas:
        row = [0.0] * nu
        row[spot] = 1.0
        if n_passes > 1:
            if p:
                row[n_spots + p - 1] = 1.0
            row[-1] = t - t0
        rows.append(row)
        vals.append(d)
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(nu)] for i in range(nu)]
    atb = [sum(r[i] * v for r, v in zip(rows, vals)) for i in range(nu)]
    x = _solve(ata, atb)
    offsets = [0.0] + x[n_spots:n_spots + n_passes - 1]
    drift = x[-1] if n_passes > 1 else 0.0
    res = [[] for _ in range(n_spots)]
    for (spot, p, t, d), row in zip(meas, rows):
        res[spot].append(d - sum(a * b for a, b in zip(row, x)))
    return x[:n_spots], offsets, drift, res


class TwistTouch:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.passes = config.getint("passes", 2, minval=1)
        self.samples = config.getint("samples", 3, minval=1)
        self.points = config.getint("points", 7, minval=3)
        # a spot whose passes disagree by more than this makes the run invalid
        # (after taking out the drift, see fit_twist)
        self.max_pass_diff = config.getfloat("max_pass_diff", 0.04, above=0.0)
        # whether the last run was applied (_CALIB_TWIST_DONE reads it)
        self.applied = False
        # never apply a result this far from the current values
        self.max_change = config.getfloat("max_change", 0.1, above=0.0)
        self.gcode = self.printer.lookup_object("gcode")
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.gcode.register_command(
            "AXIS_TWIST_TOUCH_CALIBRATE",
            self.cmd_AXIS_TWIST_TOUCH_CALIBRATE,
            desc=self.cmd_AXIS_TWIST_TOUCH_CALIBRATE_help,
        )

    def _connect(self):
        # Pellcorp's probe.py adds the axis twist compensation to every probe
        # result, the load cell touch included. The compensation corrects the
        # Microprobe against the nozzle; on the nozzle's own touch it would
        # shift True Zero by the compensation at the touch X. Take it out.
        atc = self.printer.lookup_object("axis_twist_compensation", None)
        lcp = self.printer.lookup_object("load_cell_probe", None)
        if atc is None or lcp is None:
            return
        pp = lcp.get_printer_probe()
        toolhead = self.printer.lookup_object("toolhead")
        orig = pp._probe

        def _probe(speed, gcmd):
            # same position probe.py looks the compensation up with
            pos = toolhead.get_position()
            epos, is_good = orig(speed, gcmd)
            epos = list(epos)
            epos[2] -= atc.get_z_compensation_value(pos)
            return epos, is_good

        pp._probe = _probe

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def get_status(self, eventtime):
        return {"applied": self.applied}

    cmd_AXIS_TWIST_TOUCH_CALIBRATE_help = (
        "Measure axis twist with the load cells (nozzle) against the probe"
        " and the LiDAR (LIDAR=0: probe only)"
    )

    def cmd_AXIS_TWIST_TOUCH_CALIBRATE(self, gcmd):
        atc = self.printer.lookup_object("axis_twist_compensation", None)
        if atc is None:
            raise gcmd.error("AXIS_TWIST_TOUCH_CALIBRATE: no [axis_twist_compensation]")
        probe = self.printer.lookup_object("probe")
        lc = self.printer.lookup_object("load_cell_probe").get_printer_probe()
        x_off, y_off = probe.get_offsets()[:2]
        x0, x1, y = atc.calibrate_start_x, atc.calibrate_end_x, atc.calibrate_y
        n = self.points
        xs = [x0 + (x1 - x0) * i / (n - 1) for i in range(n)]
        samples = gcmd.get_int("SAMPLES", self.samples, minval=1)
        reactor = self.printer.get_reactor()
        self.applied = False
        # the LiDAR (21mm in front, 36mm left of the nozzle) tips with the
        # toolhead too, the other way round than the probe behind it: read it
        # at the same nozzle touches. Its spots are the probe's it can reach,
        # plus the end of its reach (its mesh goes that far)
        lidar = self.printer.lookup_object("lidar", None)
        if not gcmd.get_int("LIDAR", 1) or lidar is None or not lidar.service:
            lidar = None
        lxs = []
        if lidar is not None:
            lxs = [x for x in xs if lidar.twist_reach(x, y)]
            # the end of its reach, skew correction included (it shifts X
            # along Y; without it the end spot fell out and the table
            # stopped at X243)
            end = round(lidar._reach_x(y - 1.0, y + 1.0, lidar.scan_z)[1], 1)
            if (not lxs or end > lxs[-1] + 10.0) and lidar.twist_reach(end, y):
                lxs.append(end)
            if len(lxs) < 3:
                gcmd.respond_info("Axis twist: the LiDAR can't reach enough"
                                  " spots, probe only")
                lidar, lxs = None, []
        # stops along X: (x, probe spot index or None, LiDAR spot or None)
        stops = sorted(set(xs) | set(lxs))
        stops = [(x, xs.index(x) if x in xs else None,
                  lxs.index(x) if x in lxs else None) for x in stops]
        gcode_move = self.printer.lookup_object("gcode_move")
        z_off = gcode_move.get_status(reactor.monotonic())["homing_origin"][2]
        bed_mesh = self.printer.lookup_object("bed_mesh", None)
        mesh = bed_mesh.get_mesh() if bed_mesh is not None else None

        old = (
            list(atc.z_compensations),
            atc.compensation_start_x,
            atc.compensation_end_x,
        )

        def restore(msg):
            atc.z_compensations = old[0]
            atc.compensation_start_x = old[1]
            atc.compensation_end_x = old[2]
            gcmd.respond_info(
                "Axis twist: %s, keeping the current compensation" % (msg,)
            )

        # measure without any compensation applied
        atc.clear_compensations("X")
        gcmd.respond_info(
            "Axis twist: %d spots X%.0f-X%.0f at Y%.0f, %d passes%s"
            % (n, x0, x1, y, self.passes,
               ", LiDAR at %d spots up to X%.0f" % (len(lxs), lxs[-1])
               if lidar else "")
        )
        meas = []   # (spot, pass, time, probe - nozzle)
        lmeas = []  # (LiDAR spot, pass, time, LiDAR - nozzle)
        dists = []  # LiDAR distance with the nozzle on the bed
        try:
            # the LiDAR reads in the toolhead frame: no mesh on top
            if bed_mesh is not None:
                bed_mesh.set_mesh(None)
            self._run("G90\nG0 Z5 F900")
            # the first nozzle tap after a wipe reads off, throw it away
            self._run(
                "G0 X%.2f Y%.2f F9000\nG0 Z2 F900\nLOAD_CELL_PROBE SAMPLES=1\nG0 Z5 F900"
                % (xs[0], y)
            )
            for p in range(self.passes):
                for x, i, j in (stops if p % 2 == 0 else stops[::-1]):
                    if i is not None:
                        self._run(
                            "G0 Z5 F900\nG0 X%.2f Y%.2f F9000\n"
                            "PROBE SAMPLES=%d SAMPLES_RESULT=median"
                            % (x - x_off, y - y_off, samples)
                        )
                        probe_z = probe.last_z_result
                    self._run(
                        "G0 Z5 F900\nG0 X%.2f Y%.2f F9000\nG0 Z2 F900\n"
                        "LOAD_CELL_PROBE SAMPLES=%d SAMPLES_RESULT=median"
                        % (x, y, samples)
                    )
                    nz = lc.last_z_result
                    if i is not None:
                        meas.append((i, p, reactor.monotonic(), probe_z - nz))
                    if j is not None:
                        h = lidar.twist_reading(x, y)
                        lmeas.append((j, p, reactor.monotonic(), h - nz))
                        dists.append(-h - (lidar.scan_z + z_off - nz))
                self._run("G0 Z5 F900")
        except self.printer.command_error as e:
            restore("measuring failed (%s)" % (e,))
            return
        finally:
            if bed_mesh is not None:
                bed_mesh.set_mesh(mesh)
        if lidar is not None:
            self._lidar_result(gcmd, lidar, lmeas, lxs, y, dists)

        try:
            means, offsets, drift, res = fit_twist(meas, n, self.passes)
        except ValueError:
            restore("could not separate the drift from the twist")
            return
        spread = max(max(r) - min(r) for r in res)
        worst = max(range(n), key=lambda i: max(res[i]) - min(res[i]))
        gcmd.respond_info(
            "Axis twist: pass offsets %s, drift %+.4fmm/min"
            % (", ".join("%+.3f" % o for o in offsets), drift * 60.0)
        )
        if self.passes > 1 and spread > self.max_pass_diff:
            restore("passes disagree by %.3fmm at X%.0f (drift taken out)"
                    % (spread, xs[worst]))
            return
        avg = sum(means) / len(means)
        comp = [avg - m for m in means]
        if len(old[0]) == len(comp):
            change = max(abs(a - b) for a, b in zip(comp, old[0]))
            if change > self.max_change:
                restore("result differs %.3fmm from the current values" % (change,))
                return
        # Klipper looks the value up by the nozzle X at the moment it probes,
        # which is the spot minus the probe x_offset
        start, end = x0 - x_off, x1 - x_off
        atc.z_compensations = comp
        atc.compensation_start_x = start
        atc.compensation_end_x = end
        configfile = self.printer.lookup_object("configfile")
        section = "axis_twist_compensation"
        configfile.set(section, "z_compensations", ", ".join("%.6f" % c for c in comp))
        configfile.set(section, "compensation_start_x", "%.1f" % start)
        configfile.set(section, "compensation_end_x", "%.1f" % end)
        self.applied = True
        gcmd.respond_info(
            "Axis twist: %s (probe-nozzle offset %.3f, pass spread %.3f). "
            "In use now, saved by the next SAVE_CONFIG"
            % (", ".join("%+.3f" % c for c in comp), avg, spread)
        )


    def _lidar_result(self, gcmd, lidar, lmeas, lxs, y, dists):
        # same fit as the probe; saved on its own, whatever the probe's
        # result is
        try:
            means, offsets, drift, res = fit_twist(lmeas, len(lxs), self.passes)
        except ValueError:
            gcmd.respond_info("LiDAR twist: could not separate the drift, not"
                              " saved")
            return
        spread = max(max(r) - min(r) for r in res)
        avg = sum(means) / len(means)
        comp = [round(m - avg, 5) for m in means]
        gcmd.respond_info(
            "LiDAR twist (LiDAR - nozzle, um): %s; pass spread %.3fmm"
            % (", ".join("X%.0f %+.0f" % (x, c * 1000.0)
                         for x, c in zip(lxs, comp)), spread))
        if self.passes > 1 and spread > self.max_pass_diff:
            gcmd.respond_info("LiDAR twist: passes disagree by %.3fmm, not"
                              " saved" % (spread,))
            return
        lidar.twist_save(gcmd, lxs, comp, y, lidar.scan_z, spread, dists)


def load_config(config):
    return TwistTouch(config)
