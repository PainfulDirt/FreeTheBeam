# Nozzle cleaning on the rear edge of the bed, without wipe pads
#
# Creality's NOZZLE_CLEAR (prtouch_v2_wrapper.py, clear_nozzle(), K1 source
# on GitHub) plus what the old macro wipe did well:
#   1. a random start along the strip every time, so the residue of earlier
#      wipes isn't wiped into the nozzle again (stock). Everything, peel and
#      flick included, stays inside the strip: the probe 20.3mm behind the
#      nozzle and the keep-out zones leave no other room at the back
#   2. the bed height measured with the load cells at both ends of the wipe,
#      with the bed mesh off, so the press depth is relative to the real
#      surface (stock)
#   3. hover just above the start while heating to the filament's print
#      temperature: the ooze lands on the PEI and anchors there (stock)
#   4. hot drag down into the PEI, then scrub passes on fresh lines next to
#      it (the old macro)
#   5. lift 0.2mm and push out a small blob, then press back down into the
#      PEI: the blob bonds to the plate and wraps around the tip
#   6. short retract while still hot (no extruder moves below
#      min_extrude_temp): the melt inside lets go of the blob, and the nozzle
#      doesn't keep oozing afterwards
#   7. fan on, cool down pressed: the blob sets on the PEI around the tip,
#      with the residue in it (stock cools pressed, the old macro too)
#   8. slow drag rising 5 mm (1 mm/s), extra slow over the first 3 mm
#      (0.25 mm/s): the set blob peels everything off the tip (stock: 2 mm/s)
#   9. flick: dip over the last scrub line and move fast, a string still
#      hanging off the tip catches on the PEI and snaps
# As NOZZLE_SCRUB (Kalico's load cell nozzle_scrubber_gcode, run on bad taps)
# it skips the touches and presses relative to the strip height measured by
# the last full wipe: a touch inside a running probe would nest probe sessions.
#
# Copy or symlink into klippy/plugins (see docs/install-k1max.md).
# Part of FreeTheBeam, https://github.com/PainfulDirt/FreeTheBeam
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import random


class NozzleWipe:
    def __init__(self, config):
        self.printer = config.get_printer()
        # the strip (nozzle coordinates): the whole wipe stays inside it
        self.x_min = config.getfloat("x_min", 40.0)
        self.x_max = config.getfloat("x_max", 100.0)
        self.y_min = config.getfloat("y_min", 297.4)
        self.y_max = config.getfloat("y_max", 298.6)
        self.wipe_len = config.getfloat("wipe_len", 25.0, above=5.0)
        # the peel runs on from the end of the drag, towards +X
        self.peel_len = config.getfloat("peel_len", 25.0, above=5.0)
        self.peel_rise = config.getfloat("peel_rise", 5.0, above=0.0)
        # mm/s: slow, extra slow over the first break_len while the set
        # plastic lets go of the tip
        self.peel_speed = config.getfloat("peel_speed", 1.0, above=0.0)
        self.break_speed = config.getfloat("break_speed", 0.25, above=0.0)
        self.break_len = config.getfloat("break_len", 3.0, above=0.0)
        # stock pa_clr_down_mm: hover this far above the start, press this far
        # into the PEI at the end
        self.down = config.getfloat("down", 0.15, minval=0.0, maxval=0.3)
        # mm/s for the drags (stock clr_xy_spd 2)
        self.speed = config.getfloat("speed", 2.0, above=0.0)
        self.scrub_passes = config.getint("scrub_passes", 2, minval=0)
        self.scrub_speed = config.getfloat("scrub_speed", 15.0, above=0.0)
        self.y_step = config.getfloat("y_step", 0.6, above=0.0)
        self.retract = config.getfloat("retract", 1.0, minval=0.0, maxval=3.0)
        # mm of filament pushed out before cooling, this far above the strip
        # (0: cool pressed into the PEI instead)
        self.blob = config.getfloat("blob", 1.0, minval=0.0, maxval=5.0)
        self.blob_z = config.getfloat("blob_z", 0.2, minval=0.05, maxval=1.0)
        # without a print temperature (manual run) wipe at this
        self.wipe_temp = config.getfloat("wipe_temp", 200.0, minval=150.0)
        self.max_wipe_temp = config.getfloat("max_wipe_temp", 260.0)
        # touch the strip at this, cool down to cool_temp while pressed
        self.touch_temp = config.getfloat("touch_temp", 150.0)
        self.cool_temp = config.getfloat("cool_temp", 160.0)
        self.flick = config.getboolean("flick", True)
        # above the strip: a hanging string reaches down further, set
        # residue on the scrub lines doesn't
        self.flick_z = config.getfloat("flick_z", 1.0, above=0.2)
        # what measures the strip: "nozzle" (load cells, like stock: reads
        # early by whatever sticks to the tip) or "probe" (Microprobe on the
        # same spots, 20.3mm behind: blind to the tip, needs the axis twist
        # compensation to match the nozzle)
        self.strip_sensor = config.getchoice(
            "strip_sensor", {"nozzle": "nozzle", "probe": "probe"}, "nozzle"
        )
        # a strip touch further than this from Z0 is not the strip
        self.max_surface = config.getfloat("max_surface", 0.5, above=0.0)
        room = self.x_max - self.x_min - self.wipe_len - self.peel_len
        if room < 0.0:
            raise config.error(
                "nozzle_wipe: wipe_len + peel_len don't fit between x_min and"
                " x_max"
            )
        self.active = False
        # strip height of the last full wipe (z at the strip start, slope/mm)
        self.strip = None
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "NOZZLE_WIPE", self.cmd_NOZZLE_WIPE, desc=self.cmd_NOZZLE_WIPE_help
        )
        self.gcode.register_command(
            "NOZZLE_SCRUB", self.cmd_NOZZLE_SCRUB, desc=self.cmd_NOZZLE_SCRUB_help
        )

    def get_status(self, eventtime):
        return {"active": self.active}

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _temp(self, temp):
        # M109 only waits while heating up (MPC), cooling needs its own wait
        heater = self.printer.lookup_object("extruder").get_heater()
        now = self.printer.get_reactor().monotonic()
        # within 5C like stock: the heater holding the target makes the
        # last degrees slow
        if heater.get_temp(now)[0] > temp + 5.0:
            self._run(
                "M104 S%.0f\nTEMPERATURE_WAIT SENSOR=extruder MAXIMUM=%.0f"
                % (temp, temp + 5.0)
            )
        else:
            self._run("M109 S%.0f" % (temp,))

    def _touch(self, x, y, sensor):
        if sensor == "probe":
            # the pin lands where the nozzle would; the result minus
            # z_offset is the surface (twist compensation already added)
            probe = self.printer.lookup_object("probe")
            x_off, y_off, z_off = probe.get_offsets()[:3]
            self._run(
                "G0 Z5 F900\nG0 X%.2f Y%.2f F6000\n"
                "PROBE SAMPLES=3 SAMPLES_RESULT=median\nG0 Z5 F900"
                % (x - x_off, y - y_off)
            )
            z = probe.last_z_result - z_off
        else:
            lc = self.printer.lookup_object("load_cell_probe").get_printer_probe()
            self._run(
                "G0 Z5 F900\nG0 X%.2f Y%.2f F6000\nG0 Z1 F900\n"
                "LOAD_CELL_PROBE SAMPLES=1\nG0 Z5 F900" % (x, y)
            )
            z = lc.last_z_result
        if abs(z) > self.max_surface:
            raise self.printer.command_error(
                "NOZZLE_WIPE: strip touched at Z%.3f at X%.1f Y%.1f, expected"
                " ~0: not wiping there" % (z, x, y)
            )
        return z

    cmd_NOZZLE_WIPE_help = (
        "Clean the nozzle on the rear edge of the bed (TEMP=print temperature)"
    )

    def cmd_NOZZLE_WIPE(self, gcmd):
        # a failed wipe doesn't stop the print: the True Zero touch checks
        # the nozzle against the Microprobe afterwards anyway
        try:
            self._wipe(gcmd, touch=True)
        except self.printer.command_error as e:
            gcmd.respond_info("Nozzle wipe skipped: %s" % (e,))
            self._run("G90\nG0 Z5 F900\nM106 S0")

    cmd_NOZZLE_SCRUB_help = (
        "Nozzle wipe without the strip touches, back to where it started"
        " (for the load cell's nozzle_scrubber_gcode)"
    )

    def cmd_NOZZLE_SCRUB(self, gcmd):
        if self.active:
            # a bad tap during the wipe's own touches: just retry the tap
            return
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        temp = self.printer.lookup_object("extruder").get_heater().target_temp
        self._wipe(gcmd, touch=False)
        self._temp(temp)
        # back to the exact toolhead position the probe left (manual moves:
        # G-code moves would add the bed mesh a second time)
        z = toolhead.get_position()[2]
        toolhead.manual_move([None, None, max(z, pos[2], 5.0)], 15.0)
        toolhead.manual_move([pos[0], pos[1], None], 100.0)
        toolhead.manual_move([None, None, pos[2]], 15.0)
        self.printer.lookup_object("gcode_move").reset_last_position()

    def _wipe(self, gcmd, touch):
        sensor = gcmd.get("SENSOR", self.strip_sensor).lower()
        if sensor not in ("nozzle", "probe"):
            raise gcmd.error("NOZZLE_WIPE: SENSOR=nozzle or SENSOR=probe")
        temp = gcmd.get_float("TEMP", 0.0)
        temp = min(temp if temp >= 150.0 else self.wipe_temp, self.max_wipe_temp)
        after = gcmd.get_float("AFTER_TEMP", self.touch_temp)
        bed_mesh = self.printer.lookup_object("bed_mesh", None)
        mesh = bed_mesh.get_mesh() if bed_mesh is not None else None
        random.seed()
        x0 = random.uniform(
            self.x_min, self.x_max - self.wipe_len - self.peel_len
        )
        x1 = x0 + self.wipe_len
        # the scrub lines step towards the bed's edge, inside the strip
        y_room = self.y_max - self.y_min - self.scrub_passes * self.y_step
        y0 = self.y_min + random.uniform(0.0, max(y_room, 0.0))
        ys = 1.0
        self.active = True
        self._run("SAVE_GCODE_STATE NAME=_nozzle_wipe\nG90\nM83\nM106 S0")
        try:
            if bed_mesh is not None:
                bed_mesh.set_mesh(None)
            if touch:
                self._temp(self.touch_temp)
                z0 = self._touch(x0, y0, sensor)
                z1 = self._touch(x1, y0, sensor)
                self.strip = (x0, z0, (z1 - z0) / self.wipe_len)
            elif self.strip is not None:
                sx, sz, k = self.strip
                z0, z1 = sz + k * (x0 - sx), sz + k * (x1 - sx)
            else:
                z0 = z1 = 0.0
            gcmd.respond_info(
                "Nozzle wipe at X%.1f-%.1f Y%.1f, strip Z%+.3f/%+.3f (%s), %.0fC"
                % (x0, x1, y0, z0, z1, sensor if touch else "last wipe", temp)
            )
            # hover over the start while heating: the ooze anchors on the PEI
            self._run(
                "G0 Z5 F900\nG0 X%.3f Y%.3f F6000\nG1 Z%.3f F300"
                % (x0, y0, z0 + self.down)
            )
            self._temp(temp)
            # hot drag down into the PEI, then scrub on fresh lines
            f = self.speed * 60.0
            self._run("G1 X%.3f Z%.3f F%.0f" % (x1, z1 - self.down, f))
            x, y = x1, y0
            for i in range(self.scrub_passes):
                y += ys * self.y_step
                x = x0 if x == x1 else x1
                z = (z0 if x == x0 else z1) - self.down
                self._run(
                    "G1 Y%.3f F600\nG1 X%.3f Z%.3f F%.0f"
                    % (y, x, z, self.scrub_speed * 60.0)
                )
            # to the end of the drag: the peel runs from there towards +X,
            # into the room left for it on the strip
            if x != x1:
                self._run("G1 Z%.3f F300\nG1 X%.3f F%.0f" % (z1 + 0.3, x1, f * 5))
            if self.blob:
                # fresh plastic, pressed back down so it bonds to the plate
                # and wraps around the tip
                self._run(
                    "G1 Z%.3f F300\nG1 E%.2f F60"
                    % (z1 + self.blob_z, self.blob)
                )
            zc = z1 - self.down
            self._run("G1 Z%.3f F120" % (zc,))
            # while still hot: the melt inside lets go of what sets outside
            if self.retract:
                self._run("G1 E-%.2f F1800" % (self.retract,))
            # cool down pressed: the blob (or the residue) sets on the PEI
            self._run("M104 S%.0f\nM106 S255" % (self.cool_temp,))
            self._temp(self.cool_temp)
            # slow rising drag, extra slow while it breaks free: the set
            # plastic peels everything off the tip
            self._run(
                "G1 X%.3f Z%.3f F%.0f\nG1 X%.3f Z%.3f F%.0f"
                % (x1 + self.break_len, zc + 0.3, self.break_speed * 60.0,
                   x1 + self.peel_len, z1 + self.peel_rise,
                   self.peel_speed * 60.0)
            )
            if self.flick:
                # a string still on the tip: dip over the last scrub line and
                # move fast, it catches on the PEI and snaps off
                fy = y
                self._run(
                    "G0 X%.3f Y%.3f F6000\nG1 Z%.3f F600\nG1 X%.3f F12000\n"
                    "G0 Z5 F900"
                    % (x1, fy, z1 + self.flick_z, x0)
                )
            self._run("G0 Z5 F900\nM106 S0")
            self._temp(after)
        finally:
            if bed_mesh is not None:
                bed_mesh.set_mesh(mesh)
            self.active = False
            self._run("RESTORE_GCODE_STATE NAME=_nozzle_wipe")


def load_config(config):
    return NozzleWipe(config)
