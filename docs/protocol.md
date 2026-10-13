# Protocol

Everything we know about talking to the Creality "AI LiDAR" module, from the
vendor's own binaries and firmware, recorded traffic, and a lot of testing on
one K1 Max. The driver in [`driver/`](../driver/) implements all of this.

> ## Do NOT send these
>
> The module stores some things in its own flash or hangs. Nobody has a
> recovery procedure.
>
> - **0x05 calibration (sticker)**: stores wrong geometry in the module if your
>   plate isn't at the stock height. Same for the middleware's
>   `set_laser_cali_step`.
> - **0x56 IR image capture** (middleware: `create_ir_image`): the module stops
>   answering until power-cycled.
> - **0x1B UART config**: saved in the module's flash. Anything that expects
>   1.5 Mbaud can't talk to it afterwards.
> - **0x10, 0x19, 0x1A OTA firmware update**, **0x12 restore** and **0xAA** (sets the "authorised" flag unchecked, a debug backdoor).
>
> `driver/lidar_serial.py` refuses all of these (`UNSAFE`) and
> `lidar-direct.py` answers code 103 to the matching socket commands.

## Socket API (served by lidar-direct.py and by Creality's cx_ai_middleware)

Unix socket `/tmp/ai_server_uds`. Request = JSON + `0x03` (ETX), reply the
same. Use **one persistent connection**: the middleware leaks a descriptor
per connection and stops accepting after a few thousand.

```
{"control": "set_laser_open"}                  -> {"result": {"code": 0}}
{"control": "open_flow"}                       start streaming
{"control": "set_point_cloud_level", "level": 0}   0 = all points, 1/2 = 1/2, 1/4
{"control": "get_point_cloud"}                 -> {"result": {"code": 0, "point_cloud": [[x,y,z], ...]}}
{"control": "close_flow"}, {"control": "set_laser_close"}
{"control": "get_laser_exposure"} / "set_laser_exposure" {"exposure": n}   default 1000
{"control": "get_laser_gain"} / "set_laser_gain" {"gain": n}               default 1000
```

Without `open_flow`, `get_point_cloud` returns code 4 and no points.

All commands (dispatch table in `.rodata`): `open_flow close_flow
get_point_cloud get_level_value set_point_cloud_level get_point_cloud_level
module_restart set_laser_open set_laser_close set_fill_light_open
set_fill_light_close create_ir_image get_an_ir_image get_flow_detection
get_select_line set_laser_cali_step set_laser_coor get_laser_coor
get_first_floor_detect get_laser_exposure set_laser_exposure get_laser_gain
set_laser_gain set_laser_point_cloud_mode set_laser_roi_area
set_take_over_model get_laser_offset get_laser_offset_0.2mm
get_flow_detection_new get_first_floor_detect_new`

Result codes seen: 0 ok, 4 no data, 101 LiDAR not ready, 102 the module
refused, 103 missing/invalid parameters.

**Don't use `create_ir_image`** (see the box at the top): `type` + `resolution`, both ints; this module refuses it (102) and then stops answering until power-cycled (`laser_power.sh off`, `on`, restart the middleware).

## Serial protocol (host <-> LiDAR module)

From the middleware (`LaserDrive.c`, `SerialPort.c`) and the module's own
firmware (RISC-V, FreeRTOS, `3D-CONTOUR-WQ-V1013-O.bin`), checked against
recorded traffic and used by the open driver (`lidar_serial.py`,
`lidar-direct.py`).

Port `/dev/serial/by-id/creality-laser`, **1 500 000 baud**, 8N1, raw.

```
AF FF | len (u16 LE, = 2 + payload) | src | cmd | payload | checksum (u16 LE)
checksum = -(sum of all bytes before it) & 0xffff
src 01 = host, 02 = module (replies echo cmd)
```

Replies: set commands and actions answer `status` (0 = ok); getters 0x03 0x07
0x09 0x0B 0x0D 0x0F 0x16 0x18 0x55 0x59 answer the bare value; the others
answer `status + data`. Simple commands take ~8 ms.

| cmd | name (firmware) | request payload | reply |
|---|---|---|---|
| 0x01 / 0x02 | stream on / off | - | status |
| 0x03 | software version | - | 3 bytes (1.0.13) |
| 0x04 | **point cloud** | - | status + N x (x, y, z) int16 LE, µm (max 1600 points) |
| 0x05 | calibration (sticker) | step | **never** (stores geometry in the module) |
| 0x06 / 0x07 | point cloud mode set / get | mode | status / value |
| 0x08 / 0x09 | point cloud level set / get | 0 all, 1 half, 2 quarter | status / value |
| 0x0A / 0x0B | ROI set / get | x_min u16, x_max u16 (sensor columns 0..1600) | status / 4 bytes |
| 0x0C / 0x0D | stream mode set / get (stream off only) | 0/1 | status / value |
| 0x0E / 0x0F | coordinate mode set / get (init sets 1) | 1 | status / value |
| 0x10, 0x19, 0x1A | OTA firmware update | | **never** |
| 0x11 | reset | - | status |
| 0x12 | restore | - | **never** |
| 0x13 | laser on/off | 1 / 0 | status |
| 0x14 | fill light on/off | 1 / 0 | status |
| 0x15 / 0x16 | exposure set / get | u24 LE (default 1000) | status / 3 bytes |
| 0x17 / 0x18 | gain set / get | u24 LE (default 1000) | status / 3 bytes |
| 0x1B | UART config | baud index (0 1.5M, 1 2M, 2 2.5M), flow control | **saved in the module's flash**: the middleware (fixed 1.5M) can't talk to it afterwards |
| 0x1C | IR sensor temperature | - | status + 4 bytes |
| 0x1D | serial number | - | status + 12 bytes ASCII |
| 0x1E | calibration parameters | - | status + 100 bytes |
| 0x1F, 0x20 | debug print, self test | | |
| 0x21 | point cloud in 2000-byte chunks | | |
| 0x55 | status | - | error bits (0 = ok) |
| 0x56 / 0x57 | IR image capture / download | | 0x56 **never** (hangs the module) |
| 0x58 / 0x59 / 0xAB | watchdog | | |
| 0xA0 / 0xA1 | auth challenge / key | - / 32 bytes | see below |
| 0xAA | set authorised flag | | sets the flag unchecked (debug backdoor) |

Before authentication the module only answers 0x55, 0xA0, 0xA1, 0xAA.
**Authentication** (no secret): 0xA0 answers `status, 8 bytes`; negate each
byte (`-b & 0xff`); bytes 0-3 = int32 LE `value`, 4-7 = 4 alphanumeric chars
`r`; key = `SHA256(str(value) + md5_hex_lowercase(r))`, sent with 0xA1 (reply
status 0 = ok). udev runs `cx_ai_crypto` for this at every enumeration; the
open driver does it itself.

**Init** (middleware order): 0x55 (must be 0), auth, 0x03, 0x1D, 0x0E = 1.
Then laser on (0x13 1), stream on (0x01), point clouds (0x04).

**Timing**: 0x04 waits for the next captured frame: ~140 ms until the reply
starts, ~60 ms for 9 KB at 1.5 Mbaud, so **~5 frames/s is the module's
limit** (direct: 4.98/s; through the middleware ~4.3/s). The module runs on
a 28.4 ms sensor tick: back-to-back frames come every 7 ticks (199 ms) at
level 0, 6 (170 ms) at level 1, 5 (142 ms) at level 2, 4 (114 ms) when the
frame is empty. A request that comes late slips the frame by a whole tick.
ROI and exposure don't change the rate (this module refuses 0x0A, status 1).

**What a frame covers** (LIDAR_TIMING, fast Z ramps over the plate, times
taken by lidar-direct.py when the frame arrives from the module): level 0
~197..163 ms before arrival (median 178, sd 10 ms), level 1 ~163..120 ms.
Measure with fast ramps (5 mm/s): the laser hits at ~45 deg, so a Z move
also slides the line over the plate's texture, which reads as height noise
(at 0.5 mm/s it smeared the result to a ~190 ms wide window). Scans that
stand still only for that measured window still came out blurred: leave
generous still time after it (lidar.py does, up to the next row's deadline).

**Dwell scans locked to the frames** (lidar.py, direct driver): frames are
streamed back-to-back by lidar-direct.py (`prefetch`) and time-stamped in
Klipper's clock; rows are planned so the toolhead stands still for one
frame's window and moves to the next row in between: one frame per row,
0.2 s. 51 rows (Y 20 mm, 0.4 mm steps): 12-14 s, was 20-21 s; 0 rows missed
in 3 runs; matches a stop-mode scan of the same strip within the scan noise
(p05..p95 of the difference 18-31 um, two stop-mode-era scans: 26 um).
