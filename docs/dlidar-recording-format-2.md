# dLidar recording format 2: what dMapper can use for intensity normalization

Received from the dLidar session (the Unitree L2 capture tool) on 2026-10-15 and kept here unchanged in content. The authoritative spec is in the dLidar repo: `docs/l2-recording-format.md` and `docs/recording-format-2.html`.

dLidar now records the per-point and per-packet data needed for a range and incidence-angle intensity correction. This is "recording format 2". It only adds data: everything dMapper reads today (LAZ X/Y/Z, gps_time, intensity, user_data, and the IMU CSV) is unchanged, and was verified identical point for point and byte for byte against format 1 on replayed packets. The existing reading code needs no change. The new data is optional input for the normalization.

## 1. Detecting the format
- LAZ header `generating_software == "dLidar recording format 2"` → format 2. Anything else (e.g. "LASzip DLL 3.4 r4 (231020)") → format 1.
- Also: `point_data_record_length` is 48 in format 2 (28 in format 1); `laszip_point->num_extra_bytes` is 20 (0 in format 1); `timingNNNN.json` has `"recording_format_version": 2` (the key is missing in format 1).
- The normalization must still work on format 1 recordings, which have no extra bytes and no sidecar. Fall back to range = |xyz| and beam direction = xyz/|xyz|. That fallback is approximate: the true emission point is offset from the origin by up to ~8 mm (see section 4).

## 2. Files of a recording (one folder, session number NNNN)
- `lidarNNNN.laz`: points (format 2 adds extra bytes).
- `imuNNNN.csv`: unchanged.
- `timingNNNN.json`: gains keys in format 2 (see section 5).
- `packetsNNNN.bin`: unchanged (diagnostic packet timing).
- `packetstateNNNN.tsv`: NEW in format 2, one row per point packet. It is .tsv on purpose so dMapper's folder scan (every .csv/.las/.laz) ignores it. Don't add any .csv output to the recording folder either.

## 3. LAZ point fields
LAS 1.2, point format 1, scale 0.0001, offset 0, legacy point count filled.

Standard fields (same in both formats):
- X/Y/Z [m]: the L2's calibrated sensor frame remapped for the vertical mount: x = orig_z, y = orig_y, z = −orig_x (+X front, +Y left, +Z up). No mounting transform or lever arm.
- gps_time [s]: L2 clock, zero = L2 stamp of the first point packet. Same clock and zero as the IMU CSV `timestamp` column [ns].
- intensity: the raw 0–255 byte from the L2, unscaled. It is not reflectivity: Unitree documents no scale, gain control or range compensation.
- user_data: always 1 (no channel information). classification, point_source_ID, return_number, scan angle: always 0. There is no channel or emitter id anywhere.
- Points dropped at capture: no return, outside the packet's range_min/max, range < 0.1 m or > 100 m, intensity < 5. So intensity 0–4 never appears.

Extra bytes (format 2): 20 bytes after the 28 standard bytes, little-endian, in this order, described by the standard Extra Bytes VLR. With the LASzip API read them from `laszip_point->extra_bytes` at these offsets.

| offset | name         | type    | unit | meaning |
|--------|--------------|---------|------|---------|
| 0      | range        | float32 | m    | corrected range r = range_scale·(raw_range + range_bias); the float the parser used for xyz |
| 4      | raw_range    | uint16  | mm   | range as sent by the L2 |
| 6      | sample_index | uint16  | –    | j, the point's index in its packet (0..299); filtered points leave gaps |
| 8      | packet_seq   | uint32  | –    | packet info.seq (wraps at 1024 on the L2) |
| 12     | alpha        | float32 | rad  | alpha scan angle used for the point, bias included |
| 16     | theta        | float32 | rad  | theta scan angle used for the point, bias included; not wrapped to [0, 2π) |

Every attribute has scale 1 and offset 0 in the VLR (LASzip always sets those option bits), so the stored numbers are the values. laspy returns the integer attributes as floats of the same value.

## 4. Beam geometry (exact sensor model, from dLidar's parser)
With α = alpha, θ = theta, r = range, and β = beta_angle, ξ = xi_angle, a = a_axis_dist, b = b_axis_dist taken from the point's packet row in `packetstateNNNN.tsv` (section 6):

    A = (−cosβ·sinξ + sinβ·cosξ·sinα)·r + b
    B = cosα·cosξ·r
    C = (sinβ·sinξ + cosβ·cosξ·sinα)·r
    orig = (cosθ·A − sinθ·B,  sinθ·A + cosθ·B,  C + a)        # physical sensor frame
    point (x, y, z) = (orig_z, orig_y, −orig_x)                # dLidar output frame = LAZ X/Y/Z

- orig is affine in r: orig(r) = P0 + r·D.
- Emission point P0 = orig(0) = (cosθ·b, sinθ·b, a).
- Beam direction D = (cosθ·D1 − sinθ·D2, sinθ·D1 + cosθ·D2, D3), with D1 = −cosβ sinξ + sinβ cosξ sinα, D2 = cosα cosξ, D3 = sinβ sinξ + cosβ cosξ sinα. Normalize it.
- Apply the same remap (x, y, z) = (orig_z, orig_y, −orig_x) to P0 and D to get them in the LAZ frame.
- For the incidence angle use the angle between −D̂ and the surface normal that dMapper estimates at the point. The true range is r, not |xyz|; they differ by at most hypot(a, b) (≈8 mm in the replay).
- Recomputing xyz from these values and the float32 parameters reproduces the LAZ coordinates to the 0.1 mm quantization.

## 5. timingNNNN.json additions (format 2)
`recording_format_version` (2), `lidar_firmware_version`, `lidar_hardware_version`, `sdk_version` ("2.0.4"), `work_mode` (0). Existing keys are unchanged; `zero_l2_ns` is the recording zero. The firmware/hardware version is "unknown" if the L2 didn't answer within 3 s / 1 s. That query is untested on a real L2.

## 6. packetstateNNNN.tsv (format 2)
Tab-separated, one header line, one row per point packet in arrival order (packets that yielded no point included). Floats are printed with 9 significant digits, which gives back the float32 exactly; parse them as float32.

Columns:
`stamp_ns l2_stamp_ns seq point_num time_increment scan_period range_min range_max apd_temperature apd_voltage laser_voltage imu_temperature dirty_index packet_lost_up packet_lost_down sys_rotation_period com_rotation_period range_scale range_bias a_axis_dist b_axis_dist theta_angle_bias alpha_angle_bias beta_angle xi_angle angle_min angle_increment com_horizontal_angle_start com_horizontal_angle_step`

- stamp_ns [ns] = l2_stamp_ns − zero_l2_ns: the time of the packet's first point (j = 0), on the LAZ/IMU zero.
- seq = packet_seq. A step of n > 1 (mod 1024) means n−1 lost point packets.
- time_increment [s] between points, scan_period [s], range_min/range_max [m].
- Calibration: range_scale [1], range_bias [mm], a_axis_dist/b_axis_dist [m]; theta_angle_bias, alpha_angle_bias, beta_angle, xi_angle, angle_min, angle_increment, com_horizontal_angle_start, com_horizontal_angle_step [rad].
- Sensor state, units undocumented by Unitree (guesses in brackets): apd_temperature [°C?], apd_voltage [V?], laser_voltage [V?], imu_temperature [°C?], dirty_index, packet_lost_up, packet_lost_down, sys_rotation_period, com_rotation_period (integers). The APD temperature/voltage and laser voltage are candidate covariates for intensity drift. They are per packet (~215 Hz).

Matching a point to its row:
- Points in the LAZ are in packet order. A new packet starts where packet_seq changes or sample_index does not increase. Walk the TSV rows forward matching seq (seq wraps at 1024, so don't use it as a global key).
- Check: gps_time == stamp_ns/1e9 + sample_index·(double)time_increment. This is exact in double, bit for bit.
- Alternatively, match on stamp: stamp_ns ≈ (gps_time − sample_index·time_increment)·1e9.

## 7. What is known about L2 intensity (recording 0001, real data, format 1, 11.3 M points, one indoor scene)
- Range 5..255, median 238; 37.8 % of all points are exactly 255.
- By range (|xyz|): share at 255 is 0 % below 0.5 m, 0.3 % at 0.75–1 m, 23 % at 1–1.5 m, 61–62 % at 1.5–3 m, 51 % at 3–5 m, then 8–25 % beyond 5 m. Mean intensity rises from ~145 at 0.1–0.3 m to ~231 at 2–3 m.
- So: heavy clipping at mid range. Treat 255 as censored (a lower bound), not as a value. The response is reduced below ~1 m; that this is near-range signal loss rather than scene content is a guess.
- Not known: emitter count (likely one beam with a scanning mirror: each packet is 300 samples at ~7.73 µs spanning 2.32 ms, packets every ~4.64 ms; unproven), gain control, temperature compensation. Unitree documents none of these.

## 8. Test data
- `/home/vs2022/dLidar_format_samples/format1_old/` and `/home/vs2022/dLidar_format_samples/format2_new/`: the same 30 s replay written in both formats. Running dMapper on both must give bit-identical results.
- The content is SYNTHETIC: real packet timing from recording 0001, but a stationary sensor in a box room with modelled intensity (reflectivity × cos(incidence) × range falloff, near-range reduction, clipped at 255). Good for testing the parsing, packet matching and geometry; not for fitting the correction.
- There is no real format 2 recording yet (the sensor was offline). The first real one will come from the next capture.
- Make more replays with dLidar's `bin/replay_capture packetsNNNN.bin OUT_DIR [seconds]`. Compare formats with `tools/compare_recording_formats.sh` (see `docs/tools.html` in dLidar).
