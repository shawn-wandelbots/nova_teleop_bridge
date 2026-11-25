# What the A3 expects on `/remote/vr_data` – analysis of the on-robot PICO sample

Everything here was obtained **offline and read-only** on the robot's head board:

* the 29 s sample in `/opt/agibot/data/pico_samples/` was replayed with
  `/opt/agibot/bin/pico_mock_player --host 127.0.0.1 --port 39118 <sample.mcap>` into
  `tools/capture_udp.py` (loopback, private port), then analysed with `tools/analyze_frames.py`;
* the robot configs were only read.

Reproduce:

```
python3 tools/capture_udp.py --port 39118 /tmp/sample.cap &
/opt/agibot/bin/pico_mock_player --host 127.0.0.1 --port 39118 /opt/agibot/data/pico_samples/<sample>.mcap
python3 tools/analyze_frames.py /tmp/sample.cap
NTB_FULL_SAMPLE=/tmp/sample.cap python3 -m unittest tests.test_a3_proto     # golden test on all frames
```

Two excerpts of that capture are checked in as test fixtures (`tests/data/*.cap`: every 100th
frame, and the contiguous frames 880–999 which contain the fastest motion).

## 1. Wire-level facts (VERIFIED on the sample)

| Item | Value |
|---|---|
| Datagrams | 2921, one raw protobuf message each, no extra header |
| Size | 1038 bytes, every frame |
| Rate | 100.0 fps nominal; arrival dt mean 10.16 ms, median 10.71 ms, p99 25.4 ms, max 99.3 ms (player jitter) |
| `timestamp` (field 1) | constant `4294967295` (0xFFFFFFFF) in every frame |
| `left_hand` / `right_hand` / `controller_raw_datas` | empty in every frame |
| `body_data` (field 4) | exactly 24 joints, `joint_code_name` 0..23 in order |
| Scalar encoding | every scalar is written, zeros included (`08 00` for joint code 0) – proto2-style explicit presence |
| Decode → re-encode | **byte-identical for 2921 / 2921 frames** (`encode_xr_frame(..., explicit=True)`) |
| Identical consecutive frames | 298 of 2920 (≈10 %): the tracker runs slower than the 100 Hz stream, repeats are normal |

Consequences for the bridge: it encodes with explicit zeros (same byte layout as a real PICO
stream; also valid proto3 input), always sends all 24 joints, and does **not** treat a repeated
pose as an error.

Caveat: the robot config calls the UDP format "proto2 package `xr_data`" (pico-direct link),
while the ROS topic type is `aimdk.protocol.XRFrameData`. The sample decodes cleanly with the
`XRFrameData` field numbers, so the two are field-compatible for body data, but the sample is
strictly ground truth for the **UDP** link. See §5.

## 2. Coordinate frame (VERIFIED on the sample)

**Right-handed, Z-up, metres. Origin is NOT on the floor – it is roughly at head height.**

| Quantity | Number |
|---|---|
| HEAD z | first frame −0.039, mean −0.120, range −0.500 … −0.014 |
| PELVIS z | mean −0.631 (−0.732 … −0.564) |
| LEFT/RIGHT_ANKLE z | mean −1.433 / −1.441 |
| LEFT/RIGHT_FOOT z | mean −1.495 / −1.511 (range −1.574 … −1.416) |
| lowest foot z per frame | mean −1.513 |
| HEAD z − lowest FOOT z | median 1.431 m, max 1.481 m (head *joint*, not top of head; a ~1.70 m person) |
| x / y extent of the pelvis | x −0.75 … +1.26, y −0.77 … +0.38 (the person walks around) |

* **Up = +z.** x and y are horizontal; z only spans what a body can span.
* **Units = metres.** Bone lengths: hip→knee 0.369, knee→ankle 0.398, shoulder→elbow 0.233,
  hip width 0.116, shoulder width 0.368 (constant over the recording – a fitted skeleton).
* **Floor / origin.** z = 0 is about where the head was when tracking started; the floor is near
  z ≈ −1.5. The horizontal origin is likewise arbitrary (near the start position).
  The robot does not need a floor-aligned origin: `calibration_max_toe_z: 0.10` is commented
  "After ground-plane normalization, foot Z should be near 0", i.e. teleop_avatar normalises
  the ground plane itself during calibration.
* **Handedness / left-right.** Frame 0 is a T-pose: LEFT_HAND = (−0.777, +0.026, −0.095),
  RIGHT_HAND = (+0.687, +0.510, −0.102), toes ahead of ankles by ≈ +0.09…0.11 in y. So with
  +z up, the person's right is ≈ +x and forward ≈ +y there: right × forward = up → right-handed.
  Over all frames cos∠(up × right, ankle→toe) has median +0.82 (a left-handed frame would give
  negative values).
* **Heading is free.** Pelvis yaw starts at +98.5° and covers −180°…+180° during the recording,
  so the robot cannot be assuming a fixed world heading; calibration takes care of it.

## 3. Quaternion convention (VERIFIED as far as the data allows)

* Order on the wire is `x, y, z, w` (fields 1..4); all norms are 1.0000000.
* **Orientations are global** (world frame), not parent-relative: in the T-pose every joint's
  quaternion is close to the pelvis quaternion (median 15.9°, max 49.5° over 24 joints × 50 frames).
* **Active, local→world, joint-local axes = +x forward, +y left, +z up** (ROS REP-103 body
  convention). Rotating the local axes by the PELVIS quaternion and comparing with directions
  built from joint positions (hips, spine) gives, over all 2921 frames:
  local +x vs forward: median 3.5°; local +y vs left: median 3.4°; local +z vs spine-up: median 3.9°
  (SPINE3 with the collars: 3.0° / 2.1° / 3.7°). For comparison the same test gives a median worst-axis error of
  118.8° for the conjugate (world→local) reading and 133.2° for a w-first reading, versus 3.9°
  for x,y,z,w active.
* Therefore **identity quaternion = standing upright facing world +x**. Frame 0:
  PELVIS q = (+0.043, −0.085, −0.752, −0.652) = a yaw of +98.5° about +z, matching the +y facing
  seen in the positions.
* The sign is not normalised: 62747 of 70104 samples have w < 0 and there are 90 sign flips
  between consecutive frames. `q` and `−q` are therefore both accepted by the robot; the bridge
  emits w ≥ 0.

This is exactly what the standard Unity→ROS conversion (ros.x = unity.z, ros.y = −unity.x,
ros.z = unity.y) produces from PICO's native Y-up data, which is why the provisional
XRoboToolkit transform in `config/transform.json` maps an OpenXR-style frame the equivalent way
(robot.x = −xrt.z, robot.y = −xrt.x, robot.z = xrt.y).

## 4. Motion statistics (used for the bridge's plausibility limits)

Per-frame (10 ms) joint displacement: median of the per-frame maximum 0.0087 m, p99 0.073 m,
**max 0.143 m** (RIGHT_FOOT, frame 913 → 14.3 m/s for one frame); hands max 0.103 m; pelvis max
0.037 m. Bridge defaults: a jump of up to `--min-jump 0.25` m is always accepted, larger ones
only if below `--max-speed 15` m/s × time since the last accepted frame. The whole fixture
passes these limits (unit test `test_real_sample_passes_all_limits`).

## 5. What the robot configs say (read-only)

`/opt/agibot/config/teleop_avatar/em_aimrt_teleop_avatar.yaml`

| Setting | Value | Meaning for us |
|---|---|---|
| `input_source` | `pico-ros2` (default), subscribes `pico_xr_subscribe_topic: /remote/vr_data` | the planned topic is the default input link |
| `pico_port` / `pico_urdr_udp_port` | 25618 / 25619 | live UDP inputs – this project never touches them |
| `output_rate_hz` | 60 | TA solves at 60 Hz; 100 Hz input is what PICO delivers |
| `require_fresh_full_frame` | true, with `max_stale_joint_count: 0`, `joint_hold_timeout_ms: 0` | every frame must carry all joints, nothing is held over |
| `input_hold_timeout_ms` | −1 ("do not republish stale /ta/whole_body_command") | the robot itself does not repeat stale output |
| `calibration_frame_count` / `calibration_timeout_ms` | 30 / 8000 | calibration needs 30 good frames within 8 s |
| `calibration_max_height_range` | 0.08 m | operator must stand still (height) during calibration |
| `calibration_max_foot_range` | 0.05 m | feet must not move during calibration |
| `calibration_max_toe_z` | 0.10 m | after ground-plane normalisation feet must be near z = 0 |
| `auto_calibrate` | false | calibration is started by the X+Y+A+B chord (`pico.xyab`, on release) |
| `hand_trig_threshold` | 0.9 | grip dead-man level |
| `control_mux.pico_axis_deadzone` / `pico_axis_stale_timeout_ms` | 0.05 / 1000 | stick handling |
| `control_timeout.slowdown` | ok 150 ms, soft 200 ms, window 2000 ms | input gaps > 150–200 ms already slow the robot down |
| `control_timeout.hard_timeout.enter_ms` | 1000 → `motion_matching_mix` (service: `whole_body_reset`), fallback `replay_then_pause` | on input loss the robot leaves take-over by itself |
| `input_bindings` | `pico.dual_axis_click` press → `emergency.stop`; `pico.a` double-tap → session toggle; … | TA reads buttons from the frame's `controller_raw_datas` |
| `translation_scale` | 1.0 | metres in, no rescale |
| `profile_id` / `robot_model` | `a3_lite_realtime_v1` / `a3_lite` | the installed profile says "a3_lite", not "ultra" |

`/opt/agibot/config/common/a3_pico.yaml`: `skeleton_normalization_height: 1.612610795`,
`skeleton_normalization_scale: 1.130521`, `skeleton_alignment: true`, `skeleton_mirror: false`,
key frames `head: Head`, toes `LeftFoot` / `RightFoot` – the retargeter rescales the operator
skeleton itself, so no scale is applied in the bridge (`scale: 1.0`).

`/opt/agibot/config/hal_remote/**` (the robot's normal publisher of both topics)

* `hal_remote` listens on UDP 25618 (`external_device: GENIELINK`) and 15000 (`YUANJI_APP`),
  answers on 51000, and publishes `/remote/vr_data`, `/remote/vr_data/raw`, `/remote/vr_button`,
  `/remote/button_edge`, `/event_trigger/button_event`.
  `control_flow_watchdog.required_body_joint_count: 24`.
* **Controller numbering:** `forward_lateral_controller: 0  # 左手柄 controller_code_name (DEVICE_LEFT)`,
  `turn_controller: 1  # 右手柄 (DEVICE_RIGHT)`; hold-repeat bindings: "left(0).key_two = Y,
  right(1).key_two = B … key_one is X/A". So **0 = left, 1 = right, key_one = X/A, key_two = Y/B**
  is stated by the robot's own configs (not yet observed on the wire – the sample has no controllers).
* Stick semantics (`axis_map`): forward = +left y, lateral = −left x, turn = −right x,
  body height = +right y.
* `session_gate`: grips (`hand_trig`) of controllers `[0, 1]`, press 0.9 / release 0.5,
  release debounce 200 ms; both grips required when `external_device == GENIELINK` and pose
  mode is ABSOLUTE; with `UNKNOWN` it "conservatively degrades to single-grip take-over".

### How the robot handles loss of input (why the bridge sends no "release" frames)

* `session_gate.input_timeout_ms: 500` – comment: vr_button is a periodic full-state frame, on a
  stream interruption the level "freezes at the last frame's value"; after 500 ms without a frame
  the controller is treated as disconnected → "all released, leave teleop automatically". The
  comment calls this the only safeguard against "controller lost but robot still follows mocap".
* `hold_repeat.input_timeout_ms: 500` (per controller) – same reasoning.
* teleop_avatar: soft slowdown from 150–200 ms gaps, hard timeout at 1000 ms with a recovery
  motion; `control_mux.external_deadman_ttl_ms: 300`.

No config asks for an explicit release/zero frame; silence is the designed "lost input"
signal. The bridge therefore simply stops (default watchdog 100 ms, i.e. before any of the
robot's own thresholds) and publishes nothing further.

## 6. Still UNVERIFIED

* Whether `hal_remote` republishes UDP frames to `/remote/vr_data` unchanged. The existence of
  a separate `/remote/vr_data/raw` suggests the non-raw topic may be processed. The frame
  described above is verified for the UDP (pico-direct) format only.
* The expected `timestamp` unit. The sample only has the constant 4294967295.
* Controller data on the wire (none in the sample).
* Everything about the XRoboToolkit side (axes, handedness, joint-local frames, body joint order).
