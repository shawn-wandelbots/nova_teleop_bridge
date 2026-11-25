# nova_teleop_bridge

Teleoperate an **AgiBot A3** humanoid from a **PICO 4** headset running the open-source
**XRoboToolkit** app, without the vendor's PICO app.

> **Status: untested on hardware.** Everything in this repository has only been run offline
> (unit tests and loopback UDP). The robot-side publisher has never been executed. Read
> "Verified vs. unverified" and the power-on checklist before going anywhere near `--publish`.

## Architecture

```
 PICO 4 headset                    Ubuntu PC                               A3 head board (Debian 12, ROS 2 Jazzy)
+----------------+   Wi-Fi   +---------------------------+   UDP :39500   +--------------------------------------+
| XRoboToolkit   | --------> | XRoboToolkit PC Service   |   "NTB1"       | robot/teleop_bridge.py               |
| client app     |           |   xrobotoolkit_sdk (py)   |   100 Hz       |  validate -> transform -> protobuf   |
+----------------+           | pc/xrt_adapter.py         | ------------>  |  watchdog, sender lock, limits       |
                             | pc/xrt_sender.py          |  raw SDK data  |          |  (only with --publish)     |
                             +---------------------------+                |          v                            |
                                                                          | /remote/vr_data   (XRFrameData)      |
                                                                          | /remote/vr_button (VRButtonChannel)  |
                                                                          +----------|---------------------------+
                                                                                     v  ROS 2 domain 232, BEST_EFFORT
                                                                      teleop_avatar (body retargeting, buttons)
                                                                      hal_remote trigger modules (grip dead-man, sticks)
```

* The PC sends **raw SDK values** (XRoboToolkit frame) in a small versioned datagram
  ([docs/WIRE_FORMAT.md](docs/WIRE_FORMAT.md)).
* The robot applies the coordinate transform from **`config/transform.json`** (the one file that
  holds every XRoboToolkit assumption: SDK function names, axis matrix, yaw, scale, offsets,
  controller codes, output options) and produces the robot's own protobuf messages with a
  hand-rolled encoder (the robot's Python has no `protobuf` package).
* The messages are wrapped in `ros2_plugin_proto/msg/RosMsgWrapper` (`serialization_type = "pb"`)
  and published on
  `/remote/vr_data/pb_3Aaimdk_2Eprotocol_2EXRFrameData` and
  `/remote/vr_button/pb_3Aaimdk_2Eprotocol_2EVRButtonChannel`.

```
config/transform.json        all XRoboToolkit-dependent settings (PROVISIONAL)
teleop_common/wire.py        NTB1 datagram pack/unpack            (PC + robot)
teleop_common/transform.py   quaternion maths, config-driven frame transform
pc/xrt_adapter.py            the only code that calls xrobotoolkit_sdk
pc/xrt_sender.py             PC main program
pc/fake_xrt.py               fake SDK (synthetic figure, or replay of recorded A3 frames)
robot/a3_proto.py            protobuf codec for XRFrameData / VRButtonChannel
robot/bridge_core.py         validation, safety logic, conversion (no I/O)
robot/teleop_bridge.py       robot main program (UDP in, dry-run / ROS out)
robot/discovery.py           NTD1 discovery responder (lets the headset app find the bridge)
robot/recording.py           record / capture file formats
tools/capture_udp.py         capture raw datagrams on loopback (used with pico_mock_player)
tools/analyze_frames.py      prints the numbers in docs/A3_FRAME_ANALYSIS.md
tests/                       unit + loopback tests, two small excerpts of the robot's PICO sample
docs/                        A3_FRAME_ANALYSIS.md, WIRE_FORMAT.md, DISCOVERY.md
```

Dependencies: Python standard library only (3.8+ on the PC, 3.11 on the robot). The PC
additionally needs `xrobotoolkit_sdk`; the robot needs `rclpy` + `ros2_plugin_proto` **only**
when `--publish` is given (they are imported lazily, tests never load them).

## Running

### Tests (anywhere, offline)

```
python3 -m unittest            # from the repository root
```

Uses loopback UDP on private ports ≥ 39200 only (never the default ports 39500/39501, the
discovery tests use ≥ 39600); publishes nothing; does not need ROS.

### Robot side – dry run (the default)

```
python3 robot/teleop_bridge.py --port 39500 --sender-ip <PC ip> --record /tmp/teleop.rec
```

`--dry-run` is the default: datagrams are received, validated, transformed and encoded, rate
statistics are printed every 5 s, `--record` stores every encoded output frame, and **nothing is
published – ROS is not even imported**. Stop with Ctrl-C / SIGTERM.

Inspect a record:

```
python3 - <<'PY'
from robot.recording import read_record
from robot.a3_proto import decode_xr_frame, decode_vr_buttons, BODY_JOINT_NAMES
recs = read_record("/tmp/teleop.rec")
f = decode_xr_frame(next(r[2] for r in recs if r[0] == 1))
for j in f.body: print(BODY_JOINT_NAMES[j.code], j.pos, j.quat)
print(decode_vr_buttons(next(r[2] for r in recs if r[0] == 2)))
PY
```

### Discovery (finding the robot from the headset)

The robot's Wi-Fi address changes with DHCP, so the headset app (Nova Teleop setup screen,
"Find robots") searches for it instead of relying on a typed-in address:

* The bridge answers **NTD1 discovery probes** on UDP **39501** (`--discovery-port N`;
  `--no-discovery` to switch it off) in both dry-run and publishing mode. The reply carries the
  robot model, the last 4 characters of the robot's name, the NTB1 data port, the mode
  (`dry-run` / `publish`) and whether a sender is locked in. Protocol and rules:
  [docs/DISCOVERY.md](docs/DISCOVERY.md).
* Independently of the bridge, the app sends an LLMNR query for the head computer's default
  hostname `hdu`, which the robot's operating system answers. That finds the robot even when the
  bridge is not running.

The responder is read-only: it answers only well-formed probes, one small reply (< 512 bytes)
per probe, rate-limited per source, and a probe can never claim the sender lock or change any
other bridge state. If port 39501 is taken the bridge logs it and runs without discovery.

Limits: broadcast and multicast do not cross routers and are blocked by Wi-Fi client isolation
(guest networks); LLMNR may be disabled; every AgiBot head computer answers to `hdu`, so several
robots give several entries (tell them apart by `model` / `robot_id`); replies are not
authenticated. A manually entered address always works.

The app sends from the **headset's** address: `--sender-ip` must be that address (shown on the
app's setup screen), or be left out to lock onto the first valid sender.

### Robot side – live publishing

**Has never been run. Do not run it before completing the checklist below.**

```
source <robot ROS 2 env: /opt/ros/jazzy, aimrt_protocol_ros2_package on PYTHONPATH/LD_LIBRARY_PATH,
        ROS_DOMAIN_ID=232, the robot's FASTRTPS_DEFAULT_PROFILES_FILE>
python3 robot/teleop_bridge.py --port 39500 --sender-ip <PC ip> --publish
```

With `--publish` the bridge refuses to start if another node already publishes either topic
(override: `--allow-other-publishers`) and refuses frames from the fake SDK (override:
`--allow-synthetic`, never on a live robot).

### PC side

```
python3 pc/xrt_sender.py --robot-ip <robot ip> --port 39500             # real headset
python3 pc/xrt_sender.py --robot-ip <robot ip> --port 39500 --fake      # synthetic figure, no headset
python3 pc/xrt_sender.py --robot-ip 127.0.0.1 --fake --fake-replay tests/data/pico_sample_contiguous.cap
```

The fake SDK is deliberately harmless (grips ≤ 0.3, stick clicks never pressed) and marks its
datagrams as synthetic.

## Safety behaviour

Robot bridge:

* **Dry-run by default**; publishing needs the explicit `--publish`.
* **One sender**: `--sender-ip`, or the first address that sends a fully valid datagram.
* **Strict datagram checks**: magic, version, exact length, flag bits, CRC-32; duplicate and
  out-of-order sequence numbers are dropped; a second sender session is ignored.
* **Content checks**: NaN/inf, non-unit quaternions, coordinates beyond `--max-abs-position`
  (10 m) and implausible jumps (more than `--min-jump` 0.25 m *and* faster than `--max-speed`
  15 m/s since the last accepted frame) are dropped. A persisting jump is only accepted after
  `--resync-ms` (600 ms) of silence, by which time the robot has ended the session on its own.
* **Clamping**: trigger and grip to 0..1, stick axes to −1..1. Buttons are passed through
  exactly as received – never synthesised, never suppressed (this includes the both-sticks
  emergency-stop chord). An untracked controller is omitted, not reported as "released".
* **Watchdog**: no valid frame for `--watchdog-ms` (100 ms) → output stops. After a stop,
  `--resume-frames` (3) consecutive valid frames are needed before output resumes. Stale data is
  never repeated; a backlog of queued datagrams is collapsed to the newest one.
* **On stop/exit nothing more is published** – no zero or "release" frames. The robot's configs
  define silence as the lost-input signal (500 ms controller timeout in `hal_remote`'s
  `session_gate`, 150/200 ms slowdown and 1000 ms hard timeout in teleop_avatar; see
  [docs/A3_FRAME_ANALYSIS.md](docs/A3_FRAME_ANALYSIS.md) §5).
* Never binds or sends to the robot's own teleop ports (25618, 25619, 15000, 51000), never calls
  a robot service. The discovery responder also refuses to reply to those ports and to ports
  below 1024, and nothing it receives reaches the bridge core.

PC sender: sends only what the SDK returned; skips a tick on any SDK error or NaN; goes silent
when the SDK timestamp stops advancing for `--stale-ms` (100 ms); sends nothing on exit.

**None of this replaces the hardware emergency stop.**

## Verified vs. unverified

VERIFIED (offline, on the robot's head board)

* Protobuf codec: decode → re-encode is byte-identical for all 2921 frames of the robot's own
  PICO sample (replayed with `pico_mock_player` to a loopback port).
* Target frame of that sample: right-handed, Z-up, metres, origin near head height (floor at
  z ≈ −1.5), global joint quaternions `x,y,z,w`, joint-local +x forward / +y left / +z up,
  arbitrary heading. Details and numbers: [docs/A3_FRAME_ANALYSIS.md](docs/A3_FRAME_ANALYSIS.md).
* Topic name/type/QoS of `/remote/vr_data/pb_3Aaimdk_2Eprotocol_2EXRFrameData`
  (`ros2_plugin_proto/msg/RosMsgWrapper`, BEST_EFFORT, VOLATILE) as seen with
  `ros2 topic list -t` / `ros2 topic info -v` (one subscriber, the bag recorder; no publisher).
* teleop_avatar's default input link is `pico-ros2` on `/remote/vr_data`.
* Maths, wire format, watchdog, sequence handling and the loopback pipeline (unit tests).

STATED BY ROBOT CONFIGS, NOT OBSERVED ON THE WIRE

* `controller_code_name`: 0 = left, 1 = right; `key_one` = X/A, `key_two` = Y/B
  (hal_remote trigger configs). Configurable in `config/transform.json` → `controllers`.
* Stick semantics: forward = +left y, lateral = −left x, turn = −right x, height = +right y.

UNVERIFIED – treat as wrong until proven on hardware

* **The whole XRoboToolkit side**: SDK function names and return shapes, axis convention and
  handedness, the joint-local frame of body joints, body joint order, stick axis signs, what
  `get_time_stamp_ns()` does when the headset stalls. The default `transform` is PROVISIONAL.
* **Whether `/remote/vr_data` uses the same frame as the sample.** The sample is the UDP
  (pico-direct) format; `hal_remote` also publishes `/remote/vr_data/raw`, so it may process
  frames before publishing `/remote/vr_data`.
* **`/remote/vr_button` was not visible** in the ROS graph when this was written (`hal_remote`
  was not running), so its exact topic name/type string and QoS are inferred from the naming
  rule and the configs, not observed.
* **Parallel publisher conflict**: `hal_remote` is the robot's normal publisher of both topics
  (fed by UDP 25618/15000). If it is running and a PICO/app talks to it, two sources would
  interleave. The bridge checks the publisher count at start-up and refuses, but that check is
  itself untested, and an idle `hal_remote` may still register publishers.
* **Bypassing `hal_remote` may disable button events.** `hal_remote` also derives
  `/remote/button_edge` and `/event_trigger/button_event` (combos such as X+Y+A+B calibration
  and the both-sticks emergency stop used by its trigger modules). This bridge does not publish
  those. teleop_avatar has its own bindings on the frame's `controller_raw_datas`
  (`pico.xyab`, `pico.dual_axis_click`), which may or may not be active on the `pico-ros2` link.
  **Do not rely on the stick-click e-stop.**
* **Whether head/controller poses must also be sent as joints.** The sample has empty
  `left_hand`/`right_hand` lists, so by default only the 24 body joints are sent. An
  experimental switch (`output.controller_pose_as_hand_joint`) exists; its joint code is a guess.
* **`timestamp` unit.** The sample has a constant 4294967295. Default is Unix milliseconds
  (`output.timestamp_mode`).
* **`external_device`.** Default `GENIELINK` (strictest dead-man: both grips in upper-body
  absolute mode). Other side effects of this tag are unknown.
* Explicit-zero protobuf encoding on the ROS topic (matches the PICO byte layout; the canonical
  proto3 form is available with `output.explicit_defaults: false`).
* The robot's installed teleop profile is named `a3_lite`.

## First power-on checklist

Two people. One does nothing but hold the emergency stop.

1. **Robot in damping (or on the hoist / e-stopped), hardware e-stop in hand and tested.**
   Clear 2 m around the robot. Agree on a spoken "stop" word.
2. Confirm with the robot's owner which teleop services are running (`hal_remote`,
   teleop_avatar) and that nobody else is connected with a PICO or the phone app.
3. PC and robot on the same network; run `python3 -m unittest` on both.
4. Robot: start the bridge in **dry-run** with `--record`. PC: start the sender with `--fake`.
   Check the statistics line: ~100 Hz valid, no rejects, sender IP as expected. Stop the sender
   and confirm the bridge logs `WATCHDOG` and the would-publish rate drops to 0.
5. Real headset, still **dry-run**: stand in a T-pose facing forward, record 10 s, then decode
   the record and check against [docs/A3_FRAME_ANALYSIS.md](docs/A3_FRAME_ANALYSIS.md):
   z is up (head ≈ 1.4–1.5 m above the feet), left wrist to the left of the right wrist with
   forward × left = up, pelvis quaternion ≈ pure yaw, quaternion's local +x points where you
   face, metres. Raise the left hand only, step forward, turn left – each must show up as the
   expected axis. Fix `config/transform.json` until it does.
6. Still dry-run: press every button, trigger, grip and stick direction one at a time and check
   the decoded `VRButtonChannel` (controller 0 = left, key_one = X/A, key_two = Y/B, ranges
   0..1 / −1..1). Check that an idle controller shows grip ≈ 0.
7. Read-only on the robot: `ros2 topic info -v` on both topics. There must be **no other
   publisher**. Note the subscribers.
8. Only now, robot still in damping: start the bridge with `--publish`, **no grips pressed**,
   hands off the sticks. Watch the robot's teleop logs/status for accepted frames, calibration
   state and errors. Stop the bridge; confirm the robot reports input loss.
9. Calibration and take-over only with the robot's normal procedure and its owner present, on
   the hoist if available. First take-over: slow, small arm motions only; release the grips
   to end the session; e-stop at the first unexpected motion.
10. Afterwards: stop the bridge first, then the sender; write down what was verified and update
    the lists above.

## Notes

* **Test fixtures.** `tests/data/*.cap` are short excerpts (151 frames, body joint poses only)
  of the vendor's sample recording shipped on the robot. They are NOT kept in git (`*.cap` is
  ignored) because redistribution has not been cleared. Without them the five tests that need
  them are skipped; everything else runs. On the robot they can be regenerated from the vendor
  sample with `tools/capture_udp.py` and `pico_mock_player` on a private loopback port, and the
  full-sample golden test is enabled with `NTB_FULL_SAMPLE=<full .cap>`.
* An alternative integration – emitting the PICO UDP format straight to `hal_remote` so that
  it produces all derived topics itself – was deliberately not implemented: it would mean
  sending to the robot's live teleop port.
