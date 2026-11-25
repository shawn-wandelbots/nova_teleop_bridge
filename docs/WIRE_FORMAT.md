# NTB1 – intermediate UDP format, PC → robot

One datagram per tick (default 100 Hz), fixed binary layout, **little-endian**, no fragmentation
(max 832 bytes). Implemented in `teleop_common/wire.py` (`struct`, stdlib only). Default UDP
port **39500**; ports 25618, 25619, 15000 and 51000 are refused by both programs.

All poses are 7 × float32 `x, y, z, qx, qy, qz, qw` **in the XRoboToolkit frame, untransformed**.
The coordinate transform is applied on the robot (`config/transform.json`).

## Layout

| Offset | Size | Type | Field | Notes |
|---:|---:|---|---|---|
| 0 | 4 | char[4] | magic | `"NTB1"` |
| 4 | 1 | u8 | version | `1` |
| 5 | 1 | u8 | flags | see below; unknown bits → rejected |
| 6 | 1 | u8 | body_count | 24 if `BODY` flag set, else 0 |
| 7 | 1 | u8 | reserved | must be 0 |
| 8 | 4 | u32 | session | random per sender start |
| 12 | 4 | u32 | seq | starts at 1, +1 per datagram sent |
| 16 | 8 | u64 | t_send_ns | sender wall clock (`time.time_ns()`) |
| 24 | 8 | u64 | t_sdk_ns | `xrobotoolkit_sdk.get_time_stamp_ns()` |
| 32 | 28 | f32[7] | head pose | |
| 60 | 28 | f32[7] | left controller pose | |
| 88 | 28 | f32[7] | right controller pose | |
| 116 | 20 | controller | left controller inputs | |
| 136 | 20 | controller | right controller inputs | |
| 156 | 672 | f32[24][7] | body joints 0..23 | only if `BODY` flag set |
| 156 / 828 | 4 | u32 | crc32 | zlib CRC-32 of all preceding bytes |

Total size: **160 bytes** without body, **832 bytes** with body. Any other length is rejected.

`controller` (20 bytes): `f32 axis_x, f32 axis_y, f32 trigger, f32 grip, u8 buttons, u8[3] pad(0)`.
`buttons`: bit0 = stick click, bit1 = key_one (A right / X left), bit2 = key_two (B right / Y left);
other bits → rejected.

`flags`:

| Bit | Name | Meaning |
|---|---|---|
| 0x01 | HEAD | head pose valid |
| 0x02 | LEFT | left controller tracked; its pose and inputs are valid |
| 0x04 | RIGHT | right controller tracked; its pose and inputs are valid |
| 0x08 | BODY | 24 body joints follow |
| 0x80 | SYNTHETIC | produced by `pc/fake_xrt.py`, never by a headset |

Struct strings: header `<4sBBBBIIQQ`, poses `<21f`, controller `<4fB3x`, body `<168f`, crc `<I`.

## Receiver rules (robot bridge)

1. Source IP must equal `--sender-ip`, or – if not given – the first address that sent a datagram
   passing **all** of step 2 (garbage can never claim the lock).
2. Reject: shorter than the header (`short`), wrong magic, wrong version, unknown flag/button
   bits, wrong body_count, wrong length, wrong CRC.
3. Session: a different `session` is only accepted after `--resync-ms` (600) without any valid
   frame, i.e. a restarted sender is picked up after a pause, a second concurrent sender is not.
4. Sequence (serial-number arithmetic on u32): `seq` must be newer than the last one seen;
   duplicates and out-of-order datagrams are dropped (`stale_seq`). Gaps are counted, not fatal.
5. Content: NaN/inf, non-unit quaternions, |coordinate| > 10 m and implausible jumps are dropped;
   triggers/grips are clamped to 0..1, axes to −1..1. A controller whose flag is clear is left
   out of the output entirely (it is not reported as "released").
6. `SYNTHETIC` frames are accepted in `--dry-run`; with `--publish` they are rejected unless
   `--allow-synthetic` is given.

## Versioning

Any layout change bumps `version` (and, for incompatible changes, the magic). A receiver only
accepts the exact version it implements.
