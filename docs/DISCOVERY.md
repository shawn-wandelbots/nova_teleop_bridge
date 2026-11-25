# NTD1 – discovery of the teleop bridge on the local network

The robot's Wi-Fi address comes from DHCP and changes. The headset app ("Find robots" on its
setup screen) therefore looks for robots itself, with two independent mechanisms whose results
it merges by address:

1. **LLMNR name query** (RFC 4795) for the AgiBot head computer's default hostname `hdu`:
   a DNS-format query (QTYPE A, QCLASS IN) to UDP `224.0.0.252:5355`. The robot's operating
   system answers it; this repository is not involved, and it works when the bridge is not running.
2. **NTD1 probe**, defined here and answered by `robot/teleop_bridge.py`
   (implementation: `robot/discovery.py`). It tells the app that a bridge is really listening,
   on which data port, and whether it is in dry-run or publishing mode.

## NTD1 datagrams

UDP, default port **39501** (NTB1 data port + 1), little-endian, same header for both types:

| Offset | Size | Type | Field | Probe | Reply |
|---:|---:|---|---|---|---|
| 0 | 4 | char[4] | magic | `"NTD1"` | `"NTD1"` |
| 4 | 1 | u8 | version | `1` | `1` |
| 5 | 1 | u8 | type | `1` | `2` |
| 6 | 2 | u16 | length | reserved, must be `0` | number of JSON bytes that follow |
| 8 | 8 | bytes | nonce | random, chosen by the client | copied from the probe |
| 16 | n | UTF-8 | JSON | *(absent)* | one JSON object, n ≤ 480 |

A probe is **exactly 16 bytes**. A reply is 16 + n bytes and always **shorter than 512 bytes**.
Struct string of the header: `<4sBBH8s`.

Reply JSON (ASCII-escaped, no nesting):

```json
{"v":1,"product":"nova_teleop_bridge","bridge_version":"0.2.0","model":"A3_ULTRA_T3D0",
 "robot_id":"AB12","data_port":39500,"mode":"dry-run","sender_locked":false}
```

| Key | Type | Meaning |
|---|---|---|
| `v` | int | protocol version, equals the header version |
| `product` | string | always `nova_teleop_bridge`; clients ignore anything else |
| `bridge_version` | string | version of the bridge program |
| `model` | string | first line of `/agibot/info/model` (≤ 48 chars, `[A-Za-z0-9_.-]`), empty if unreadable |
| `robot_id` | string | **last 4 characters** of `/agibot/info/name`, to tell robots apart; the full name/serial is never sent |
| `data_port` | int | UDP port on which this bridge receives NTB1 datagrams |
| `mode` | string | `dry-run` (nothing is published) or `publish` (the robot follows the stream) |
| `sender_locked` | bool | a sender address is fixed (`--sender-ip`, or a first valid sender was seen) |

Clients must ignore unknown keys. New keys may be added without a version bump; any change to
the header or to the meaning of an existing key bumps `version`, and a peer only accepts the
exact version it implements.

## Client rules (headset app)

* Send the probe to the subnet broadcast address (interface address | ~netmask) and to
  `255.255.255.255`, and unicast to a known address for a connection check. Probes may be
  repeated (the app sends each twice, 0.7 s apart) and collects replies for about 2 s.
* Accept a reply only if magic, version, type and length are right, the nonce equals the one
  sent, and `product` is `nova_teleop_bridge`. The robot's address is the reply's source address.
* Use `data_port` only if it is a valid port and not one of 25618, 25619, 15000, 51000.
* A reply is information for the operator. It is never a reason to start sending.

## Responder rules (bridge)

* Enabled by default; `--no-discovery` turns it off, `--discovery-port N` moves it. If the port
  cannot be bound the bridge logs that and runs without discovery.
* Only a well-formed probe is answered. Wrong length, magic, version, type or a non-zero
  reserved field → **silent drop**. Replies are never answered, so two bridges cannot ping-pong.
* **One reply per probe**, sent to the probe's source address and port, < 512 bytes.
* Probes from source ports below 1024 or from 25618, 25619, 15000, 51000 are dropped, so a
  forged source cannot make the bridge send to a system service or to the robot's own teleop ports.
* Rate limit (token buckets): per source address 8 replies burst, then 4 per second; over all
  sources 40 burst, then 20 per second; at most 256 sources are remembered. Excess probes are
  dropped silently.
* A probe **never changes bridge state**: it cannot claim the sender lock, does not count as a
  frame and does not touch the watchdog. The responder only reads the mode and the lock state.
* Works identically in dry-run and publishing mode; it never imports ROS.

## Limits – read before relying on it

* **No authentication.** Anyone on the network segment can ask, and anyone can answer. The reply
  discloses model, a 4-character id, the data port and the mode – nothing else. A forged reply can
  at worst make the app *display* a wrong address; the operator still chooses the address and the
  robot-side safety rules (sender lock, validation, dry-run default) are unaffected.
* UDP source addresses can be forged, so the responder can be used as a reflector with a gain of
  at most 512/16 bytes per probe, capped by the rate limit above (≈ 10 kB/s in total).
* **Broadcast and multicast stay on one network segment** and are often filtered: guest Wi-Fi with
  client isolation, some mesh/enterprise access points and routed networks block both mechanisms.
  The unicast connection check and a manually entered address still work there.
* LLMNR may be disabled on a network or on the robot image, and **every** AgiBot head computer
  answers to `hdu`: with several robots on the Wi-Fi the list has several entries. Use `model`
  and `robot_id` from the NTD1 reply (bridge running) to tell them apart.
* The robot can have several addresses. The app lists the address the answer came from.

## Trying it by hand

```
python3 robot/teleop_bridge.py --listen-ip 127.0.0.1 --port 39610 --discovery-port 39611 &
python3 - <<'PY'
import os, socket
from robot import discovery as d
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(1)
nonce = os.urandom(8)
s.sendto(d.encode_probe(nonce), ("127.0.0.1", 39611))
print(d.decode_reply(s.recvfrom(2048)[0], nonce))
PY
```
