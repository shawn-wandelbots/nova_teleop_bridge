#!/usr/bin/env python3
"""Robot-side teleop bridge: NTB1 UDP datagrams -> /remote/vr_data + /remote/vr_button.

DEFAULT IS --dry-run: nothing is published and ROS is not even imported.  Real publishing
needs the explicit --publish flag AND the robot's ROS 2 environment (domain 232, see README).

    python3 robot/teleop_bridge.py --port 39500 --record /tmp/out.rec          # dry run
    python3 robot/teleop_bridge.py --port 39500 --sender-ip 192.168.1.50 --publish

Unless --no-discovery is given it also answers NTD1 discovery probes on --discovery-port
(default 39501) so the headset app can find it; see docs/DISCOVERY.md.

Safety behaviour (details in README.md):
  * one sender only (--sender-ip, or the first address that sends a fully valid datagram)
  * wrong magic/version/length/CRC, duplicate and out-of-order datagrams are dropped
  * NaN/inf, non-unit quaternions, out-of-range and implausibly jumping positions are dropped
  * triggers/grips are clamped to 0..1 and stick axes to -1..1; buttons are never synthesised
  * watchdog: no valid frame for --watchdog-ms -> publishing stops; NOTHING is sent after that
    (no stale repeats, no invented "release" frames; the robot handles input loss itself)
"""
from __future__ import annotations

import argparse
import os
import select
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import a3_proto, discovery  # noqa: E402
from robot.bridge_core import BridgeCore, Limits  # noqa: E402
from robot.recording import KIND_VR_BUTTON, KIND_XR_FRAME, RecordWriter  # noqa: E402
from teleop_common.transform import load_config  # noqa: E402

VR_DATA_TOPIC = a3_proto.ros_topic_name("/remote/vr_data", a3_proto.XR_FRAME_TYPE)
VR_BUTTON_TOPIC = a3_proto.ros_topic_name("/remote/vr_button", a3_proto.VR_BUTTON_TYPE)
# The robot's own live teleop ports. This program never binds or sends to them.
FORBIDDEN_PORTS = (25618, 25619, 15000, 51000)
DEFAULT_PORT = 39500
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "config", "transform.json")


def log(msg: str) -> None:
    print("[bridge %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


class DryRunPublisher:
    """Counts what would have been published. Touches neither ROS nor the network."""
    dry_run = True

    def __init__(self):
        self.count = 0

    def publish(self, xr_frame: bytes, vr_button: bytes) -> None:
        self.count += 1

    def close(self) -> None:
        pass


class RosPublisher:
    """Real publisher. Only constructed with --publish; rclpy is imported here and nowhere else."""
    dry_run = False

    def __init__(self, allow_other_publishers: bool, discovery_wait_s: float = 2.0):
        import rclpy  # noqa: PLC0415  (lazy on purpose)
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from ros2_plugin_proto.msg import RosMsgWrapper

        self._rclpy = rclpy
        self._msg_type = RosMsgWrapper
        self.count = 0
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node("nova_teleop_bridge")
        # Give DDS discovery a moment, then refuse to become a second publisher on the same
        # topics (the robot's hal_remote publishes them when a PICO talks to it directly).
        deadline = time.monotonic() + discovery_wait_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        for topic in (VR_DATA_TOPIC, VR_BUTTON_TOPIC):
            others = self._node.count_publishers(topic)
            if others and not allow_other_publishers:
                self._node.destroy_node()
                rclpy.shutdown()
                raise SystemExit("REFUSING to publish: %d other publisher(s) already on %s "
                                 "(pass --allow-other-publishers to override)" % (others, topic))
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self._pub_data = self._node.create_publisher(RosMsgWrapper, VR_DATA_TOPIC, qos)
        self._pub_btn = self._node.create_publisher(RosMsgWrapper, VR_BUTTON_TOPIC, qos)

    def _wrap(self, payload: bytes):
        m = self._msg_type()
        m.serialization_type = "pb"
        m.context = []
        m.data = [bytes([x]) for x in payload]   # byte[] = list of 1-byte bytes objects
        return m

    def publish(self, xr_frame: bytes, vr_button: bytes) -> None:
        self._pub_data.publish(self._wrap(xr_frame))
        self._pub_btn.publish(self._wrap(vr_button))
        self.count += 1

    def close(self) -> None:
        # Deliberately no final "zero"/"release" message: stopping means silence.
        self._node.destroy_node()
        self._rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", dest="publish", action="store_false",
                      help="decode/validate/encode only, publish nothing (DEFAULT)")
    mode.add_argument("--publish", dest="publish", action="store_true",
                      help="REALLY publish to the robot's ROS 2 topics")
    ap.set_defaults(publish=False)
    ap.add_argument("--listen-ip", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--sender-ip", default=None, help="only accept this source IP (default: first valid sender)")
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--record", default=None, metavar="FILE", help="save every encoded output frame")
    ap.add_argument("--stats-interval", type=float, default=5.0, help="seconds between statistics lines (0 = off)")
    ap.add_argument("--duration", type=float, default=0.0, help="exit after this many seconds (0 = run until signalled)")
    d = Limits()
    ap.add_argument("--watchdog-ms", type=float, default=d.watchdog_ms)
    ap.add_argument("--resume-frames", type=int, default=d.resume_frames)
    ap.add_argument("--resync-ms", type=float, default=d.resync_ms)
    ap.add_argument("--max-abs-position", type=float, default=d.max_abs_position_m)
    ap.add_argument("--max-speed", type=float, default=d.max_speed_mps, help="per-joint speed limit, m/s")
    ap.add_argument("--min-jump", type=float, default=d.min_jump_m, help="jump (m) that is always tolerated")
    ap.add_argument("--no-discovery", dest="discovery", action="store_false",
                    help="do not answer NTD1 discovery probes (docs/DISCOVERY.md)")
    ap.add_argument("--discovery-port", type=int, default=discovery.DEFAULT_DISCOVERY_PORT,
                    help="UDP port of the discovery responder (default %(default)s)")
    ap.add_argument("--allow-no-body", action="store_true", help="accept frames without body tracking")
    ap.add_argument("--allow-synthetic", action="store_true",
                    help="with --publish: also publish frames produced by the fake SDK (never do this on a live robot)")
    ap.add_argument("--allow-other-publishers", action="store_true",
                    help="with --publish: publish even if another node already publishes these topics")
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    a = ap.parse_args(argv)
    if a.port in FORBIDDEN_PORTS:
        ap.error("port %d belongs to the robot's own teleop stack" % a.port)
    if a.discovery:
        if a.discovery_port in FORBIDDEN_PORTS:
            ap.error("discovery port %d belongs to the robot's own teleop stack" % a.discovery_port)
        if not 1024 <= a.discovery_port <= 65535 or a.discovery_port == a.port:
            ap.error("--discovery-port must be 1024..65535 and differ from --port")
    if a.resync_ms <= a.watchdog_ms:
        ap.error("--resync-ms must be larger than --watchdog-ms")
    limits = Limits(watchdog_ms=a.watchdog_ms, resume_frames=max(1, a.resume_frames), resync_ms=a.resync_ms,
                    max_abs_position_m=a.max_abs_position, max_speed_mps=a.max_speed, min_jump_m=a.min_jump,
                    require_body=not a.allow_no_body,
                    allow_synthetic=(not a.publish) or a.allow_synthetic)
    cfg = load_config(a.config)
    core = BridgeCore(cfg, limits, sender_ip=a.sender_ip, log=log)

    stop = {"flag": False, "sig": None}

    def _on_signal(signum, _frame):
        stop["flag"], stop["sig"] = True, signum

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((a.listen_ip, a.port))
    sock.setblocking(False)

    recorder = RecordWriter(a.record) if a.record else None
    publisher = RosPublisher(a.allow_other_publishers) if a.publish else DryRunPublisher()

    # Discovery responder (docs/DISCOVERY.md). Read-only: it reports mode and lock state and
    # never touches the bridge core. A port that cannot be bound only disables discovery.
    disco = None
    if a.discovery:
        model, robot_id = discovery.read_robot_identity()
        responder = discovery.DiscoveryResponder(
            a.port, lambda: (discovery.MODE_PUBLISH if a.publish else discovery.MODE_DRY_RUN,
                             core.sender_ip is not None),
            model=model, robot_id=robot_id)
        try:
            disco = discovery.DiscoveryServer(responder, a.listen_ip, a.discovery_port)
        except OSError as e:
            log("discovery: cannot bind UDP %s:%d (%s) -> discovery disabled, teleop unaffected"
                % (a.listen_ip, a.discovery_port, e))
    read_socks = [sock] + ([disco] if disco else [])

    log("mode=%s listen=%s:%d sender=%s config=%s (%s)" % (
        "PUBLISH" if a.publish else "DRY-RUN", a.listen_ip, a.port, a.sender_ip or "first-seen",
        a.config, cfg.get("status", "?")))
    log("topics: %s , %s" % (VR_DATA_TOPIC, VR_BUTTON_TOPIC))
    log("discovery: %s" % ("answering NTD1 probes on UDP %d" % disco.port if disco else "off"))
    if a.publish:
        log("*** LIVE: publishing to the robot. Keep the e-stop within reach. ***")

    t_start = time.monotonic()
    t_stats = t_start
    last_stats = dict(core.stats)
    last_pub = 0
    lat_ms = []
    try:
        while not stop["flag"]:
            now = time.monotonic()
            if a.duration and now - t_start >= a.duration:
                break
            try:
                readable, _, _ = select.select(read_socks, [], [], 0.02)
            except InterruptedError:
                continue
            now = time.monotonic()
            newest = None
            if disco is not None and disco in readable:
                disco.service(now)
            if sock in readable:
                # Drain the socket; validate everything, but only pass on the newest valid frame
                # so a backlog (e.g. after a scheduling hiccup) is never replayed late.
                while True:
                    try:
                        data, addr = sock.recvfrom(65535)
                    except (BlockingIOError, InterruptedError):
                        break
                    out = core.handle(data, addr[0], now)
                    if out is not None:
                        if newest is not None:
                            core.stats["superseded"] += 1
                        newest = out
            core.tick(now)
            if newest is not None and core.active and not stop["flag"]:
                publisher.publish(newest.xr_frame, newest.vr_button)
                core.stats["out"] += 1
                t_ns = time.time_ns()
                lat_ms.append((t_ns - newest.t_send_ns) / 1e6)
                if recorder:
                    recorder.write(KIND_XR_FRAME, t_ns, newest.xr_frame)
                    recorder.write(KIND_VR_BUTTON, t_ns, newest.vr_button)
            if a.stats_interval and now - t_stats >= a.stats_interval:
                dt = now - t_stats
                s = core.stats
                rej = {k[7:]: v - last_stats.get(k, 0) for k, v in s.items()
                       if k.startswith("reject_") and v != last_stats.get(k, 0)}
                lat = ("%.1f/%.1f" % (sum(lat_ms) / len(lat_ms), max(lat_ms))) if lat_ms else "-"
                log("%s rx %.1f Hz | valid %.1f Hz | %s %.1f Hz | rejects %s | seq gaps %d | watchdog trips %d | "
                    "sender->robot clock delta avg/max ms %s | state %s" % (
                        "PUBLISH" if a.publish else "DRY-RUN",
                        (s["rx"] - last_stats.get("rx", 0)) / dt,
                        (s["valid"] - last_stats.get("valid", 0)) / dt,
                        "published" if a.publish else "would-publish", (publisher.count - last_pub) / dt,
                        rej or "none", s["seq_gaps"], s["watchdog_trips"], lat,
                        "ACTIVE" if core.active else "STOPPED"))
                t_stats, last_stats, last_pub, lat_ms = now, dict(s), publisher.count, []
    finally:
        # Stop = silence. Nothing further is published from here on.
        sock.close()
        if disco is not None:
            disco.close()
        if recorder:
            recorder.close()
        total = publisher.count
        publisher.close()
        s = core.stats
        log("exit (%s): rx=%d valid=%d %s=%d rejects=%s watchdog_trips=%d" % (
            signal.Signals(stop["sig"]).name if stop["sig"] else "done", s["rx"], s["valid"],
            "published" if a.publish else "would-publish", total,
            {k[7:]: v for k, v in s.items() if k.startswith("reject_")} or "none", s["watchdog_trips"]))
        if disco is not None:
            log("discovery: %s" % (dict(disco.responder.stats) or "no probes"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
