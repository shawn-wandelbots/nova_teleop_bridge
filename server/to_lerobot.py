#!/usr/bin/env python3
"""Builds a LeRobot dataset (format v2.1) from episodes stored by ingest_server.py.

    python3 server/to_lerobot.py --episodes ~/nova_teleop_data/episodes --out ~/nova_teleop_data/lerobot/my_task

The stored episodes are never changed: they stay the accurate raw record.  The dataset is a
derived copy and can be rebuilt at any time with other choices (rate, joints, video quality).

What the conversion has to decide, because a LeRobot dataset is one table with one row per
video picture while the robot's streams all run on their own clocks:

  * Time grid.  Rows are spaced 1/fps apart.  The grid starts when every camera has delivered
    its first complete picture (key frame) and joint data exists, and ends when the first
    stream ends.
  * Lost pictures.  When the recording lost a picture, the pictures after it cannot be decoded
    correctly until the camera's next key frame.  Rows in such a stretch repeat the last good
    picture; an episode with a stretch longer than --max-held-s is left out.
  * Video.  For every row, the picture of each camera closest in time is used.  The camera
    streams are decoded and encoded again (they cannot be cut picture-exact otherwise), so
    the dataset video is slightly lossy; the original stream stays in the raw episode.
  * State.  The newest joint reading at or before the row's time (never a later one).
  * Action.  The newest joint COMMAND at or before the row's time.  Where an episode has no
    commands for a joint group, the next row's state is used instead and the report says so.

Needs: ffmpeg and ffprobe on the PATH, and the Python packages numpy, pyarrow, mcap and
mcap-ros2-support (see server/requirements.txt).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

CODEBASE_VERSION = "v2.1"
CHUNK_SIZE = 1000
VIDEO_TYPE = "foxglove_msgs/msg/CompressedVideo"
JOINT_TYPE = "sensor_msgs/msg/JointState"
STATE_SUFFIX, COMMAND_SUFFIX = "_joint_state", "_joint_command"


# ------------------------------------------------------------------------------------------
# Pure helpers (tested without ffmpeg or any recording)
# ------------------------------------------------------------------------------------------
def h265_nal_types(data):
    """NAL unit types found in an Annex-B H.265 chunk."""
    out, i, n = [], 0, len(data)
    while True:
        i = data.find(b"\x00\x00\x01", i)
        if i < 0 or i + 3 >= n:
            return out
        out.append((data[i + 3] >> 1) & 0x3F)
        i += 3


def is_keyframe(data):
    """True if decoding can start here: parameter sets plus a random-access picture."""
    types = h265_nal_types(data)
    return 33 in types and any(16 <= t <= 23 for t in types)


def first_keyframe(chunks):
    for i, c in enumerate(chunks):
        if is_keyframe(c):
            return i
    return None


def pick_clock(stamp, log, tolerance=0.5):
    """Each message has the sender's own stamp and the time it was logged on the robot.

    The sender's stamp is the better clock (no network delay in it) but only usable when the
    sender's clock agrees with the recorder's.  Returns (times, "stamp" | "log")."""
    stamp, log = np.asarray(stamp, dtype=np.float64), np.asarray(log, dtype=np.float64)
    if len(stamp) and np.all(stamp > 1e6) and abs(float(np.median(stamp - log))) <= tolerance \
            and np.all(np.diff(stamp) >= 0):
        return stamp, "stamp"
    return log, "log"


def nearest_indices(times, ticks):
    """For each tick, the index of the sample closest in time.  ``times`` must be sorted."""
    times, ticks = np.asarray(times), np.asarray(ticks)
    right = np.clip(np.searchsorted(times, ticks), 0, len(times) - 1)
    left = np.clip(right - 1, 0, len(times) - 1)
    return np.where(np.abs(times[left] - ticks) <= np.abs(times[right] - ticks), left, right)


def hold_indices(times, ticks):
    """For each tick, the index of the newest sample at or before it (the first sample for
    ticks earlier than everything)."""
    return np.clip(np.searchsorted(np.asarray(times), np.asarray(ticks), side="right") - 1, 0, len(times) - 1)


def damaged_pictures(times, keyframes, fps):
    """Marks pictures that cannot be decoded correctly.

    The video is a chain: every picture is stored as the difference to the one before, until
    the next key frame starts a new chain.  When a picture was lost in the recording, all
    following pictures up to the next key frame are built on something missing."""
    times, keyframes = np.asarray(times, dtype=np.float64), np.asarray(keyframes, dtype=bool)
    bad = np.zeros(len(times), dtype=bool)
    broken = False
    for i in range(len(times)):
        if keyframes[i]:
            broken = False
        elif i and round((times[i] - times[i - 1]) * fps) > 1:
            broken = True
        bad[i] = broken
    return bad


def hold_last_good(index, bad):
    """Replaces every damaged picture in ``index`` by the last undamaged one before it.
    Returns (new index, longest run of rows that got a replaced picture)."""
    index, bad = np.asarray(index), np.asarray(bad, dtype=bool)
    good = np.where(~bad, np.arange(len(bad)), -1)
    last_good = np.maximum.accumulate(good)
    held = bad[index]
    out = np.where(held, last_good[index], index)
    longest = run = 0
    for h in held:
        run = run + 1 if h else 0
        longest = max(longest, run)
    return out, int(held.sum()), longest


def make_ticks(t_start, t_end, fps):
    n = int(np.floor((t_end - t_start) * fps + 1e-6)) + 1
    return t_start + np.arange(max(n, 0)) / float(fps)


def feature_stats(a):
    a = np.asarray(a, dtype=np.float64)
    if a.ndim == 1:
        a = a[:, None]
    return {"min": a.min(0).tolist(), "max": a.max(0).tolist(), "mean": a.mean(0).tolist(),
            "std": a.std(0).tolist(), "count": [int(a.shape[0])]}


def camera_key(topic):
    """'/hal/chest_front_d457_camera/stream' -> 'chest_front_d457'"""
    name = topic.strip("/").split("/")[-2] if topic.count("/") >= 2 else topic.strip("/")
    return name[:-len("_camera")] if name.endswith("_camera") else name


def group_of(topic, suffix):
    """'/motion/control/arm_joint_state' -> 'arm'"""
    last = topic.rsplit("/", 1)[-1]
    return last[:-len(suffix)] if last.endswith(suffix) else None


# ------------------------------------------------------------------------------------------
# Reading one episode
# ------------------------------------------------------------------------------------------
class Skip(Exception):
    """This episode cannot go into the dataset; the message says why."""


def read_episode(ep_dir):
    """Returns (info, videos, joints): videos[topic] = dict(stamp, log, data[]),
    joints[topic] = dict(stamp, log, names, position, velocity, effort)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    with open(os.path.join(ep_dir, "episode.json")) as fh:
        info = json.load(fh)
    bag_dir = os.path.join(ep_dir, "bag")
    files = sorted(f for f in os.listdir(bag_dir) if f.endswith(".mcap"))
    if not files:
        raise Skip("no recording file")
    videos, joints = {}, {}
    for f in files:
        with open(os.path.join(bag_dir, f), "rb") as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            wanted = [c.topic for c in reader.get_summary().channels.values()
                      if reader.get_summary().schemas[c.schema_id].name in (VIDEO_TYPE, JOINT_TYPE)]
            for schema, channel, message, msg in reader.iter_decoded_messages(topics=wanted):
                log = message.log_time * 1e-9
                if schema.name == VIDEO_TYPE:
                    v = videos.setdefault(channel.topic, {"stamp": [], "log": [], "data": [], "format": msg.format})
                    v["stamp"].append(msg.timestamp.sec + msg.timestamp.nanosec * 1e-9)
                    v["log"].append(log)
                    v["data"].append(bytes(msg.data))
                else:
                    j = joints.setdefault(channel.topic, {"stamp": [], "log": [], "names": list(msg.name),
                                                          "position": [], "velocity": [], "effort": []})
                    if list(msg.name) != j["names"]:
                        raise Skip("joint names change inside %s" % channel.topic)
                    n = len(j["names"])
                    j["stamp"].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                    j["log"].append(log)
                    for k in ("position", "velocity", "effort"):
                        vals = list(getattr(msg, k))
                        j[k].append(vals if len(vals) == n else [float("nan")] * n)
    return info, videos, joints


def align_episode(videos, joints, fps, state_groups, cameras=None, max_held_s=1.1):
    """Chooses the time grid and, for every row, which picture and which joint sample to use.
    Returns a dict with everything write_episode needs, plus notes for the report."""
    notes = []
    cams = {}
    for topic, v in sorted(videos.items()):
        if cameras and camera_key(topic) not in cameras:
            continue
        if v["format"] != "h265":
            raise Skip("%s is %r, only h265 is handled" % (topic, v["format"]))
        k0 = first_keyframe(v["data"])
        if k0 is None:
            raise Skip("%s has no key frame, so no picture can be decoded" % topic)
        t, clock = pick_clock(v["stamp"], v["log"])
        cams[camera_key(topic)] = {"topic": topic, "t": t[k0:], "data": v["data"][k0:], "clock": clock,
                                   "bad": damaged_pictures(t[k0:], [is_keyframe(c) for c in v["data"][k0:]], fps),
                                   "skipped_before_keyframe": k0, "t_first_message": float(t[0])}
    if cameras and sorted(cams) != sorted(cameras):
        raise Skip("camera(s) not in the recording: %s" % ", ".join(sorted(set(cameras) - set(cams))))
    if not cams:
        raise Skip("no camera stream")

    def series(suffix):
        out = {}
        for topic, j in joints.items():
            g = group_of(topic, suffix)
            if g and len(j["log"]):
                t, clock = pick_clock(j["stamp"], j["log"])
                out[g] = dict(j, t=t, clock=clock, topic=topic,
                              **{k: np.asarray(j[k], dtype=np.float64) for k in ("position", "velocity", "effort")})
        return out
    states, commands = series(STATE_SUFFIX), series(COMMAND_SUFFIX)
    missing = [g for g in state_groups if g not in states]
    if missing:
        raise Skip("no joint state recorded for: %s" % ", ".join(missing))

    t_start = max([c["t"][0] for c in cams.values()] + [states[g]["t"][0] for g in state_groups])
    t_end = min([c["t"][-1] for c in cams.values()] + [states[g]["t"][-1] for g in state_groups])
    ticks = make_ticks(t_start, t_end, fps)
    if len(ticks) < 2:
        raise Skip("the streams overlap for less than two rows")

    for key, c in cams.items():
        idx = nearest_indices(c["t"], ticks)
        idx, c["rows_held"], longest = hold_last_good(idx, c["bad"])
        if longest > max_held_s * fps:
            raise Skip("%s: pictures were lost in the recording and %.1f s of video in a row cannot be "
                       "decoded correctly (limit %.1f s)" % (key, longest / fps, max_held_s))
        if c["rows_held"]:
            notes.append("%s: %d row(s) repeat the last good picture because pictures were lost in the "
                         "recording (longest stretch %.2f s)" % (key, c["rows_held"], longest / fps))
        c["index"] = idx
        c["max_offset_ms"] = float(np.max(np.abs(c["t"][idx] - ticks)) * 1000)
        c["repeated"] = int(len(idx) - len(np.unique(idx)))
        gaps = np.diff(c["t"])
        c["longest_gap_ms"] = float(gaps.max() * 1000) if len(gaps) else 0.0
        c["missing_pictures"] = int(np.sum(np.round(gaps * fps) - 1).clip(min=0)) if len(gaps) else 0
        c["wait_for_first_picture_s"] = float(c["t"][0] - c["t_first_message"])
        if c["longest_gap_ms"] > 2500.0 / fps:
            notes.append("%s: a gap of %.0f ms between pictures" % (key, c["longest_gap_ms"]))
    first = min(c["t_first_message"] for c in cams.values())
    slow = max(cams, key=lambda k: cams[k]["t"][0])
    if t_start - first > 1.5:
        notes.append("%.1f s at the start are not in the dataset: %s delivered its first complete picture "
                     "only then" % (t_start - first, slow))

    names, cols = [], {"position": [], "velocity": [], "effort": []}
    action_cols, action_source = [], {}
    for g in state_groups:
        s = states[g]
        idx = hold_indices(s["t"], ticks)
        names += ["%s.%s" % (g, n) for n in s["names"]]
        for k in cols:
            cols[k].append(s[k][idx])
        c = commands.get(g)
        if c is not None and c["names"] == s["names"] and c["t"][0] <= ticks[-1]:
            cidx = hold_indices(c["t"], ticks)
            a = c["position"][cidx].copy()
            early = ticks < c["t"][0]                 # rows before the first command: hold the state
            a[early] = s["position"][idx][early]
            action_cols.append(a)
            action_source[g] = "command"
            if early.any():
                notes.append("%s: %d row(s) before the first command use the state as action" % (g, int(early.sum())))
        else:
            nxt = np.minimum(np.arange(len(ticks)) + 1, len(ticks) - 1)
            action_cols.append(s["position"][idx][nxt])
            action_source[g] = "next_state"
    if any(v == "next_state" for v in action_source.values()):
        notes.append("action is the next state (no commands recorded) for: %s"
                     % ", ".join(g for g, v in action_source.items() if v == "next_state"))
    out = {"ticks": ticks, "cams": cams, "trimmed_start_s": float(t_start - first), "names": names, "action_source": action_source, "notes": notes,
           "state": np.concatenate(cols["position"], axis=1).astype(np.float32),
           "velocity": np.concatenate(cols["velocity"], axis=1).astype(np.float32),
           "effort": np.concatenate(cols["effort"], axis=1).astype(np.float32),
           "action": np.concatenate(action_cols, axis=1).astype(np.float32),
           "clocks": dict([(k, c["clock"]) for k, c in cams.items()] + [(g, states[g]["clock"]) for g in state_groups])}
    for k in ("state", "action"):
        if not np.all(np.isfinite(out[k])):
            raise Skip("%s contains values that are not numbers" % k)
    return out


# ------------------------------------------------------------------------------------------
# Video
# ------------------------------------------------------------------------------------------
def probe_size(path):
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                        "-of", "json", path], capture_output=True, text=True, check=True)
    s = json.loads(r.stdout)["streams"][0]
    return int(s["width"]), int(s["height"])


def count_frames(path):
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-count_frames", "-show_entries",
                        "stream=nb_read_frames", "-of", "json", path], capture_output=True, text=True, check=True)
    return int(json.loads(r.stdout)["streams"][0]["nb_read_frames"])


def write_video(cam, out_path, fps, work_dir, codec, crf, stats_every=10):
    """Decodes the camera's stream, keeps for every row the chosen picture, encodes an mp4.
    Returns (width, height, per-channel stats or None)."""
    raw = os.path.join(work_dir, "stream.h265")
    with open(raw, "wb") as fh:
        for chunk in cam["data"]:
            fh.write(chunk)
    w, h = probe_size(raw)
    size = w * h * 3
    counts = np.bincount(cam["index"], minlength=len(cam["data"]))      # how often each picture is used
    last_needed = int(cam["index"].max())
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    dec = subprocess.Popen(["ffmpeg", "-v", "error", "-f", "hevc", "-i", raw, "-vsync", "0", "-f", "rawvideo",
                            "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    enc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (w, h),
                            "-r", str(fps), "-i", "-", "-an", "-c:v", codec, "-pix_fmt", "yuv420p", "-g", "2",
                            "-crf", str(crf), "-r", str(fps), out_path], stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    sums, sq, mins, maxs, npx, written, i = np.zeros(3), np.zeros(3), np.full(3, 1.0), np.zeros(3), 0, 0, 0
    try:
        while i <= last_needed:
            buf = dec.stdout.read(size)
            if len(buf) < size:
                break
            for _ in range(int(counts[i])):
                enc.stdin.write(buf)
                if written % stats_every == 0:
                    px = np.frombuffer(buf, np.uint8).reshape(h, w, 3)[::4, ::4].reshape(-1, 3) / 255.0
                    sums += px.sum(0)
                    sq += (px ** 2).sum(0)
                    mins, maxs = np.minimum(mins, px.min(0)), np.maximum(maxs, px.max(0))
                    npx += len(px)
                written += 1
            i += 1
    finally:
        dec.stdout.close()
        dec.kill()
        dec.wait()
        enc.stdin.close()
        err = enc.stderr.read().decode(errors="replace")
        enc.wait()
    if i <= last_needed:
        raise Skip("%s: only %d of %d pictures could be decoded" % (cam["topic"], i, last_needed + 1))
    if enc.returncode != 0:
        raise Skip("%s: video encoding failed: %s" % (cam["topic"], err[-300:]))
    if written != len(cam["index"]) or count_frames(out_path) != written:
        raise Skip("%s: the video does not have one picture per row" % cam["topic"])
    stats = None
    if npx:
        mean = sums / npx
        std = np.sqrt(np.maximum(sq / npx - mean ** 2, 0))
        stats = {k: [[[float(x)]] for x in v] for k, v in (("min", mins), ("max", maxs), ("mean", mean), ("std", std))}
        stats["count"] = [int(written)]
    return w, h, stats


# ------------------------------------------------------------------------------------------
# Writing the dataset
# ------------------------------------------------------------------------------------------
def write_parquet(path, al, episode_index, first_index, task_index, fps):
    import pyarrow as pa
    import pyarrow.parquet as pq
    n, dim = al["state"].shape

    def vec(a):
        return pa.FixedSizeListArray.from_arrays(pa.array(a.reshape(-1), type=pa.float32()), a.shape[1])
    table = pa.table({
        "observation.state": vec(al["state"]), "observation.velocity": vec(al["velocity"]),
        "observation.effort": vec(al["effort"]), "action": vec(al["action"]),
        "timestamp": pa.array(np.arange(n, dtype=np.float32) / np.float32(fps), type=pa.float32()),
        "frame_index": pa.array(np.arange(n), type=pa.int64()),
        "episode_index": pa.array(np.full(n, episode_index), type=pa.int64()),
        "index": pa.array(first_index + np.arange(n), type=pa.int64()),
        "task_index": pa.array(np.full(n, task_index), type=pa.int64()),
    })
    os.makedirs(os.path.dirname(path), exist_ok=True)
    pq.write_table(table, path)


def convert(episodes_dir, out_dir, fps=30, state_groups=("arm", "hand", "waist", "neck"), codec="libx264", crf=18,
            include_warnings=False, overwrite=False, robot_type="agibot_a3", log=print, cameras=None, max_held_s=1.1):
    episodes_dir, out_dir = os.path.abspath(os.path.expanduser(episodes_dir)), os.path.abspath(os.path.expanduser(out_dir))
    if os.path.exists(out_dir):
        if not overwrite:
            raise SystemExit("%s exists already; use --overwrite to rebuild it" % out_dir)
        if not os.path.exists(os.path.join(out_dir, "meta", "info.json")) and os.listdir(out_dir):
            raise SystemExit("%s is not a dataset made by this tool; not touching it" % out_dir)
        shutil.rmtree(out_dir)
    os.makedirs(os.path.join(out_dir, "meta"))
    work = os.path.join(out_dir, ".work")
    os.makedirs(work)
    tasks, ep_rows, ep_stats, report = [], [], [], []
    names = cam_info = None
    total = 0
    for ep in sorted(os.listdir(episodes_dir)):
        ep_dir = os.path.join(episodes_dir, ep)
        entry = {"episode": ep}
        report.append(entry)
        made = []
        try:
            if not os.path.exists(os.path.join(ep_dir, "receipt.json")):
                raise Skip("not completely uploaded and checked yet")
            info, videos, joints = read_episode(ep_dir)
            if info.get("warnings") and not include_warnings:
                raise Skip("recorded with warnings: %s" % "; ".join(info["warnings"]))
            al = align_episode(videos, joints, fps, list(state_groups), cameras, max_held_s)
            if names is None:
                names = al["names"]
            elif al["names"] != names:
                raise Skip("its joints differ from the first episode's")
            if cam_info is not None and sorted(al["cams"]) != sorted(cam_info):
                raise Skip("its cameras differ from the first episode's")
            idx = len(ep_rows)
            chunk = "chunk-%03d" % (idx // CHUNK_SIZE)
            stats, sizes = {}, {}
            for key, cam in al["cams"].items():
                feat = "observation.images." + key
                path = os.path.join(out_dir, "videos", chunk, feat, "episode_%06d.mp4" % idx)
                made.append(path)
                w, h, st = write_video(cam, path, fps, work, codec, crf)
                if cam_info is not None and cam_info[key] != (w, h):
                    raise Skip("%s changed its picture size" % key)
                sizes[key] = (w, h)
                if st:
                    stats[feat] = st
            task = info.get("task") or "unnamed task"
            if task not in tasks:
                tasks.append(task)
            path = os.path.join(out_dir, "data", chunk, "episode_%06d.parquet" % idx)
            made.append(path)
            write_parquet(path, al, idx, total, tasks.index(task), fps)
            cam_info = cam_info or sizes
            n = len(al["ticks"])
            for feat, key in (("observation.state", "state"), ("observation.velocity", "velocity"),
                              ("observation.effort", "effort"), ("action", "action")):
                stats[feat] = feature_stats(al[key])
            stats["timestamp"] = feature_stats(np.arange(n) / float(fps))
            stats["frame_index"] = feature_stats(np.arange(n))
            stats["episode_index"] = feature_stats(np.full(n, idx))
            stats["index"] = feature_stats(total + np.arange(n))
            stats["task_index"] = feature_stats(np.full(n, tasks.index(task)))
            ep_rows.append({"episode_index": idx, "tasks": [task], "length": n})
            ep_stats.append({"episode_index": idx, "stats": stats})
            total += n
            entry.update(status="converted", episode_index=idx, rows=n, seconds=round(n / fps, 2),
                         start_unix=float(al["ticks"][0]),
                         not_used_at_start_s=round(al["trimmed_start_s"], 2), action_source=al["action_source"], clocks=al["clocks"],
                         notes=al["notes"], cameras={k: {"max_offset_ms": round(c["max_offset_ms"], 1),
                                                         "pictures_used_twice": c["repeated"],
                                                         "pictures_missing_in_recording": c["missing_pictures"],
                                                         "rows_with_repeated_picture_after_loss": c["rows_held"],
                                                         "wait_for_first_picture_s": round(c["wait_for_first_picture_s"], 2),
                                                         "pictures_before_first_keyframe": c["skipped_before_keyframe"]}
                                                     for k, c in al["cams"].items()})
            log("%s -> episode %d, %d rows (%.1f s)%s" % (ep, idx, n, n / fps,
                                                          "".join("\n    note: " + x for x in al["notes"])))
        except Skip as e:
            for p in made:
                if os.path.exists(p):
                    os.remove(p)
            entry.update(status="skipped", reason=str(e))
            log("%s skipped: %s" % (ep, e))
    shutil.rmtree(work, ignore_errors=True)
    if not ep_rows:
        shutil.rmtree(out_dir)
        raise SystemExit("no episode could be converted; nothing written")

    dim = len(names)
    vec = lambda: {"dtype": "float32", "shape": [dim], "names": names}          # noqa: E731
    scalar = lambda t: {"dtype": t, "shape": [1], "names": None}                # noqa: E731
    features = {"observation.state": vec(), "observation.velocity": vec(), "observation.effort": vec(), "action": vec()}
    for key, (w, h) in sorted(cam_info.items()):
        features["observation.images." + key] = {
            "dtype": "video", "shape": [h, w, 3], "names": ["height", "width", "channels"],
            "info": {"video.fps": float(fps), "video.height": h, "video.width": w, "video.channels": 3,
                     "video.codec": {"libx264": "h264", "libx265": "hevc", "libaom-av1": "av1"}.get(codec, codec),
                     "video.pix_fmt": "yuv420p", "video.is_depth_map": False, "has_audio": False}}
    features.update({"timestamp": scalar("float32"), "frame_index": scalar("int64"), "episode_index": scalar("int64"),
                     "index": scalar("int64"), "task_index": scalar("int64")})
    info = {"codebase_version": CODEBASE_VERSION, "robot_type": robot_type, "total_episodes": len(ep_rows),
            "total_frames": total, "total_tasks": len(tasks), "total_videos": len(ep_rows) * len(cam_info),
            "total_chunks": (len(ep_rows) - 1) // CHUNK_SIZE + 1, "chunks_size": CHUNK_SIZE, "fps": fps,
            "splits": {"train": "0:%d" % len(ep_rows)},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": features}
    meta = os.path.join(out_dir, "meta")
    with open(os.path.join(meta, "info.json"), "w") as fh:
        json.dump(info, fh, indent=4)

    def jsonl(name, rows):
        with open(os.path.join(meta, name), "w") as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")
    jsonl("tasks.jsonl", [{"task_index": i, "task": t} for i, t in enumerate(tasks)])
    jsonl("episodes.jsonl", ep_rows)
    jsonl("episodes_stats.jsonl", ep_stats)
    with open(os.path.join(out_dir, "conversion_report.json"), "w") as fh:
        json.dump({"made_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "source": episodes_dir,
                   "options": {"fps": fps, "state_groups": list(state_groups), "codec": codec, "crf": crf,
                               "include_warnings": include_warnings},
                   "episodes": report}, fh, indent=2)
    log("dataset: %d episode(s), %d rows, %d camera(s), %d joints -> %s"
        % (len(ep_rows), total, len(cam_info), dim, out_dir))
    return info, report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", default="~/nova_teleop_data/episodes", help="folder written by ingest_server.py")
    ap.add_argument("--out", required=True, help="dataset folder to create")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--state-groups", default="arm,hand,waist,neck",
                    help="joint groups in state and action, in this order (also possible: leg)")
    ap.add_argument("--cameras", default="",
                    help="cameras to use, comma separated, e.g. chest_front_d457,waist_front_d415 (default: all recorded)")
    ap.add_argument("--max-held-s", type=float, default=1.1,
                    help="skip an episode when a camera lost pictures and more than this many seconds in a row "
                         "cannot be decoded correctly")
    ap.add_argument("--codec", default="libx264", choices=["libx264", "libx265", "libaom-av1"])
    ap.add_argument("--crf", type=int, default=18, help="video quality: lower is better and larger")
    ap.add_argument("--include-warnings", action="store_true", help="also use episodes recorded with warnings")
    ap.add_argument("--overwrite", action="store_true", help="rebuild an existing dataset folder")
    a = ap.parse_args(argv)
    for tool in ("ffmpeg", "ffprobe"):
        if not shutil.which(tool):
            sys.exit("%s is not installed" % tool)
    convert(a.episodes, a.out, a.fps, tuple(g for g in a.state_groups.split(",") if g), a.codec, a.crf,
            a.include_warnings, a.overwrite, cameras=[c for c in a.cameras.split(",") if c] or None,
            max_held_s=a.max_held_s)


if __name__ == "__main__":
    main()
