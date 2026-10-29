"""Record three existing JPEG streams on a fixed timeline, without robot control.

Only ZMQ SUB sockets on camera ports 55555 through 55557 are constructed.
Missing images remain black until the first frame, then repeat the last image.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np
import zmq


FPS = 10
STREAMS = {
    "head_stereo": (55555, 1280, 480),
    "left_wrist": (55556, 640, 480),
    "right_wrist": (55557, 640, 480),
}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def save_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def receive(latest, stats, errors, lock, stop):
    """Own camera subscriptions and decoding independently of video encoding."""
    context = zmq.Context()
    poller = zmq.Poller()
    sockets = {}
    try:
        for name, (port, _, _) in STREAMS.items():
            camera = context.socket(zmq.SUB)
            camera.setsockopt(zmq.LINGER, 0)
            camera.setsockopt(zmq.RCVHWM, 2)
            # These three streams are single-part JPEG, never multipart depth.
            camera.setsockopt(zmq.CONFLATE, 1)
            camera.setsockopt(zmq.SUBSCRIBE, b"")
            camera.connect(f"tcp://127.0.0.1:{port}")
            poller.register(camera, zmq.POLLIN)
            sockets[camera] = name
        while not stop.is_set():
            for camera, _ in poller.poll(25):
                name = sockets[camera]
                parts = camera.recv_multipart()
                received = time.monotonic()
                try:
                    if len(parts) != 1 or not parts[0].startswith(b"\xff\xd8"):
                        raise ValueError("Expected one JPEG message")
                    image = cv2.imdecode(np.frombuffer(parts[0], np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError("JPEG decoding failed")
                    with lock:
                        stats[name]["received_frames"] += 1
                        sequence = stats[name]["received_frames"]
                        latest[name] = (image, received, sequence)
                except (ValueError, cv2.error) as exc:
                    with lock:
                        stats[name]["decode_errors"] += 1
                        errors.append({"stream": name, "error": str(exc), "utc": utc_now()})
    except BaseException as exc:
        with lock:
            errors.append({"receiver_error": str(exc), "type": type(exc).__name__, "utc": utc_now()})
            stats["receiver_failed"] = True
    finally:
        for camera in sockets:
            camera.close()
        context.term()


def fit_frame(image, width, height):
    """Letterbox unexpected dimensions without stretching or cropping."""
    h, w = image.shape[:2]
    if (w, h) == (width, height):
        return image.copy()
    scale = min(width / w, height / h)
    size = (max(1, round(w * scale)), max(1, round(h * scale)))
    resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    canvas = np.zeros((height, width, 3), np.uint8)
    x, y = (width - size[0]) // 2, (height - size[1]) // 2
    canvas[y:y + size[1], x:x + size[0]] = resized
    return canvas


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Absolute new directory; never overwritten")
    parser.add_argument("--duration", type=float, default=420, help="Seconds, greater than 0 and at most 3600")
    parser.add_argument("--stop-file", type=Path, help="Stop gracefully when this file appears")
    parser.add_argument("--stale-after", type=float, default=0.5, help="Mark receive age above this many seconds STALE")
    args = parser.parse_args(argv)
    if not args.output.is_absolute():
        parser.error("--output must be absolute")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error("--duration must be greater than 0 and at most 3600")
    if not math.isfinite(args.stale_after) or not 0 < args.stale_after <= 5:
        parser.error("--stale-after must be greater than 0 and at most 5")
    if args.stop_file is not None and not args.stop_file.is_absolute():
        parser.error("--stop-file must be absolute")
    cv2.setNumThreads(1)
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    stop, request_stop = threading.Event(), threading.Event()
    requested_reason = [None]

    def handle_signal(signum, _frame):
        requested_reason[0] = signal.Signals(signum).name
        request_stop.set()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    latest, lock = {}, threading.Lock()
    stats = {name: {"received_frames": 0, "decode_errors": 0} for name in STREAMS}
    errors = deque(maxlen=30)
    report = {
        "schema": "g1d_vr.fixed_timeline_recording.v1",
        "status": "starting", "created_utc": utc_now(), "output": str(output),
        "commands_sent": 0, "robot_motion_commands_sent": 0,
        "camera_socket_type": "ZMQ_SUB", "camera_host": "127.0.0.1",
        "fps": FPS, "requested_duration_s": args.duration,
        "stop_file": str(args.stop_file) if args.stop_file else None,
        "timing": "One video frame per 0.1 seconds, including missing/stale frames; no timeline compression",
        "receive_age_note": "Local receive age at nominal timeline sample; not exposure age or camera latency",
        "stale_after_s": args.stale_after, "streams": {},
    }
    save_json(output / "recording.json", report)
    writers, used, previous_sequence = {}, {}, {}
    worker = None
    index = 0
    failure = None
    start = None
    reason = "duration"
    try:
        for name, (port, width, height) in STREAMS.items():
            path = output / f"{name}.mp4"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (width, height))
            if not writer.isOpened():
                raise OSError(f"VideoWriter failed: {path}")
            writers[name] = writer
            report["streams"][name] = {
                "path": str(path), "port": port, "width": width, "height": height,
                "frames": 0, "placeholder_frames": 0, "stale_frames": 0, "repeated_frames": 0,
            }
        start = time.monotonic()
        report.update(status="recording", started_utc=utc_now(), start_monotonic=start)
        save_json(output / "recording.json", report)
        worker = threading.Thread(target=receive, args=(latest, stats, errors, lock, stop), daemon=True)
        worker.start()
        print(json.dumps({"status": "recording", "output": str(output), "commands_sent": 0}), flush=True)
        end_elapsed = args.duration
        total = math.ceil(end_elapsed * FPS)
        next_status = start + 2
        with (output / "frames.jsonl").open("x", encoding="utf-8") as events:
            while index < total:
                now = time.monotonic()
                if reason == "duration" and (request_stop.is_set() or (args.stop_file and args.stop_file.exists())):
                    reason = requested_reason[0] or "stop_file"
                    end_elapsed = min(args.duration, max(0, now - start))
                    total = math.ceil(end_elapsed * FPS)
                    if index >= total:
                        break
                scheduled = start + index / FPS
                if now < scheduled:
                    request_stop.wait(min(0.025, scheduled - now))
                    continue
                with lock:
                    snapshot = dict(latest)
                event = {
                    "frame_index": index, "video_elapsed_s": index / FPS,
                    "scheduled_monotonic": scheduled, "written_monotonic": now,
                    "schedule_lag_s": max(0, now - scheduled), "commands_sent": 0, "streams": {},
                }
                for name, (_, width, height) in STREAMS.items():
                    candidate = snapshot.get(name)
                    # Never insert an image received after this timeline slot.
                    # On catch-up, repeat the last eligible image for missed slots.
                    if candidate is not None and candidate[1] <= scheduled:
                        used[name] = candidate
                    current = used.get(name)
                    placeholder = current is None
                    age = None if placeholder else max(0, scheduled - current[1])
                    sequence = None if placeholder else current[2]
                    stale = placeholder or age > args.stale_after
                    repeated = sequence is not None and sequence == previous_sequence.get(name)
                    source_shape = None if placeholder else list(current[0].shape)
                    frame = np.zeros((height, width, 3), np.uint8) if placeholder else fit_frame(current[0], width, height)
                    label = "NO FRAME / STALE" if placeholder else ("STALE" if stale else "LIVE")
                    age_label = "unavailable" if age is None else f"{age:.3f}s"
                    cv2.rectangle(frame, (0, 0), (width, 53), (0, 0, 0), -1)
                    color = (40, 40, 255) if stale else (70, 240, 70)
                    cv2.putText(frame, f"{name} {label} receive_age={age_label}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)
                    cv2.putText(frame, f"timeline={index / FPS:.1f}s frame={index} repeated={int(repeated)}", (8, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.49, (235, 235, 235), 1, cv2.LINE_AA)
                    writers[name].write(frame)
                    row = report["streams"][name]
                    row["frames"] += 1
                    row["placeholder_frames"] += int(placeholder)
                    row["stale_frames"] += int(stale)
                    row["repeated_frames"] += int(repeated)
                    previous_sequence[name] = sequence
                    event["streams"][name] = {
                        "source_sequence": sequence, "source_received_monotonic": None if placeholder else current[1],
                        "receive_age_s": age, "placeholder": placeholder, "stale": stale,
                        "repeated": repeated, "source_shape": source_shape,
                    }
                events.write(json.dumps(event, allow_nan=False) + "\n")
                events.flush()
                index += 1
                if now >= next_status:
                    with lock:
                        report["receive_stats"] = {k: dict(v) if isinstance(v, dict) else v for k, v in stats.items()}
                    report.update(elapsed_s=now - start, timeline_frames=index, heartbeat_utc=utc_now())
                    save_json(output / "recording.json", report)
                    next_status = now + 2
            events.flush()
            os.fsync(events.fileno())
    except BaseException as exc:
        failure = {"type": type(exc).__name__, "error": str(exc)}
        reason = "exception"
    finally:
        stop.set()
        if worker is not None:
            worker.join(timeout=3)
        for writer in writers.values():
            writer.release()
        verification_ok = bool(writers)
        for name, row in report["streams"].items():
            path = Path(row["path"])
            row["bytes"] = path.stat().st_size if path.exists() else 0
            cap = cv2.VideoCapture(str(path))
            opened = cap.isOpened()
            decoded, _ = cap.read()
            count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            fps = float(cap.get(cv2.CAP_PROP_FPS))
            last_decoded = False
            if count > 0:
                cap.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
                last_decoded, _ = cap.read()
            cap.release()
            row.update(decoded_frame_count=count, decoded_fps=fps, duration_s=count / FPS,
                       verified=bool(opened and decoded and last_decoded and count == row["frames"] and abs(fps - FPS) < 0.01))
            verification_ok = verification_ok and row["verified"]
            if path.exists():
                with path.open("rb") as handle:
                    os.fsync(handle.fileno())
        with lock:
            report["receive_stats"] = stats
            report["recent_errors"] = list(errors)
        receiver_failed = stats.get("receiver_failed", False) or (worker is not None and worker.is_alive())
        report.update(
            status="failed" if failure or receiver_failed or not verification_ok else "finalized",
            stopped_utc=utc_now(), stop_reason=reason, failure=failure,
            elapsed_s=0 if start is None else time.monotonic() - start,
            timeline_frames=index, video_duration_s=index / FPS,
            all_streams_received=all(stats[name]["received_frames"] > 0 for name in STREAMS),
            all_mp4s_verified=verification_ok, commands_sent=0,
        )
        save_json(output / "recording.json", report)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["status"] == "finalized" else 1


if __name__ == "__main__":
    raise SystemExit(main())
