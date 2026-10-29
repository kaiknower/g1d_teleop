"""Trim three video streams and DDS action/state to a measured interval."""
from collections import Counter
import gzip
import json

def save(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def read(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def trim_episode(output, interval, phase="dual_arm_dex1_vr_teleoperation"):
    import cv2
    start, end = interval["start_monotonic_s"], interval["end_monotonic_s"]
    raw, task = output / "raw", output / "task"
    task.mkdir(exist_ok=False)
    rows = [json.loads(line) for line in (raw / "video/frames.jsonl").open()]
    selected = [row for row in rows if start <= row["scheduled_monotonic"] <= end]
    if not selected:
        raise RuntimeError("No video frames within measured-motion interval")
    indices = [row["frame_index"] for row in selected]
    if indices != list(range(indices[0], indices[-1] + 1)):
        raise RuntimeError("Video timeline indices are not contiguous")
    video_report = read(raw / "video/recording.json")
    videos = {}
    for name, spec in video_report["streams"].items():
        cap = cv2.VideoCapture(str(raw / "video" / f"{name}.mp4"))
        path = task / f"{name}.mp4"
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), video_report["fps"], (spec["width"], spec["height"]))
        if not cap.isOpened() or not writer.isOpened():
            cap.release()
            writer.release()
            raise RuntimeError(f"Could not prepare video trim: {name}")
        try:
            # Decode sequentially to avoid inaccurate codec frame seeking.
            for index in range(indices[-1] + 1):
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f"Video decode failed: {name} at {index}")
                if index >= indices[0]:
                    writer.write(frame)
        finally:
            cap.release()
            writer.release()
        check = cv2.VideoCapture(str(path))
        count = int(check.get(cv2.CAP_PROP_FRAME_COUNT))
        first_ok, _ = check.read()
        check.set(cv2.CAP_PROP_POS_FRAMES, max(0, count - 1))
        last_ok, _ = check.read()
        check.release()
        if count != len(selected) or not first_ok or not last_ok:
            raise RuntimeError(f"Trimmed video verification failed: {name}")
        videos[name] = {"path": str(path), "frames": count, "verified": True}
    with (task / "frames.jsonl").open("x") as handle:
        for i, row in enumerate(selected):
            row.update(task_frame_index=i, task_elapsed_s=i / video_report["fps"])
            handle.write(json.dumps(row) + "\n")
    counts = Counter()
    with gzip.open(raw / "action-state/action_state.jsonl.gz", "rt") as source, gzip.open(task / "action_state.jsonl.gz", "wt", compresslevel=1) as sink:
        for line in source:
            event = json.loads(line)
            if start <= event["receive_monotonic_ns"] / 1e9 <= end:
                sink.write(line)
                counts[event["topic"]] += 1
    if not counts.get("rt/lowstate") or not counts.get("rt/lowcmd"):
        raise RuntimeError("Task action/state trim lacks lowstate or lowcmd")
    report = {"phase": phase, "interval": interval,
        "raw_directory": str(raw), "raw_is_delivery": False, "task_directory": str(task),
        "video_frame_boundary_uncertainty_s": 1 / video_report["fps"],
        "videos": videos, "action_events": dict(counts), "status": "finalized_verified"}
    save(task / "episode.json", report)
    return report
