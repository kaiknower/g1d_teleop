"""Passive DDS command/state recording. Never constructs a publisher or RPC."""
from g1d_vr_teleop.config import DDS_INTERFACE
import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import json
import math
from pathlib import Path
import queue
import signal
import threading
import time


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def pack(msg, kind):
    if kind == "column":
        return {"x": msg.x, "y": msg.y, "z": msg.z}
    if kind == "lowcmd":
        motors, names = msg.motor_cmd, ("mode", "q", "dq", "tau", "kp", "kd")
    elif kind == "lowstate":
        motors, names = msg.motor_state, ("mode", "q", "dq", "tau_est")
    elif kind == "gripper_cmd":
        motors, names = msg.cmds, ("mode", "q", "dq", "tau", "kp", "kd")
    else:
        motors, names = msg.states, ("mode", "q", "dq", "tau_est")
    result = {name: [getattr(m, name) for m in motors] for name in names}
    for name in ("mode_pr", "mode_machine", "tick", "crc"):
        if hasattr(msg, name):
            result[name] = getattr(msg, name)
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--stop-file", type=Path, required=True)
    p.add_argument("--duration", type=float, default=1800)
    args = p.parse_args(argv)
    if not args.output.is_absolute() or not 0 < args.duration <= 3600:
        p.error("Absolute new output directory and duration <=3600 required")
    args.output.mkdir(parents=True, exist_ok=False)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_
    ChannelFactoryInitialize(0, DDS_INTERFACE)
    events, stop, failed = queue.Queue(maxsize=20000), threading.Event(), threading.Event()
    received, written, errors = Counter(), Counter(), []
    lock = threading.Lock()
    start = time.monotonic()
    report = {"schema": "g1d_vr.passive_dds_recording.v1", "status": "recording",
        "started_utc": datetime.now(timezone.utc).isoformat(), "start_monotonic_s": start,
        "start_unix_s": time.time(), "robot_commands_sent": 0, "invalid_motor_slots": [13],
        "timing": "Robot host receive monotonic_ns and unix_ns, same host clock as video frames.jsonl; not hardware synchronized",
        "provenance": "Observed DDS commands, publisher identity unavailable. Cross-check intended actions with sampler wire audit and lifecycle receipts.",
        "nonfinite_values": "Nonfinite scalar values are null; raw invalid slot 13 is never interpreted as a real joint",
        "scope": "All available motors: q/dq/tau/kp/kd/mode commands, q/dq/tau_est state; raw Dex1 units; signed column coordinate"}

    def save():
        with lock:
            report.update(received=dict(received), written=dict(written), errors=list(errors))
        report.update(elapsed_s=time.monotonic()-start, queue_size=events.qsize())
        temporary = args.output / "recording.json.tmp"
        temporary.write_text(json.dumps(clean(report), indent=2, allow_nan=False)+"\n")
        temporary.replace(args.output / "recording.json")

    def receive(topic, kind, msg):
        if stop.is_set() or failed.is_set():
            return
        try:
            now_m, now_u = time.monotonic_ns(), time.time_ns()
            data = pack(msg, kind)
            with lock:
                received[topic] += 1
                seq = received[topic]
            events.put_nowait({"topic": topic, "kind": kind, "sequence": seq,
                "receive_monotonic_ns": now_m, "receive_unix_ns": now_u, "data": data})
        except Exception as exc:
            with lock:
                errors.append({"topic": topic, "error": repr(exc), "monotonic_ns": time.monotonic_ns()})
            failed.set()

    def write():
        try:
            with gzip.open(args.output / "action_state.jsonl.gz", "wt", compresslevel=1) as handle:
                while not stop.is_set() or not events.empty():
                    try:
                        event = events.get(timeout=.1)
                    except queue.Empty:
                        continue
                    handle.write(json.dumps(clean(event), separators=(",", ":"), allow_nan=False)+"\n")
                    with lock:
                        written[event["topic"]] += 1
                    events.task_done()
                handle.flush()
        except Exception as exc:
            with lock:
                errors.append({"writer_error": repr(exc)})
            failed.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    worker = threading.Thread(target=write, daemon=True)
    worker.start()
    subscriptions = []
    specs = [("rt/lowcmd", LowCmd_, "lowcmd"), ("rt/lowstate", LowState_, "lowstate"),
        ("rt/dex1/left/cmd", MotorCmds_, "gripper_cmd"), ("rt/dex1/right/cmd", MotorCmds_, "gripper_cmd"),
        ("rt/dex1/left/state", MotorStates_, "gripper_state"), ("rt/dex1/right/state", MotorStates_, "gripper_state"),
        ("rt/hispeed_state", Point32_, "column")]
    try:
        for topic, typ, kind in specs:
            sub = ChannelSubscriber(topic, typ)
            sub.Init(lambda msg, topic=topic, kind=kind: receive(topic, kind, msg), 100)
            subscriptions.append(sub)
        while not stop.is_set() and not failed.is_set() and time.monotonic()-start < args.duration:
            save()
            if args.stop_file.exists():
                break
            stop.wait(1)
    finally:
        stop.set()
        worker.join(timeout=20)
        report.update(status="failed" if failed.is_set() or worker.is_alive() else "finalized",
            stopped_utc=datetime.now(timezone.utc).isoformat(), writer_thread_alive=worker.is_alive())
        save()
        print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
