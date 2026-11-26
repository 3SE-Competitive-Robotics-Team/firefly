"""Export one launch window from a sealed HIL recording for landing --takeoff-replay."""

import argparse
from bisect import bisect_right
import json
from pathlib import Path

import numpy as np
from PIL import Image

from firefly_workspace.recording import recording_chunks


def samples(run, entity, columns):
    rows = {}
    for chunk in recording_chunks(run, content=entity):
        batch = chunk.to_record_batch()
        if not {"sim_time", *columns} <= set(batch.schema.names):
            continue
        stamps = batch["sim_time"].cast("int64").to_pylist()
        for stamp, *values in zip(stamps, *(batch[c].to_pylist() for c in columns), strict=True):
            if stamp is not None and all(values):
                rows[stamp] = values
    stamps = sorted(rows)

    def latest(stamp, max_age_s):
        index = bisect_right(stamps, stamp) - 1
        if index >= 0:
            age = (stamp - stamps[index]) * 1e-9
            if age <= max_age_s:
                return age, rows[stamps[index]]
        return None

    return latest


def mission_events(run, end):
    events = []
    for chunk in recording_chunks(run, content="/logs/flight"):
        batch = chunk.to_record_batch()
        if not {"sim_time", "TextLog:text"} <= set(batch.schema.names):
            continue
        for stamp, messages in zip(batch["sim_time"].cast("int64").to_pylist(), batch["TextLog:text"].to_pylist(), strict=True):
            if stamp is None or stamp * 1e-9 > end:
                continue
            for message in messages:
                if message.startswith("mission {"):
                    event = json.loads(message.removeprefix("mission "))
                    if event["kind"] in ("home", "phase"):
                        events.append(dict(time=stamp * 1e-9, **event))
    return sorted(events, key=lambda event: event["time"])


def export(run, start, end, output):
    output.mkdir(parents=True, exist_ok=False)
    scalar = ["Scalars:scalars"]
    attitude = samples(run, "/fc/standby_eskf/quaternion_xyzw", scalar)
    ready = samples(run, "/fc/standby_eskf/health", scalar)
    imu_age = samples(run, "/fc/standby_eskf/imu_age_s", scalar)
    odom = samples(run, "/vio/odom", ["Transform3D:translation", "Transform3D:quaternion"])
    tracks = samples(run, "/vio/diagnostics/tracks", scalar)
    age = samples(run, "/vio/prediction/posterior_age_s", scalar)
    velocity = samples(run, "/vio/prediction/velocity_body_odom", scalar)
    fc = samples(run, "/fc/debug/state", scalar)
    frames = {}
    for chunk in recording_chunks(run, content="/cameras/down/image"):
        batch = chunk.to_record_batch()
        if not {"sim_time", "Image:buffer"} <= set(batch.schema.names):
            continue
        for stamp, buf in zip(batch["sim_time"].cast("int64").to_pylist(), batch["Image:buffer"], strict=True):
            if stamp is None or not start <= stamp * 1e-9 <= end or stamp in frames:
                continue
            # The standby ESKF diagnostic is archived at 1 Hz.
            att, health, imu_delay = (read(stamp, 1.2) for read in (attitude, ready, imu_age))
            imu = None
            if att and health and imu_delay and health[1][0][0] == 1 and imu_delay[1][0][0] <= 0.1:
                imu = att[1][0]
            pose, support, posterior, speed = (read(stamp, 0.15) for read in (odom, tracks, age, velocity))
            vio = None
            if pose and support and posterior and speed:
                pos, quat = pose[1]
                _, _, stereo, landmarks = support[1][0]
                vio = dict(position=pos[0], orientation=quat[0], velocity=speed[1][0],
                           posterior_age_s=posterior[1][0][0] + posterior[0],
                           stereo_ids=int(stereo), landmark_ids=int(landmarks))
            image = f"{stamp}.png"
            pixels = buf.values[0].values.to_numpy().reshape(288, 352)
            Image.fromarray(np.asarray(pixels)).save(output / image)
            mode = fc(stamp, 0.25)
            launch_active = mode[1][0][0] in (2, 3) if mode else None  # TAKEOFF, HOLD
            takeoff_report = stamp * 1e-9 - mode[0] if mode and mode[1][0][0] == 2 else None
            frames[stamp] = dict(time=stamp * 1e-9, image=image, imu=imu, vio=vio,
                                 launch_active=launch_active, takeoff_report=takeoff_report)
    events = mission_events(run, end)
    if not frames and not events:
        raise ValueError("no mission events or down-camera frames in the selected interval")
    (output / "frames.json").write_text(json.dumps(dict(
        end=end, events=events, frames=[frames[t] for t in sorted(frames)]
    )))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("start", type=float, help="first capture time in seconds")
    parser.add_argument("end", type=float, help="last capture time in seconds")
    parser.add_argument("output", type=Path, help="new directory for one takeoff")
    args = parser.parse_args()
    export(args.run, args.start, args.end, args.output)
