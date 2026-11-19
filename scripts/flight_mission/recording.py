"""Read complete Mimir recordings into independent verification evidence."""

from dataclasses import dataclass, field
import json


@dataclass
class Evidence:
    events: list[tuple[float, dict]] = field(default_factory=list)
    poses: list[tuple[float, list[float], list[float]]] = field(default_factory=list)
    scalars: dict[str, list[tuple[float, list[float]]]] = field(default_factory=dict)
    recording_complete: bool = False
    down_frames: list[tuple[float, int, int, int]] = field(default_factory=list)
    capture: dict = field(default_factory=dict)


def read_recording(path):
    from rerun.chunk import RrdReader

    reader = RrdReader(path)
    [recording] = reader.recordings()
    # Indexed access requires a completed footer; live/truncated runs cannot pass.
    reader.store(store=recording).schema()
    data = Evidence()
    scalar_paths = {
        "/fc/debug/state",
        "/fc/debug/precision",
        "/landing/target",
        "/landing/quality",
        "/landing/runtime",
        "/fc/imu/source",
        "/fc/imu/health",
        "/fc/imu/quaternion_xyzw",
    }
    for chunk in reader.stream(store=recording):
        entity = str(chunk.entity_path)
        if entity not in scalar_paths | {
            "/gt/pose",
            "/logs/flight",
            "/logs/render",
            "/logs/mimir",
            "/cameras/down/image",
        }:
            continue
        batch = chunk.to_record_batch()
        if entity == "/logs/mimir" and "TextLog:text" in batch.schema.names:
            data.recording_complete |= any(
                text == "recording complete"
                for row in batch.column("TextLog:text").to_pylist()
                for text in row or []
            )
        if "sim_time" not in batch.schema.names:
            continue
        stamps = [value / 1e9 for value in batch.column("sim_time").cast("int64").to_pylist()]
        if entity == "/cameras/down/image" and "Image:buffer" in batch.schema.names:
            for stamp, image, formats in zip(
                stamps, batch.column("Image:buffer"), batch.column("Image:format").to_pylist()
            ):
                if image.is_valid and len(image) == 1 and formats:
                    data.down_frames.append(
                        (stamp, formats[0]["width"], formats[0]["height"], len(image[0]))
                    )
        if entity == "/gt/pose" and "Transform3D:translation" in batch.schema.names:
            for stamp, position, quaternion in zip(
                stamps,
                batch.column("Transform3D:translation").to_pylist(),
                batch.column("Transform3D:quaternion").to_pylist(),
            ):
                if position and quaternion:
                    data.poses.append((stamp, position[0], quaternion[0]))
        if entity in scalar_paths and "Scalars:scalars" in batch.schema.names:
            data.scalars.setdefault(entity, []).extend(
                (stamp, row)
                for stamp, row in zip(stamps, batch.column("Scalars:scalars").to_pylist())
                if row
            )
        if "TextLog:text" in batch.schema.names:
            for stamp, row in zip(stamps, batch.column("TextLog:text").to_pylist()):
                for text in row or []:
                    if entity == "/logs/flight" and text.startswith("mission {"):
                        data.events.append((stamp, json.loads(text.removeprefix("mission "))))
                    if entity == "/logs/render" and text.startswith("capture {"):
                        config = json.loads(text.removeprefix("capture "))
                        if data.capture and config != data.capture:
                            raise ValueError("render capture configuration changed during recording")
                        data.capture = config
    data.events.sort(key=lambda row: row[0])
    data.poses.sort(key=lambda row: row[0])
    for values in data.scalars.values():
        values.sort(key=lambda row: row[0])
    data.down_frames.sort()
    return data
