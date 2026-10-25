"""IPC 诊断的严格解码与 Rerun 写入；体素增量先折叠，再按预算写快照。"""

import math
import time

import numpy as np
import pyarrow as pa
import rerun as rr
from firefly_mujoco import IMAGE_HEIGHT, IMAGE_WIDTH

from .messages import (
    VIZ_KIND_POSE, VIZ_KIND_KEYPOINTS_2D, VIZ_KIND_MATRIX, VIZ_KIND_LINE_STRIP,
    VIZ_KIND_VOXELS, VIZ_KIND_VOXELS_ADD, VIZ_KIND_VOXELS_REMOVE, VIZ_KIND_SCALARS,
    VIZ_KIND_BAR_CHART, VIZ_KIND_ARROWS, VIZ_KIND_CLEAR, VIZ_KIND_CAMERA,
)
from .recording import RecordingOutputs, PIXEL_CENTER_OFFSET, at


TRAJECTORIES = {"gt/pose": ("gt/traj", [64, 128, 255]),
                "okvis/odom": ("okvis/traj", [255, 160, 64]),
                "okvis/map/pose": ("okvis/map/traj", [90, 235, 120])}


def entity(msg):
    if not 0 < msg.entity_len <= len(msg.entity):
        raise ValueError("invalid entity length")
    result = bytes(msg.entity[:msg.entity_len]).decode("utf-8")
    if ("\x00" in result or any(part in ("", ".", "..") for part in result.split("/"))
            or result == "live" or result.startswith("live/")):
        raise ValueError("invalid entity path")
    return result


def count(value, capacity):
    if not 0 <= value <= capacity:
        raise ValueError(f"payload count {value} exceeds {capacity}")
    return value


class Writer:
    def __init__(self, recordings: RecordingOutputs, options):
        self.recordings, self.options = recordings, options
        self.rec = recordings.archive
        self.voxels = {}
        self.voxel_sequence = None
        self.voxel_pending = None
        self.calibrations = {}
        self.previous_poses = {}
        self.last_flush = time.monotonic()

    def consume(self, item):
        kind, msg = item
        if kind == "gt":
            self.ground_truth(msg)
        elif kind == "map":
            if msg.is_initialized:
                self._pose("okvis/map/pose", msg.timestamp,
                           [msg.position_x, msg.position_y, msg.position_z],
                           [msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w])
        elif kind == "log":
            self.log_text(msg)
        elif kind == "viz":
            self.handle(msg)
        else:
            raise ValueError(f"unsupported telemetry kind {kind}")

    def ground_truth(self, msg):
        if not msg.is_initialized:
            self.previous_poses.pop("gt/pose", None)
            return
        self._pose("gt/pose", msg.timestamp,
                   [msg.position_x, msg.position_y, msg.position_z],
                   [msg.quat_x, msg.quat_y, msg.quat_z, msg.quat_w])

    def _pose(self, name, stamp, xyz, quat):
        if (not math.isfinite(stamp) or stamp < 0 or not np.isfinite([*xyz, *quat]).all()
                or abs(np.linalg.norm(quat) - 1) > 1e-5):
            raise ValueError(f"invalid pose for {name}")
        trajectory = TRAJECTORIES.get(name)
        previous = self.previous_poses.get(name)
        if previous is not None:
            if stamp < previous[0]:
                raise ValueError(f"regressed pose timestamp for {name}")
            if stamp == previous[0]:
                return
        at(self.rec, stamp)
        self.rec.log(name, rr.Transform3D(translation=xyz, quaternion=quat), strict=True)
        if trajectory is not None:
            path, color = trajectory
            if previous is not None:
                self.rec.log(path, rr.LineStrips3D([[previous[1], xyz]], colors=color), strict=True)
            self.previous_poses[name] = stamp, xyz

    def handle(self, msg):
        name = entity(msg)
        if not math.isfinite(msg.timestamp) or msg.timestamp < 0:
            raise ValueError(f"invalid sim_time for {name}")
        kind = msg.kind
        if kind == VIZ_KIND_POSE:
            self._pose(name, msg.timestamp, list(msg.xyz), list(msg.quat_xyzw))
            return
        at(self.rec, msg.timestamp)
        if kind == VIZ_KIND_SCALARS:
            n = count(msg.scalar_count, len(msg.scalars))
            self.rec.log(name, rr.Scalars(list(msg.scalars[:n])), strict=True)
        elif kind == VIZ_KIND_KEYPOINTS_2D:
            n = count(msg.point_count, len(msg.points))
            width, height, total = msg.scalars[:3]
            if (msg.scalar_count != 3 or not np.isfinite([width, height, total]).all()
                    or width <= 0 or height <= 0 or width != int(width) or height != int(height)
                    or total < n or total != int(total)):
                raise ValueError(f"invalid keypoint dimensions/count {name}")
            if any(not np.isfinite(p).all() or p[2] < 0 or p[2] != int(p[2]) or p[2] >= 2**64
                   for p in msg.points[:n]):
                raise ValueError(f"invalid keypoints {name}")
            points = np.array([[p[0], p[1]] for p in msg.points[:n]], dtype=np.float32).reshape(-1, 2)
            ids = np.array([p[2] for p in msg.points[:n]], dtype=np.uint64)
            if name in ("okvis/debug/keypoints/left", "okvis/debug/keypoints/right"):
                if (width, height) != (IMAGE_WIDTH, IMAGE_HEIGHT):
                    raise ValueError(f"keypoint resolution does not match camera: {name}")
                self.recordings.matches.observe(name.rsplit("/", 1)[1], msg.timestamp, points, ids, list(msg.color))
            else:
                self.rec.log(name, rr.Points2D(points, colors=list(msg.color), radii=2),
                             rr.AnyValues(track_id=pa.array(ids, type=pa.uint64()),
                                          image_width=int(width), image_height=int(height), total_keypoints=int(total)), strict=True)
        elif kind == VIZ_KIND_MATRIX:
            rows, cols = msg.scalars[:2]
            n = count(msg.point_count, 3 * len(msg.points))
            if (msg.scalar_count != 2 or not np.isfinite([rows, cols]).all() or rows <= 0 or cols <= 0
                    or rows != int(rows) or cols != int(cols) or int(rows) * int(cols) != n):
                raise ValueError(f"invalid matrix {name}")
            values = np.ctypeslib.as_array(msg.points).reshape(-1)[:n].copy().reshape(int(rows), int(cols))
            self.rec.log(name, rr.Tensor(values), strict=True)
        elif kind == VIZ_KIND_LINE_STRIP:
            n = count(msg.point_count, len(msg.points))
            self.rec.log(name, rr.LineStrips3D([list(map(list, msg.points[:n]))], colors=list(msg.color)), strict=True)
        elif kind in (VIZ_KIND_VOXELS, VIZ_KIND_VOXELS_ADD, VIZ_KIND_VOXELS_REMOVE):
            self._voxel(name, msg)
        elif kind == VIZ_KIND_BAR_CHART:
            n = count(msg.bin_count, len(msg.bins))
            self.rec.log(name, rr.BarChart(list(msg.bins[:n]),
                                          abscissa=[msg.bin_start + i * msg.bin_width for i in range(n)]), strict=True)
        elif kind == VIZ_KIND_ARROWS:
            n = count(msg.arrow_count, len(msg.arrow_vectors))
            self.rec.log(name, rr.Arrows3D(origins=list(map(list, msg.arrow_origins[:n])),
                                          vectors=list(map(list, msg.arrow_vectors[:n])), colors=list(msg.color)), strict=True)
        elif kind == VIZ_KIND_CLEAR:
            self.rec.log(name, rr.Clear(recursive=True), strict=True)
            for key in list(self.voxels):
                if key == name or key.startswith(name + "/"):
                    del self.voxels[key]
            for key in list(self.previous_poses):
                if key == name or key.startswith(name + "/") or TRAJECTORIES[key][0] == name:
                    del self.previous_poses[key]
        elif kind == VIZ_KIND_CAMERA:
            self._calibration(name, msg)
        else:
            raise ValueError(f"unsupported viz kind {kind}")

    def log_text(self, msg):
        if not math.isfinite(msg.sim_time) or not 0 <= msg.wall_nanos < 1_000_000_000:
            raise ValueError("invalid log timestamp")
        tag = bytes(msg.tag[:count(msg.tag_len, len(msg.tag))]).decode(errors="replace") or "unknown"
        text = bytes(msg.text[:count(msg.text_len, len(msg.text))]).decode(errors="replace")
        self.rec.reset_time()
        if msg.sim_time >= 0:
            self.rec.set_time("sim_time", duration=float(msg.sim_time))
        else:
            self.rec.set_time("wall_time", timestamp=float(msg.wall_secs) + float(msg.wall_nanos) * 1e-9)
        self.rec.log(f"logs/{tag}", rr.TextLog(text, level={1: "ERROR", 2: "WARN", 3: "INFO", 4: "DEBUG", 5: "TRACE"}.get(msg.log_level, "INFO")), strict=True)

    def _calibration(self, name, msg):
        if name not in ("cameras/left", "cameras/right"):
            raise ValueError("unknown calibrated camera")
        value = (*msg.xyz, *msg.quat_xyzw, *msg.scalars, *msg.points[0][:2])
        width, height = msg.points[0][:2]
        if (msg.scalar_count != 4 or msg.point_count != 1
                or not np.isfinite(value).all() or min(msg.scalars[:2]) <= 0
                or width <= 0 or height <= 0 or width != int(width) or height != int(height)
                or abs(np.linalg.norm(msg.quat_xyzw) - 1) > 1e-5):
            raise ValueError("invalid camera calibration")
        if name in self.calibrations:
            if self.calibrations[name] != value:
                raise ValueError("calibration changed within recording; start a new session")
            return
        self.calibrations[name] = value
        fx, fy, cx, cy = msg.scalars
        self.rec.log(name, rr.Pinhole(focal_length=[fx, fy],
                                     principal_point=[cx + PIXEL_CENTER_OFFSET, cy + PIXEL_CENTER_OFFSET],
                                     resolution=list(msg.points[0][:2]), camera_xyz=rr.ViewCoordinates.RDF),
                     rr.Transform3D(translation=list(msg.xyz), quaternion=list(msg.quat_xyzw)),
                     rr.AnyValues(sensor_intrinsics=list(msg.scalars), display_pixel_offset=PIXEL_CENTER_OFFSET),
                     static=True, strict=True)

    def _voxel(self, name, msg):
        if self.voxel_sequence is not None and msg.sequence != self.voxel_sequence + 1:
            raise ValueError("voxel sequence gap or publisher restart; recording is incomplete")
        self.voxel_sequence = msg.sequence
        n = count(msg.voxel_count, len(msg.voxels))
        incoming = {tuple(p) for p in msg.voxels[:n]}
        geometry = tuple(msg.voxel_size), tuple(msg.voxel_origin)
        if not np.isfinite(geometry).all() or min(geometry[0]) <= 0:
            raise ValueError("invalid voxel geometry")
        max_chunks = math.ceil(self.options.capture.max_voxels / len(msg.voxels))
        if not 1 <= msg.chunk_count <= max_chunks or msg.chunk_index >= msg.chunk_count:
            raise ValueError("invalid voxel chunk bounds")
        key = name, msg.timestamp, msg.kind, geometry, msg.chunk_count
        if msg.chunk_index == 0:
            if self.voxel_pending is not None:
                raise ValueError("unfinished voxel update replaced by another update")
            self.voxel_pending = [key, 0, set()]
        if (self.voxel_pending is None or self.voxel_pending[0] != key
                or self.voxel_pending[1] != msg.chunk_index):
            raise ValueError("voxel chunks missing or out of order")
        self.voxel_pending[2].update(incoming)
        self.voxel_pending[1] += 1
        if len(self.voxel_pending[2]) > self.options.capture.max_voxels:
            raise ValueError("voxel update budget exceeded")
        if self.voxel_pending[1] != msg.chunk_count:
            return
        incoming = self.voxel_pending[2]
        self.voxel_pending = None
        if name not in self.voxels:
            if len(self.voxels) >= self.options.capture.max_entities:
                raise ValueError("voxel entity budget exceeded")
            if msg.kind != VIZ_KIND_VOXELS:
                raise ValueError("voxel delta arrived before its baseline")
            self.voxels[name] = [set(), geometry, msg.timestamp, True, float("-inf")]
        state = self.voxels[name]
        if msg.timestamp < state[2]:
            raise ValueError("out-of-order voxel delta")
        if msg.kind != VIZ_KIND_VOXELS and geometry != state[1]:
            raise ValueError("voxel geometry changed without a baseline")
        if msg.kind == VIZ_KIND_VOXELS:
            state[0] = incoming
        elif msg.kind == VIZ_KIND_VOXELS_ADD:
            state[0].update(incoming)
        else:
            state[0].difference_update(incoming)
        if len(state[0]) > self.options.capture.max_voxels:
            raise ValueError("voxel cache budget exceeded")
        state[1:4] = geometry, msg.timestamp, True

    def tick(self, *, final=False):
        now = time.monotonic()
        for name, state in self.voxels.items():
            values, (size, origin), stamp, dirty, last = state
            hz = self.options.capture.voxel_hz
            if not (hz and dirty and (final or now - last >= 1 / hz)):
                continue
            value = rr.VoxelGridMap(np.asarray(sorted(values), dtype=np.int32).reshape(-1, 3),
                                   size, translation=origin)
            at(self.rec, stamp)
            self.rec.log(name, value, strict=True)
            state[3:5] = False, now
        if final or now - self.last_flush >= self.options.flush_seconds:
            # flush 在 writer 线程执行，接收端只受自己的有界队列约束。
            self.rec.flush(timeout_sec=self.options.shutdown_seconds)
            self.last_flush = now
        if final and self.voxel_pending is not None:
            raise ValueError("recording ended during a voxel update")
