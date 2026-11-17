"""Validate the exported INT8 detector and metric corners on rendered CIF frames.

Release render/landing/mimir binaries and RMUC assets are required. Runtime
evidence goes to Mimir; exported image artifacts go to a fresh output directory.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import iceoryx2 as iox2
import numpy as np
from PIL import Image
from firefly_mujoco import DownImageMessage, OdomMessage, TraceContext, LogMessage
from firefly_mujoco.scene import resolve
from firefly_workspace import workspace_root
from firefly_workspace.process import ManagedProcess, isolated_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = workspace_root(Path(__file__))
    args.output.mkdir(parents=True, exist_ok=False)
    with isolated_run(root):
        env = {**os.environ, "RUST_LOG": "info"}
        recorder = ManagedProcess([str(root/"target/release/mimir")], cwd=root, env=env)
        render = None
        try:
            recording = recorder.wait_recording()
            iox2.set_log_level(iox2.LogLevel.Error)
            node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            def port(name, payload, publish=False, log=False):
                builder = node.service_builder(iox2.ServiceName.new(name)).publish_subscribe(payload).user_header(TraceContext)
                if log:
                    builder = builder.subscriber_max_buffer_size(256).max_publishers(10)
                service = builder.open_or_create()
                return service.publisher_builder().create() if publish else service.subscriber_builder().create()
            poses = port("Firefly/Offline/GroundTruth",OdomMessage,True)
            images = port("Firefly/Offline/CameraDown",DownImageMessage)
            recorded_images = port("Firefly/CameraDown",DownImageMessage,True)
            truth = port("Firefly/GroundTruth",OdomMessage,True)
            logs = port("Firefly/Log",LogMessage,True,True)
            def event(text, stamp, sequence):
                print(text,flush=True)
                message = LogMessage(sim_time=stamp,log_level=3,sequence=sequence)
                message.wall_secs,message.wall_nanos=divmod(time.time_ns(),10**9)
                message.tag[:7],message.tag_len=b"landing",7
                text=text.encode()[:512]
                message.text[:len(text)],message.text_len=text,len(text)
                logs.loan_uninit().write_payload(message).send()
            render = ManagedProcess([str(root/"target/release/render"),"--offline"],cwd=root,env=env)
            stamp, sequence = 0., 0
            failures=[]
            for team in ("red","blue"):
                x,y,z = resolve().hangar(team)
                for height in (1.,1.5):
                    for dx,dy in ((0.,0.),(.08,-.06)):
                        start_stamp = stamp
                        deadline = time.monotonic()+180.
                        next_send = 0.
                        frame = None
                        while time.monotonic()<deadline:
                            if render.process.poll() is not None:
                                raise RuntimeError("renderer exited: "+" | ".join(render.tail))
                            if time.monotonic()>=next_send:
                                stamp = round(stamp+.2,6)
                                pose=OdomMessage(timestamp=stamp,position_x=x+dx,position_y=y+dy,position_z=z+height,quat_w=1.,is_initialized=True)
                                poses.loan_uninit().write_payload(pose).send()
                                truth.loan_uninit().write_payload(pose).send()
                                next_send = time.monotonic()+.2
                            while (sample:=images.receive()) is not None:
                                candidate=DownImageMessage.from_buffer_copy(sample.payload().contents)
                                if candidate.timestamp>start_stamp:
                                    pixels=np.ctypeslib.as_array(candidate.data).reshape(288,352)
                                    if pixels.std()>10:
                                        frame=candidate
                            if frame is not None:
                                break
                            time.sleep(.01)
                        if frame is None:
                            raise TimeoutError("no textured down frame: "+" | ".join(render.tail))
                        recorded_images.loan_uninit().write_payload(frame).send()
                        path=args.output/f"{team}_{height:g}_{dx:g}.png"
                        Image.fromarray(np.ctypeslib.as_array(frame.data).reshape(288,352)).save(path)
                        result=subprocess.run([str(root/"target/release/landing"),"--simulation","--image",str(path.resolve())],cwd=root,capture_output=True,text=True,check=True)
                        detections=json.loads(result.stdout.strip().splitlines()[-1])
                        expected=np.array([dy,dx-.03,height-.001])
                        errors=[float(np.linalg.norm(np.array(d["camera_translation"])-expected)) for d in detections if d["camera_translation"] is not None]
                        error=min(errors,default=float("inf"))
                        event(f"landing image={path.name} detections={len(detections)} metric_error_m={error:.6f}",frame.timestamp,sequence)
                        sequence+=1
                        if error>.04:
                            failures.append(f"{path.name}: metric target error {error:.4f}m; detections={detections}")
            if failures:
                raise AssertionError("\n".join(failures))
            print(f"landing image validation passed; recording: {recording}")
        finally:
            if render is not None:
                code=render.stop(30.)
                if code not in (0,-2):
                    raise RuntimeError(f"renderer cleanup failed: {code}")
            if recorder.stop(30.):
                raise RuntimeError("recorder did not close cleanly")


if __name__ == "__main__":
    main()
