"""Prepare the fixed H model and its calibration images for the A733 ACUITY compiler."""
import argparse
import json
from pathlib import Path

import cv2
import onnx


from export_landing import CAMERA_SIZE, camera_gray, sha256


def prepare(model_dir, dataset, output):
    manifest = json.loads((model_dir / "manifest.json").read_text())
    metadata = dataset / "manifest.json"
    if sha256(metadata) != manifest["calibration_sha256"] or sha256(model_dir / "source.pt") != manifest["source_sha256"]:
        raise ValueError("The checkpoint or calibration dataset differs from the ONNX export")
    graph = onnx.load(model_dir / "fp32.onnx")
    shape = [d.dim_value for d in graph.graph.input[0].type.tensor_type.shape.dim]
    if shape != [1, 3, CAMERA_SIZE[1], CAMERA_SIZE[0]] or shape != manifest["input"]:
        raise ValueError(f"Expected the fixed camera resolution, got {shape}")
    head = graph.graph.node[-1]
    if head.op_type != "Concat" or list(head.output) != ["output0"] or len(head.input) != 2:
        raise ValueError("Expected the final single-class box/score concatenation")
    # Separate quantization scales preserve probabilities alongside pixel coordinates.
    count = graph.graph.output[0].type.tensor_type.shape.dim[2].dim_value
    outputs = [onnx.helper.make_tensor_value_info(name, onnx.TensorProto.FLOAT, [1, channels, count])
               for name, channels in zip(head.input, (4, 1))]
    del graph.graph.node[-1]
    del graph.graph.output[:]
    graph.graph.output.extend(outputs)
    onnx.checker.check_model(graph)
    output.mkdir(parents=True, exist_ok=True)
    onnx.save(graph, output / "h.onnx")
    calibration = output / "calibration"
    calibration.mkdir(exist_ok=True)
    rows = []
    for frame in json.loads(metadata.read_text())["frames"]:
        if frame["split"] != "train":
            continue
        image = dataset / frame["image"]
        if sha256(image) != frame["sha256"]:
            raise ValueError(f"Calibration checksum mismatch: {image}")
        path = calibration / f"{len(rows):05}.png"
        if not cv2.imwrite(str(path), cv2.cvtColor(camera_gray(image), cv2.COLOR_GRAY2BGR)):
            raise OSError(path)
        rows.append(str(path.relative_to(output)))
    if not rows:
        raise ValueError("Empty calibration dataset")
    (output / "dataset.txt").write_text("\n".join(rows) + "\n")
    manifest.pop("sha256", None)
    manifest.update(model="h-marker.nb", quantization="viplite_int8")
    (output / "manifest-template.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"{output / 'h.onnx'} with {len(rows)} calibration frames and {count} candidates")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, default=Path("models/h-marker-yolo26n-cif-v1"))
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.model_dir, args.dataset, args.output)
