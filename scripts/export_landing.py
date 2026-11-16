"""Export the trained H detector and calibrate a fixed INT8 QDQ ONNX artifact.

Run with se3_yolo's export environment. A new output directory preserves each
model's training checkpoint, calibration provenance and inference contract.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


CAMERA_SIZE = (352, 288)


def preprocess(path: Path, size: tuple[int, int]):
    import cv2
    import numpy as np

    # Synthetic calibration first samples the selected camera output resolution.
    # Real OV2640 frames replace this input when the module and lens are available.
    rgb = cv2.imread(str(path))
    if rgb is None:
        raise ValueError(f"Cannot read calibration image: {path}")
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, CAMERA_SIZE, interpolation=cv2.INTER_LINEAR)
    target_width, target_height = size
    scale = min(target_width / CAMERA_SIZE[0], target_height / CAMERA_SIZE[1])
    width, height = (round(value * scale) for value in CAMERA_SIZE)
    resized = cv2.resize(gray, (width, height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((target_height, target_width), 114, dtype=np.uint8)
    left, top = (target_width - width) // 2, (target_height - height) // 2
    canvas[top : top + height, left : left + width] = resized
    return np.repeat(canvas[None, None], 3, axis=1).astype(np.float32) / 255


def main():
    import onnx
    import onnxruntime as ort
    from onnxruntime.quantization import (
        CalibrationDataReader, QuantFormat, QuantType, quantize_static,
    )
    from ultralytics import YOLO

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=352)
    parser.add_argument("--height", type=int, default=288)
    args = parser.parse_args()
    size = (args.width, args.height)
    if any(value <= 0 or value > 640 or value % 32 for value in size):
        parser.error("--width and --height must be multiples of 32 in 32..640")
    dataset_manifest = args.dataset / "manifest.json"
    dataset = json.loads(dataset_manifest.read_text())
    if dataset["status"] != "complete" or dataset["names"] != ["h_marker"]:
        raise ValueError("Expected a complete h_marker dataset")
    calibration = [row for row in dataset["frames"] if row["split"] == "train"]
    if not calibration:
        raise ValueError("No training calibration frames")
    for row in calibration:
        if sha256(args.dataset / row["image"]) != row["sha256"]:
            raise ValueError(f"Calibration checksum mismatch: {row['image']}")
    args.output.mkdir(parents=True, exist_ok=False)
    checkpoint = args.output / "source.pt"
    shutil.copyfile(args.checkpoint, checkpoint)
    # The raw single-class head leaves TopK/NMS in bounded CPU postprocessing.
    exported = Path(YOLO(checkpoint).export(
        format="onnx", imgsz=(args.height, args.width), opset=17, simplify=False,
        dynamic=False, batch=1, end2end=False, nms=False,
    ))
    fp32 = args.output / "fp32.onnx"
    exported.rename(fp32)
    graph = onnx.load(fp32)
    onnx.checker.check_model(graph)
    if [item.dim_value for item in graph.graph.input[0].type.tensor_type.shape.dim] != [1, 3, args.height, args.width]:
        raise ValueError("Unexpected model input")
    output_shape = [item.dim_value for item in graph.graph.output[0].type.tensor_type.shape.dim]
    if len(output_shape) != 3 or output_shape[:2] != [1, 5]:
        raise ValueError(f"Expected single-class raw detection head, got {output_shape}")

    class Reader(CalibrationDataReader):
        def __init__(self):
            self.frames = iter(calibration)

        def get_next(self):
            row = next(self.frames, None)
            return None if row is None else {"images": preprocess(args.dataset / row["image"], size)}

    quantized = args.output / "int8.onnx"
    quantize_static(
        str(fp32), str(quantized), Reader(), quant_format=QuantFormat.QDQ,
        activation_type=QuantType.QInt8, weight_type=QuantType.QInt8,
        per_channel=True, op_types_to_quantize=["Conv", "MatMul"],
    )
    graph = onnx.load(quantized)
    if not any(node.op_type == "QuantizeLinear" for node in graph.graph.node):
        raise ValueError("Quantized artifact has no QDQ nodes")
    onnx.checker.check_model(graph)
    session = ort.InferenceSession(str(quantized), providers=["CPUExecutionProvider"])
    output = session.run(None, {"images": preprocess(args.dataset / calibration[0]["image"], size)})[0]
    if list(output.shape) != output_shape:
        raise ValueError("Quantized output shape changed")
    manifest = {
        "schema": 1, "model": quantized.name, "sha256": sha256(quantized),
        "source_sha256": sha256(checkpoint),
        "calibration_sha256": sha256(dataset_manifest),
        "quantization": "int8_qdq", "input": [1, 3, args.height, args.width],
        "camera": list(CAMERA_SIZE),
        "output": "yolo_xywh_score", "classes": ["h_marker"],
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
