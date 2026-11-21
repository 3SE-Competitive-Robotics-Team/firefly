"""Estimate depth for one saved RGB image and write a float32 NumPy array in meters."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


def resize_depth(depth: np.ndarray, height: int, width: int) -> np.ndarray:
    """Bilinear interpolation with the endpoint alignment used by DepthPredictor."""
    if depth.shape == (height, width):
        return depth
    x = np.linspace(0, depth.shape[1] - 1 if width > 1 else 0, width, dtype=np.float32)
    y = np.linspace(
        0, depth.shape[0] - 1 if height > 1 else 0, height, dtype=np.float32
    )
    ix = x.astype(np.intp)
    iy = y.astype(np.intp)
    dx = (x - ix).astype(np.float32)
    dy = (y - iy).astype(np.float32)[:, None]
    left = depth[:, ix]
    right = depth[:, np.minimum(ix + 1, depth.shape[1] - 1)]
    rows = left + (right - left) * dx
    return rows[iy] + (rows[np.minimum(iy + 1, depth.shape[0] - 1)] - rows[iy]) * dy


def predict(model: Path, image: Path) -> np.ndarray:
    frame = cv2.imread(str(image), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError(f"Cannot decode image: {image}")
    session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    (port,) = session.get_inputs()
    (output,) = session.get_outputs()
    if (len(port.shape) != 4 or port.shape[:2] != [1, 3]
            or not all(isinstance(n, int) and n > 0 for n in port.shape[2:])
            or port.type != "tensor(float)"):
        raise ValueError(f"Unexpected YOLO26 depth input: {port.shape} {port.type}")
    height, width = port.shape[2:]
    rgb = cv2.cvtColor(cv2.resize(frame, (width, height)), cv2.COLOR_BGR2RGB)
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255
    (raw,) = session.run([output.name], {port.name: tensor})
    if not isinstance(raw, np.ndarray):
        raise TypeError("Model output must be a dense tensor")
    if raw.shape[:-2] not in ((1,), (1, 1)) or raw.dtype != np.float32:
        raise ValueError(f"Unexpected depth output: {raw.shape} {raw.dtype}")
    depth = raw.reshape(raw.shape[-2:])
    if not np.isfinite(depth).all() or np.any(depth <= 0):
        raise ValueError("Model returned invalid metric depth")
    return resize_depth(resize_depth(depth, height, width), *frame.shape[:2])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = predict(args.model, args.image)
    with args.output.open("xb") as stream:
        np.save(stream, result, allow_pickle=False)


if __name__ == "__main__":
    main()
