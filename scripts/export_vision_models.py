#!/usr/bin/env python3
"""Export official ALIKED-N16 / LightGlue weights, with numerical parity checks.

Use the isolated models/vision-export/.venv described in docs/how_to_run.md.
The source checkout is pinned; outputs and provenance live under models/.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
from pathlib import Path
import subprocess
import sys
import types
import urllib.request

import numpy as np
import onnx
import onnxruntime as ort
import torch
import torch.nn.functional as F
import torchvision

ROOT = Path(__file__).resolve().parents[1]
REVISION = "eb42fee2d71449efb0aa5c10549752b5d75384d8"
K = 512
log = logging.getLogger(__name__)


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def deform_forward(self, x):
    """Single-group 3x3 DCN expressed as bilinear sampling and weighted sum.

    torchvision offsets are interleaved (dy, dx), with zero padding.
    align_corners=True maps integer pixel centers to the normalized grid.
    """
    b, c, h, w = map(int, x.shape)
    offset = self.offset_conv(x).clamp(-max(h, w) / 4., max(h, w) / 4.)
    oh, ow = map(int, offset.shape[-2:])
    yy, xx = torch.meshgrid(torch.arange(oh, device=x.device), torch.arange(ow, device=x.device), indexing="ij")
    values = []
    for ky in range(3):
        for kx in range(3):
            j = ky * 3 + kx
            y = yy + ky - self.regular_conv.padding[0] + offset[:, 2 * j]
            z = xx + kx - self.regular_conv.padding[1] + offset[:, 2 * j + 1]
            grid = torch.stack((2 * z / (w - 1) - 1, 2 * y / (h - 1) - 1), -1)
            values.append(F.grid_sample(x, grid, mode="bilinear", padding_mode="zeros", align_corners=True))
    samples = torch.stack(values, 2).reshape(b, c * 9, oh, ow)
    return F.conv2d(samples, self.regular_conv.weight.reshape(-1, c * 9, 1, 1), self.regular_conv.bias)


class Extractor(torch.nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, image):
        out = self.model({"image": image})
        return out["keypoints"], out["descriptors"], out["keypoint_scores"]


class Matcher(torch.nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, k0, d0, k1, d1, s0, s1):
        out = self.model({"image0": {"keypoints": k0, "descriptors": d0, "image_size": s0},
                          "image1": {"keypoints": k1, "descriptors": d1, "image_size": s1}})
        return out["matches0"], out["matches1"], out["matching_scores0"], out["matching_scores1"]


def export_check(model, args, path, inputs, outputs):
    torch.onnx.export(model, args, str(path), input_names=inputs, output_names=outputs,
                      opset_version=18, dynamo=isinstance(model, Matcher), external_data=False)
    onnx.checker.check_model(str(path))
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
    with torch.no_grad():
        expected = model(*args)
    actual = session.run(None, dict(zip(inputs, [a.numpy() for a in args])))
    errors = {}
    for name, a, e in zip(outputs, actual, expected):
        e = e.numpy()
        if np.issubdtype(e.dtype, np.integer):
            np.testing.assert_array_equal(a, e)
        else:
            np.testing.assert_allclose(a, e, atol=2e-4, rtol=2e-4)
        errors[name] = float(np.max(np.abs(a - e)))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "models/vision-export/LightGlue")
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING)
    log.setLevel(logging.INFO)
    if not args.source.exists():
        subprocess.run(["git", "clone", "https://github.com/cvg/LightGlue.git", str(args.source)], check=True)
        subprocess.run(["git", "-C", str(args.source), "checkout", "--detach", REVISION], check=True)
    revision = subprocess.check_output(["git", "-C", str(args.source), "rev-parse", "HEAD"], text=True).strip()
    if revision != REVISION or subprocess.check_output(["git", "-C", str(args.source), "status", "--porcelain"]):
        raise ValueError("LightGlue checkout must be clean at the pinned revision")
    sys.path.insert(0, str(args.source.resolve()))
    from lightglue import ALIKED, LightGlue
    from lightglue.aliked import DeformableConv2d

    torch.set_num_threads(1)
    torch.manual_seed(0)
    torch.hub.set_dir(str(ROOT / "models/vision-export/weights"))
    checkpoints = ROOT / "models/vision-export/weights/checkpoints"
    checkpoints.mkdir(parents=True, exist_ok=True)
    sources = {
        "aliked-n16.pth": ("https://github.com/Shiaoming/ALIKED/raw/main/models/aliked-n16.pth", "5be8704840ed662d9d8c561bf7279c222092674e7eb05fd0feab94899e9d82f2"),
        "aliked_lightglue_v0-1_arxiv.pth": ("https://github.com/cvg/LightGlue/releases/download/v0.1_arxiv/aliked_lightglue.pth", "d975e965b105311a6143194852297dff4f02aea5cc2e10cecfed966ca0e22503"),
    }
    for name, (url, expected) in sources.items():
        path = checkpoints / name
        if not path.exists():
            partial = path.with_suffix(".download")
            urllib.request.urlretrieve(url, partial)
            if digest(partial) != expected:
                raise ValueError(f"checkpoint checksum mismatch: {name}")
            partial.replace(path)
        if digest(path) != expected:
            raise ValueError(f"checkpoint checksum mismatch: {name}")
    extractor = ALIKED(model_name="aliked-n16", max_num_keypoints=K, detection_threshold=-1).eval()
    matcher = LightGlue(features="aliked", flash=False, depth_confidence=-1, width_confidence=-1).eval()
    from lightglue.lightglue import Attention
    for layer in matcher.modules():
        if isinstance(layer, Attention):
            layer.has_sdp = False
    parity = {}
    with torch.no_grad():
        for name, layer in extractor.named_modules():
            if isinstance(layer, DeformableConv2d):
                assert not layer.mask and layer.regular_conv.stride == (1, 1)
                probe = torch.randn(1, layer.regular_conv.in_channels, 13, 17)
                expected = layer(probe)
                layer.forward = types.MethodType(deform_forward, layer)
                actual = layer(probe)
                torch.testing.assert_close(actual, expected, atol=1e-5, rtol=1e-4)
                parity[name] = float((actual - expected).abs().max())
        image = torch.rand(1, 1, 240, 320).expand(-1, 3, -1, -1).contiguous()
        extraction = Extractor(extractor).eval()
        feature = extraction(image)
        # Different images exercise unmatched as well as matched outputs.
        shifted = extraction(torch.roll(image, 7, -1))
        size = torch.tensor([[320, 240]], dtype=torch.int64)
        pair = (feature[0], feature[1], shifted[0], shifted[1], size, size)
        a = ROOT / "models/aliked-n16-k512.onnx"
        m = ROOT / "models/lightglue-aliked-k512.onnx"
        parity["extractor_onnx"] = export_check(extraction, (image,), a, ["image"], ["keypoints", "descriptors", "scores"])
        parity["matcher_onnx"] = export_check(Matcher(matcher).eval(), pair, m,
                                              ["k0", "d0", "k1", "d1", "s0", "s1"],
                                              ["matches0", "matches1", "scores0", "scores1"])
    weights = {p.name: digest(p) for p in (ROOT / "models/vision-export/weights/checkpoints").glob("*.pth")}
    manifest = {"source": "https://github.com/cvg/LightGlue", "revision": revision,
                "extractor": "aliked-n16", "keypoints": K, "descriptor_dim": 128,
                "image_size": [320, 240], "opset": 18, "pruning": False, "early_stopping": False,
                "weights_sha256": weights, "artifacts": {p.name: digest(p) for p in [a, m]},
                "parity_max_absolute_error": parity,
                "versions": {"torch": torch.__version__, "torchvision": torchvision.__version__, "onnx": onnx.__version__, "onnxruntime": ort.__version__}}
    (ROOT / "models/vision_models.json").write_text(json.dumps(manifest, indent=2) + "\n")
    log.info("Verified models and provenance: models/vision_models.json")


if __name__ == "__main__":
    main()
