"""Prepare calibration tensors from a ZIP of images, or compile the fixed 320 depth model inside ACUITY."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile


def prepare(archive_path):
    import cv2
    import numpy as np

    directory = Path("calibration")
    directory.mkdir(exist_ok=True)
    with zipfile.ZipFile(archive_path) as archive:
        names = sorted(n for n in archive.namelist()
                       if Path(n).suffix.lower() in {".jpg", ".jpeg", ".png"}
                       and not Path(n).name.startswith("._"))
        if not names:
            raise ValueError("No calibration images in archive")
        rows = []
        for index, name in enumerate(names):
            image = cv2.imdecode(np.frombuffer(archive.read(name), np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"Cannot decode calibration image: {name}")
            rgb = cv2.cvtColor(cv2.resize(image, (320, 320)), cv2.COLOR_BGR2RGB)
            tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255
            path = directory / f"{index:05}.npy"
            np.save(path, tensor)
            rows.append(str(path))
    Path("dataset.txt").write_text("\n".join(rows) + "\n")


def compact_head(graph):
    layers = graph["Layers"]
    output, = (layer for layer in layers.values() if layer["op"] == "output")

    def upstream(layer):
        port, = layer["inputs"]
        if not port.startswith("@"):
            raise ValueError("Unexpected ACUITY connection")
        return port[1:].split(":")[0]

    permute = layers[upstream(output)]
    resize_id = upstream(permute)
    resize = layers[resize_id]
    if (permute["op"] != "permute" or resize["op"] != "image_resize"
            or resize["parameters"] != dict(type="bilinear", align_corners=True, half_pixel=False, new_size=[320, 320])
            or layers[upstream(resize)]["op"] != "exp"
            or sum(f"@{resize_id}:out0" in layer["inputs"] for layer in layers.values()) != 1):
        raise ValueError("Expected a single aligned resize after metric-depth calibration")
    permute["inputs"] = resize["inputs"]
    del layers[resize_id]


def compile_model():
    import yaml

    def pegasus(*args):
        subprocess.run([sys.executable, str(Path(os.environ["ACUITY_PATH"]) / "pegasus.py"), *args], check=True)

    pegasus("import", "onnx", "--model", "yolo26n-depth-320.onnx", "--output-model", "depth.json", "--output-data", "depth.data")
    pegasus("generate", "inputmeta", "--model", "depth.json", "--separated-database", "--input-meta-output", "input.yml")
    meta = yaml.safe_load(Path("input.yml").read_text())
    db, = meta["input_meta"]["databases"]
    port, = db["ports"]
    if port["shape"] != [1, 3, 320, 320] or port["layout"] != "nchw":
        raise ValueError("Unexpected depth input")
    db["path"] = "dataset.txt"
    port["category"] = "tensor"
    port["preprocess"].update(reverse_channel=False, scale=[1.] * 3, mean=[0] * 3)
    Path("input.yml").write_text(yaml.safe_dump(meta))
    pegasus("generate", "postprocess-file", "--model", "depth.json", "--postprocess-file-output", "output.yml")
    common = ["--model-data", "depth.data", "--model-quantize", "depth.quantize", "--with-input-meta", "input.yml"]
    frames = Path(db["path"]).read_text().splitlines()
    if not frames:
        raise ValueError("Empty calibration dataset")
    pegasus("quantize", "--model", "depth.json", *common, "--device", "CPU", "--quantizer", "perchannel_symmetric_affine",
            "--qtype", "int8", "--rebuild", "--batch-size", "1", "--iterations", str(len(frames)))
    graph = json.loads(Path("depth.json").read_text())
    compact_head(graph)
    native = Path("native")
    native.mkdir(exist_ok=True)
    (native / "depth.json").write_text(json.dumps(graph))
    pegasus("export", "ovxlib", "--model", "native/depth.json", *common, "--dtype", "quantized", "--postprocess-file", "output.yml",
            "--target-ide-project", "linux64", "--pack-nbg-unify", "--viv-sdk", os.environ["VIV_SDK"],
            "--optimize", "VIP9000NANODI_PLUS_PID0X1000003B", "--output-path", "native/wksp/depth")
    shutil.copyfile(native / "wksp_nbg_unify/network_binary.nb", "yolo26n-depth-320-int8.nb")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", type=Path, metavar="IMAGE_ZIP", help="write normalized calibration inputs on the export host")
    args = parser.parse_args()
    if args.prepare:
        prepare(args.prepare)
    else:
        compile_model()
