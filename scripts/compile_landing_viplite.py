"""Run inside the A733 ACUITY image in the directory prepared by export_landing_viplite.py."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


def main():
    tool = Path(os.environ["ACUITY_PATH"]) / "pegasus.py"
    sdk = os.environ["VIV_SDK"]

    def pegasus(*args):
        subprocess.run([sys.executable, str(tool), *args], check=True)

    pegasus("import", "onnx", "--model", "h.onnx", "--output-model", "h.json", "--output-data", "h.data")
    pegasus("generate", "inputmeta", "--model", "h.json", "--separated-database", "--input-meta-output", "input.yml")
    meta = yaml.safe_load(Path("input.yml").read_text())
    port = meta["input_meta"]["databases"][0]["ports"][0]
    if port["shape"] != [1, 3, 288, 352] or port["layout"] != "nchw":
        raise ValueError("Unexpected ACUITY input")
    port["preprocess"].update(mean=[0, 0, 0], scale=[1 / 255] * 3)
    Path("input.yml").write_text(yaml.safe_dump(meta))
    pegasus("generate", "postprocess-file", "--model", "h.json", "--postprocess-file-output", "output.yml")
    common = ["--model", "h.json", "--model-data", "h.data", "--with-input-meta", "input.yml",
              "--model-quantize", "h.quantize"]
    frames = Path("dataset.txt").read_text().splitlines()
    pegasus("quantize", *common, "--device", "CPU", "--quantizer", "perchannel_symmetric_affine",
            "--qtype", "int8", "--rebuild", "--batch-size", "1", "--iterations", str(len(frames)))
    pegasus("export", "ovxlib", *common, "--dtype", "quantized", "--postprocess-file", "output.yml",
            "--target-ide-project", "linux64", "--pack-nbg-unify", "--viv-sdk", sdk,
            "--optimize", "VIP9000NANODI_PLUS_PID0X1000003B", "--output-path", "wksp/h")
    binary = Path("wksp_nbg_unify/network_binary.nb")
    data = binary.read_bytes()
    if data[:4] != b"VPMN" or int.from_bytes(data[8:12], "little") != 0x1000003B:
        raise ValueError("Expected an A733 NBG binary")
    bundle = Path("bundle")
    bundle.mkdir(exist_ok=True)
    manifest = json.loads(Path("manifest-template.json").read_text())
    shutil.copyfile(binary, bundle / manifest["model"])
    (bundle / "viplite.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
