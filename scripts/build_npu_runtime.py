"""Build the strict VSINPU runtime bundle on aarch64 Linux with the board's SDK.

The SDK and TIM-VX installation must match the deployed A733 kernel driver.
This builds the native EP; landing performs the model compatibility check.
"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def revision(source: Path, expected: str) -> str:
    actual = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, text=True).strip()
    if actual != expected or subprocess.check_output(["git", "status", "--porcelain"], cwd=source):
        raise ValueError(f"Expected clean source at {expected}: {source}")
    return actual


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ort-source", type=Path, required=True)
    parser.add_argument("--ort-revision", required=True, help="full immutable git commit")
    parser.add_argument("--tim-vx-install", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--sdk-version", required=True, help="board SDK/driver release identity")
    parser.add_argument("--build-dir", type=Path, default=Path("target/npu-runtime"))
    parser.add_argument("--output", type=Path, default=Path("models/npu"))
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    if platform.system() != "Linux" or platform.machine() not in ("aarch64", "arm64"):
        parser.error("Run in an aarch64 Linux build environment with the target SDK")
    if args.jobs <= 0 or not re.fullmatch(r"[0-9a-f]{40}", args.ort_revision):
        parser.error("Expected positive jobs and a full ORT revision")
    source, timvx, sdk = (p.resolve(strict=True) for p in (args.ort_source, args.tim_vx_install, args.sdk))
    commit = revision(source, args.ort_revision)
    header = (source / "include/onnxruntime/core/session/onnxruntime_c_api.h").read_text()
    if not re.search(r"#define ORT_API_VERSION\s+31\b", header):
        raise ValueError("ort 2.0.0-rc.15 requires ONNX Runtime C API 31")
    if not (timvx / "lib/libtim-vx.so").is_file():
        raise FileNotFoundError("Expected TIM-VX installation with lib/libtim-vx.so")
    if args.output.exists():
        raise FileExistsError(f"Choose a fresh output directory: {args.output}")
    build = args.build_dir.resolve()
    if "tmp" in build.parts or ("target", "debug") == build.parts[-2:]:
        raise ValueError("Use a persistent build directory outside /tmp and target/debug")
    command = [sys.executable, str(source / "tools/ci_build/build.py"), "--build_dir", str(build),
               "--config", "Release", "--update", "--build", "--skip_tests", "--build_shared_lib",
               "--use_vsinpu", "--parallel", str(args.jobs), "--cmake_extra_defines",
               "CMAKE_BUILD_RPATH=$ORIGIN", "CMAKE_INSTALL_RPATH=$ORIGIN"]
    env = os.environ.copy()
    env.update(TIM_VX_INSTALL=str(timvx), VIVANTE_SDK_DIR=str(sdk))
    subprocess.run(command, env=env, cwd=source, check=True)
    output = args.output.resolve()
    output.mkdir(parents=True)
    libraries = [*sorted((build / "Release").glob("libonnxruntime.so*")),
                 *sorted((timvx / "lib").glob("*.so*"))]
    # SDK directories contain vendor userspace libraries, not kernel modules.
    for directory in (sdk / "lib", sdk / "drivers"):
        libraries.extend(sorted(directory.glob("*.so*")))
    files = {}
    originals = {}
    for library in libraries:
        if not library.is_file():
            continue
        destination = output / library.name
        checksum = digest(library)
        if destination.name in files:
            if originals[destination.name] != checksum:
                raise ValueError(f"Conflicting SDK libraries: {library.name}")
            continue
        originals[destination.name] = checksum
        shutil.copyfile(library, destination)
        subprocess.run(["patchelf", "--set-rpath", "$ORIGIN", str(destination)], check=True)
        files[destination.name] = digest(destination)
    runtime = output / "libonnxruntime.so"
    library = ctypes.CDLL(str(runtime))
    getattr(library, "OrtSessionOptionsAppendExecutionProvider_VSINPU")
    library.OrtGetApiBase.restype = ctypes.POINTER(ctypes.c_void_p)
    get_api = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_uint32)(library.OrtGetApiBase().contents.value)
    if not get_api(31):
        raise RuntimeError("Native runtime does not expose C API 31")
    (output / "manifest.json").write_text(json.dumps({
        "schema": 1, "ort_revision": commit, "ort_api": 31, "provider": "VSINPU",
        "sdk_version": args.sdk_version, "files": files,
    }, indent=2) + "\n")
    print(f"VSINPU runtime bundle: {output}; run landing with a real camera frame to verify NPU placement")


if __name__ == "__main__":
    main()
