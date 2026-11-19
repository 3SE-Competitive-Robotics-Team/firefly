"""Compatibility entry point for the standalone Rust flight verifier."""
import os
from pathlib import Path
import sys


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    binary = root / "target/release/firefly-verify"
    if binary.is_file():
        os.execv(binary, [str(binary), *sys.argv[1:]])
    manifest = root / "apps/firefly-verify/Cargo.toml"
    os.execvp("cargo", ["cargo", "run", "--quiet", "--release", "--manifest-path", str(manifest),
                        "--bin", "firefly-verify", "--", *sys.argv[1:]])
