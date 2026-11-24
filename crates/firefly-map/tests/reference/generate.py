"""Compile the pinned C++ mapping functions, without ROS transport.

Run with --upstream <EGO-Planner-v2 checkout> --eigen <Eigen include directory>.
Build artifacts live in target/ego-map-reference. The numerical algorithm bodies
are extracted verbatim, including raycast.cpp; the harness supplies image/clock
types and initial parameter values, not a second implementation of the algorithm.
The reference-boundaries binary applies half-open bounds and clears only evicted
slices. Both binaries read the same operation stream used by cpp_reference.rs.
"""

import argparse
from pathlib import Path
import re
import subprocess

REVISION = "9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010"


def braced(source, start):
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def functions(source, names):
    result = []
    for name in names:
        matches = list(re.finditer(r"^(?:inline )?[^\n]*GridMap::" + name + r"\(", source, re.M))
        if not matches:
            raise ValueError(f"missing upstream function: {name}")
        for match in matches:
            result.append(braced(source, match.start()).replace("GridMap::", ""))
    return "\n".join(result)


def boundary_reference(translation):
    """Change only the two boundary rules in the pinned oracle for causal comparison."""
    for axis in range(3):
        # Upstream clearBuffer loops include the last index on each axis.
        for case, side, step in [(2 * axis, "up", "+ 1"), (2 * axis + 1, "low", "- 1")]:
            bound = f"ringbuffer_{side}bound3i_new({axis})"
            call = f"clearBuffer({case}, {bound});"
            assert translation.count(call) == 1
            translation = translation.replace(call, f"clearBuffer({case}, {bound} {step});")
        for prefix in ["ringbuffer_", "ringbuffer_inf_"]:
            check = f"pos({axis}) > md_.{prefix}upbound3d_({axis})"
            assert translation.count(check) == 1
            translation = translation.replace(check, check.replace(" > ", " >= "))
    return translation


def compile_reference(build, eigen, name, source):
    cpp = build / f"{name}.cpp"
    cpp.write_text(source)
    subprocess.run([
        "c++", "-std=c++17", "-O2", "-ffp-contract=off", "-fsigned-char",
        "-I" + str(eigen), "-I" + str(build), str(cpp), str(build / "raycast.cpp"),
        "-o", str(build / name),
    ], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--eigen", type=Path, required=True)
    args = parser.parse_args()
    revision = subprocess.check_output(
        ["git", "-C", str(args.upstream), "rev-parse", "HEAD"], text=True
    ).strip()
    if revision != REVISION:
        raise ValueError(f"expected {REVISION}, got {revision}")
    package = "swarm-playground/main_ws/src/planner/plan_env"
    # Read committed blobs so uncommitted upstream edits cannot change the oracle.
    def blob(path):
        return subprocess.check_output(
            ["git", "-C", str(args.upstream), "show", f"{REVISION}:{package}/{path}"], text=True
        )
    header = blob("include/plan_env/grid_map.h")
    source = blob("src/grid_map.cpp")
    structs = "\n".join(
        braced(header, header.index("struct " + name)) + ";"
        for name in ("MappingParameters", "MappingData")
    )
    inline = functions(header, [
        "setCacheOccupancy", "changeInfBuf", "globalIdx2BufIdx", "globalIdx2InfBufIdx",
        "BufIdx2GlobalIdx", "infBufIdx2GlobalIdx", "getOccupancy", "getInflateOccupancy",
        "isInBuf", "isInInfBuf", "globalIdx2Pos", "pos2GlobalIdx", "getResolution",
    ])
    methods = functions(source, [
        "updateOccupancyCallback", "fadingCallback", "moveRingBuffer", "projectDepthImage",
        "raycastProcess", "clearAndInflateLocalMap", "initMapBoundary", "clearBuffer",
        "closetPointInMap", "checkDepthOdomNeedupdate",
    ])
    template = Path(__file__).with_name("harness.cpp").read_text()
    translation = template.replace("// UPSTREAM_STRUCTS", structs).replace(
        "// UPSTREAM_METHODS", inline + "\n" + methods
    )
    root = Path(__file__).resolve().parents[4]
    build = root / "target/ego-map-reference"
    (build / "plan_env").mkdir(parents=True, exist_ok=True)
    (build / "plan_env/raycast.h").write_text(blob("include/plan_env/raycast.h"))
    (build / "raycast.cpp").write_text(blob("src/raycast.cpp"))
    compile_reference(build, args.eigen, "reference", translation)
    compile_reference(build, args.eigen, "reference-boundaries", boundary_reference(translation))


if __name__ == "__main__":
    main()
