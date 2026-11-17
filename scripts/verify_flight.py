"""Independently grade a closed SIL/HIL RRD; ground truth is used only here."""

import argparse
import json
from pathlib import Path
from flight_mission.verification import verify_recording


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    args = parser.parse_args()
    try:
        report = verify_recording(args.recording)
    except Exception as error:
        report = {"passed": False, "failures": [f"invalid recording: {error}"], "metrics": {}}
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
