"""Run native SIL/USB HIL missions; Mimir owns all runtime evidence."""

from pathlib import Path
from firefly_workspace import workspace_root
from firefly_workspace.game import load_team
from flight_mission.config import parse_args
from flight_mission.runtime import FlightRunner


def main(argv=None, *, hil=None, mission=None):
    options = parse_args(argv, hil=hil, mission=mission)
    root = workspace_root(Path(__file__))
    return FlightRunner(options, root, load_team()).run()


if __name__ == "__main__":
    raise SystemExit(main())
