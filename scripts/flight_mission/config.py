"""Validated immutable mission inputs and legal flight phase transitions."""

import argparse
from dataclasses import dataclass
from enum import StrEnum
import math
import sys

CUBOID_BASE = 1.3
CUBOID_ROOF = CUBOID_BASE + 2.0


@dataclass(frozen=True)
class MissionPlan:
    hil: bool
    mission: str
    hover_seconds: float
    goal: tuple[float, float] | None
    shape: str | None
    waypoint: tuple[tuple[float, float], ...] | None
    side: float | None
    altitude: float
    return_offset: tuple[float, float]
    precision_land: bool
    acquire_timeout: float
    seed: int
    pass_at: tuple[int, ...]


class ArrivalMode(StrEnum):
    STOP = "stop"
    PASS = "pass"


@dataclass(frozen=True)
class SystemRun:
    missions: tuple[MissionPlan, ...]

    def __post_init__(self):
        if not self.missions:
            raise ValueError("select at least one mission to fly")
        if len({(m.hil, m.seed) for m in self.missions}) != 1:
            raise ValueError("all missions share one transport and simulation seed")


@dataclass(frozen=True)
class Waypoint:
    offset: tuple[float, float, float]
    arrival: ArrivalMode


class Phase(StrEnum):
    STARTUP = "startup"
    TAKEOFF = "takeoff"
    WAYPOINT = "waypoint"
    WAYPOINT_HOVER = "waypoint_hover"
    RETURN = "return"
    ACQUIRE = "acquire"
    LAND = "land"
    COMPLETE = "complete"

    def advance(self, following: "Phase", *, passed=False) -> "Phase":
        if passed and self == Phase.WAYPOINT and following in (Phase.WAYPOINT, Phase.RETURN):
            return following
        if following not in _TRANSITIONS[self]:
            raise ValueError(f"invalid mission phase transition: {self} -> {following}")
        return following


_TRANSITIONS = {
    Phase.STARTUP: (Phase.TAKEOFF,),
    Phase.TAKEOFF: (Phase.WAYPOINT, Phase.ACQUIRE, Phase.LAND),
    Phase.WAYPOINT: (Phase.WAYPOINT_HOVER,),
    Phase.WAYPOINT_HOVER: (Phase.WAYPOINT, Phase.RETURN),
    Phase.RETURN: (Phase.ACQUIRE, Phase.LAND),
    Phase.ACQUIRE: (Phase.LAND,),
    Phase.LAND: (Phase.COMPLETE,),
    Phase.COMPLETE: (),
}


def parse_args(argv=None, *, hil=None, mission=None) -> MissionPlan:
    parser = argparse.ArgumentParser(
        description="Fly a hover or waypoint mission and return to the selected hangar.",
        epilog="Separate successive missions with --next. --hil and --seed apply to the entire run.",
    )
    if hil is None:
        parser.add_argument("--hil", action="store_true")
    else:
        parser.set_defaults(hil=hil)
    if mission is None:
        parser.add_argument("--mission", choices=["hover", "roundtrip"], default="roundtrip")
    else:
        parser.set_defaults(mission=mission)
    parser.add_argument(
        "--hover-seconds", type=float, help="hover duration (hover: 20s; cuboid: 1s; other waypoints: 3s)"
    )
    route = parser.add_mutually_exclusive_group()
    route.add_argument(
        "--goal",
        type=float,
        nargs=2,
        metavar=("DX", "DY"),
        help="single waypoint offset in odom metres (red: 8,0; blue: -8,0)",
    )
    route.add_argument(
        "--shape",
        choices=("square", "triangle", "cuboid"),
        help="field-side polygon or 3D cuboid wireframe",
    )
    route.add_argument(
        "--waypoint",
        type=float,
        nargs=2,
        action="append",
        metavar=("DX", "DY"),
        help="repeat for ordered offsets from takeoff in odom metres",
    )
    parser.add_argument(
        "--side", type=float, help="side length in metres (square: 4; triangle: 2; cuboid long edge: 3)"
    )
    parser.add_argument(
        "--altitude", type=float, help="takeoff and return height in metres (hover: 1; cuboid: 1.9; other roundtrips: 2)"
    )
    parser.add_argument(
        "--return-offset",
        type=float,
        nargs=2,
        default=[0.12, -0.08],
        metavar=("DX", "DY"),
        help="return near takeoff before visual alignment, in odom metres",
    )
    parser.add_argument(
        "--precision-land",
        action="store_true",
        help="finish with fresh down-camera acquisition and precision landing",
    )
    parser.add_argument(
        "--acquire-timeout",
        type=float,
        default=12.0,
        help="visual reacquisition deadline, in wall seconds",
    )
    parser.add_argument("--seed", type=int, default=20260930)
    parser.add_argument("--pass-at", type=int, action="append", metavar="N",
                        help="1-based waypoint to fly through; repeat for multiple points (square defaults to point 1)")
    args = parser.parse_args(argv)
    if args.mission == "hover":
        dwell, height = 20., 1.
    elif args.shape == "cuboid":
        dwell, height = 1., 1.9
    else:
        dwell, height = 3., 2.
    if args.hover_seconds is None:
        args.hover_seconds = dwell
    if args.altitude is None:
        args.altitude = height
    if args.side is not None and args.shape is None:
        parser.error("--side requires --shape")
    if args.mission == "hover" and (args.goal is not None or args.shape or args.waypoint):
        parser.error("waypoints and shapes require --mission roundtrip")
    if args.shape and args.side is None:
        args.side = {"square": 4.0, "triangle": 2.0, "cuboid": 3.0}[args.shape]
    values = [
        *(args.goal or []),
        *(value for point in args.waypoint or [] for value in point),
        *([] if args.side is None else [args.side]),
        *args.return_offset,
        args.altitude,
        args.hover_seconds,
        args.acquire_timeout,
    ]
    if not all((math.isfinite(value) for value in values)):
        parser.error("mission parameters must be finite")
    if not 0 < args.hover_seconds <= 600 or not 0.5 <= args.altitude <= 2.0:
        parser.error("hover duration must be in (0,600] and altitude in [0.5,2] metres")
    if not 1 <= args.acquire_timeout <= 60 or math.hypot(*args.return_offset) > 0.3:
        parser.error("acquisition timeout must be in [1,60] seconds and return offset within 0.3m")
    if (
        args.mission == "roundtrip"
        and args.goal is not None
        and (not 1.0 <= math.hypot(*args.goal) <= 10.0)
    ):
        parser.error("roundtrip waypoint distance must be in [1,10] metres")
    if args.shape and (not 1.0 <= args.side <= 4.0):
        parser.error("shape side length must be in [1,4] metres")
    if args.waypoint:
        if len(args.waypoint) > 16 or not 1.0 <= math.hypot(*args.waypoint[0]) <= 10.0:
            parser.error("provide 1..16 waypoints; the first must be 1..10 metres from takeoff")
        previous = [0.0, 0.0]
        for point in args.waypoint:
            if math.hypot(*point) > 10.0 or not 0.5 <= math.dist(previous, point) <= 10.0:
                parser.error(
                    "waypoints must stay within 10m of takeoff, with consecutive legs in [0.5,10]m"
                )
            previous = point
    args.goal = tuple(args.goal) if args.goal is not None else None
    args.return_offset = tuple(args.return_offset)
    args.waypoint = tuple(map(tuple, args.waypoint)) if args.waypoint else None
    args.pass_at = tuple(args.pass_at or ((1,) if args.shape == "square" else ()))
    options = MissionPlan(**vars(args))
    if any(not 1 <= index <= len(route_points(options, "red")) for index in options.pass_at):
        parser.error("--pass-at must identify a waypoint on this route")
    return options


def parse_run(argv=None, *, hil=None, mission=None) -> SystemRun:
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--seed", type=int, default=20260930)
    if hil is None:
        parser.add_argument("--hil", action="store_true")
    else:
        parser.set_defaults(hil=hil)
    run, rest = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    groups = [[]]
    for token in rest:
        if token == "--next":
            groups.append([])
        else:
            groups[-1].append(token)
    if len(groups) > 1 and not all(groups):
        parser.error("--next requires a mission on either side")
    return SystemRun(tuple(parse_args([*group, "--seed", str(run.seed)], hil=run.hil, mission=mission)
                           for group in groups))


def default_goal(team):
    """RMUC demonstration routes head towards the central field from either hangar."""
    return (8.0, 0.0) if team == "red" else (-8.0, 0.0)


def route_points(args, team):
    """Declared offsets in initial odom axes. Planar routes inherit takeoff altitude."""
    if args.mission == "hover":
        return ()
    if args.waypoint:
        return args.waypoint
    if args.shape:
        side = args.side
        direction = 1.0 if team == "red" else -1.0
        if args.shape == "cuboid":
            length = direction * side
            width = length * 2 / 3
            corners = ((length, 0.), (2 * length, 0.), (2 * length, width), (length, width))
            roof = tuple((*xy, CUBOID_ROOF) for xy in corners)
            base = tuple((*xy, CUBOID_BASE) for xy in corners)
            a, b, c, d = range(4)
            return (roof[a], roof[b], roof[c], roof[d], roof[a],
                    base[a], base[b], base[c], base[d], base[a],
                    base[b], roof[b], roof[c], base[c], base[d], roof[d])
        points = (
            [[side, 0.0], [2 * side, 0.0], [2 * side, side], [side, side], [side, 0.0]]
            if args.shape == "square"
            else [
                [side, 0.0],
                [2 * side, 0.0],
                [1.5 * side, math.sqrt(3.0) * side / 2.0],
                [side, 0.0],
            ]
        )
        return tuple((direction * x, direction * y) for x, y in points)
    return (args.goal if args.goal is not None else default_goal(team),)


def route_waypoints(options, team):
    return tuple(
        Waypoint(offset if len(offset) == 3 else (*offset, options.altitude),
                 ArrivalMode.PASS if index in options.pass_at else ArrivalMode.STOP)
        for index, offset in enumerate(route_points(options, team), start=1)
    )
