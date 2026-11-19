"""Validated immutable mission inputs and legal flight phase transitions."""

import argparse
from dataclasses import dataclass
from enum import StrEnum
import math


@dataclass(frozen=True)
class MissionOptions:
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
class Waypoint:
    offset: tuple[float, float]
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


def parse_args(argv=None, *, hil=None, mission=None) -> MissionOptions:
    parser = argparse.ArgumentParser(
        description="Fly a hover or waypoint mission and return to the selected hangar."
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
        "--hover-seconds", type=float, help="hover duration (hover: 20s; roundtrip waypoint: 3s)"
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
        choices=("square", "triangle"),
        help="closed polygon one side-length ahead of takeoff",
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
        "--side", type=float, help="polygon side length in metres (square: 4; triangle: 2)"
    )
    parser.add_argument(
        "--altitude", type=float, help="height above takeoff (hover: 1m; roundtrip: 2m)"
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
    if args.hover_seconds is None:
        args.hover_seconds = 20.0 if args.mission == "hover" else 3.0
    if args.altitude is None:
        args.altitude = 1.0 if args.mission == "hover" else 2.0
    if args.side is not None and args.shape is None:
        parser.error("--side requires --shape")
    if args.mission == "hover" and (args.goal is not None or args.shape or args.waypoint):
        parser.error("waypoints and shapes require --mission roundtrip")
    if args.shape and args.side is None:
        args.side = 4.0 if args.shape == "square" else 2.0
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
    options = MissionOptions(**vars(args))
    if any(not 1 <= index <= len(route_points(options, "red")) for index in options.pass_at):
        parser.error("--pass-at must identify a waypoint on this route")
    return options


def default_goal(team):
    """RMUC demonstration routes head towards the central field from either hangar."""
    return (8.0, 0.0) if team == "red" else (-8.0, 0.0)


def route_points(args, team):
    """Ordered offsets in initial odom axes; polygons close at their field-side entry."""
    if args.mission == "hover":
        return ()
    if args.waypoint:
        return args.waypoint
    if args.shape:
        side = args.side
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
        direction = 1.0 if team == "red" else -1.0
        return tuple((direction * x, direction * y) for x, y in points)
    return (args.goal if args.goal is not None else default_goal(team),)


def route_waypoints(options, team):
    return tuple(
        Waypoint(offset, ArrivalMode.PASS if index in options.pass_at else ArrivalMode.STOP)
        for index, offset in enumerate(route_points(options, team), start=1)
    )
