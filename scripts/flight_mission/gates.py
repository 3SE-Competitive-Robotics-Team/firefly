"""Native-state arrival gates and bounded visual confirmation."""

from dataclasses import dataclass
import math


def at_goal(message, goal):
    position = [message.position_x, message.position_y, message.position_z]
    velocity = [message.velocity_x, message.velocity_y, message.velocity_z]
    return math.dist(position, goal) <= 0.15 and math.hypot(*velocity) <= 0.12


def fresh_target(message, stamp, *, after, session, team):
    return (
        message.valid == 1
        and message.session == session
        and (message.team == team)
        and (after < message.timestamp <= stamp)
        and (stamp - message.timestamp <= 0.25)
        and all((math.isfinite(value) for value in message.position))
        and math.isfinite(message.confidence)
        and (0.5 <= message.confidence <= 1.0)
        and math.isfinite(message.reprojection_px)
        and (0 <= message.reprojection_px <= 2.0)
    )


@dataclass
class Arrival:
    goal: tuple[float, float, float]
    identity: float
    settled_since: float | None = None

    def observe(self, planner, odometry, stamp):
        arrived = (
            planner is not None
            and odometry is not None
            and planner.goal_timestamp == self.identity
            and bool(planner.finished)
            and 0 <= stamp - planner.timestamp <= 0.5
            and at_goal(odometry, self.goal)
        )
        if not arrived:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = stamp
        return self.settled_since is not None and stamp - self.settled_since >= 0.5


@dataclass
class _ConfirmationWindow:
    first: float
    last: float
    count: int = 1


class VisualAcquisition:
    def __init__(self, *, after, session, team):
        self.after, self.session, self.team = after, session, team
        self._window = None

    @property
    def target_time(self):
        if (
            self._window is not None
            and self._window.count >= 3
            and round((self._window.last - self._window.first) * 1e9) >= 150_000_000
        ):
            return self._window.last
        return None

    def observe(self, message, stamp, received_at, now):
        if (
            message is None
            or now - received_at > 0.25
            or not fresh_target(
                message, stamp, after=self.after, session=self.session, team=self.team
            )
        ):
            self._window = None
            return False
        if self._window is None:
            self._window = _ConfirmationWindow(message.timestamp, message.timestamp)
        elif message.timestamp < self._window.last:
            self._window = None
            return False
        elif message.timestamp > self._window.last:
            self._window.last = message.timestamp
            self._window.count = min(self._window.count + 1, 3)
        return self.target_time is not None
