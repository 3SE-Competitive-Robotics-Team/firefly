"""Build the shared firefly Rerun layout with visible down-camera landing evidence."""
import argparse
from pathlib import Path

import rerun as rr
import rerun.blueprint as bp


def lines(*, names=None, visible=None):
    return rr.SeriesLines(names=names, visible_series=visible,
                          aggregation_policy=rr.components.AggregationPolicy.MinMax)


def chart(name, origin, *, contents=None, overrides=None):
    return bp.TimeSeriesView(
        name=name, origin=origin, contents=contents or "$origin/**",
        defaults=[lines()], overrides=overrides,
        time_ranges=[bp.VisibleTimeRange("sim_time", start=bp.TimeRangeBoundary(-10_000_000_000),
                                         end=bp.TimeRangeBoundary(0))],
    )


def build():
    cards = bp.Horizontal(*[
        bp.TextDocumentView(name=name, origin=f"/mimir/status/{path}")
        for name, path in [
            ("FC · control", "fc/tick_rate"), ("Camera · published", "camera/publish_rate"),
            ("Camera · latency", "camera/publish_latency"), ("VIO · processing", "vio/processing"),
            ("VIO · posterior age", "vio/posterior_age"), ("VIO · landmarks", "vio/landmarks"),
        ]
    ])
    history = bp.VisibleTimeRanges(timeline="sim_time", start=bp.TimeRangeBoundary(None, kind="infinite"),
                                   end=bp.TimeRangeBoundary(0))
    scene = bp.Spatial3DView(
        name="Map / estimated path / ground truth", origin="/",
        contents=["/gt/**", "/vio/odom", "/vio/traj", "/vio/map/**", "/plan/drone",
                  "/plan/global_path", "/plan/local_traj", "/plan/collision",
                  "/cameras/left", "/cameras/right", "/cameras/down"],
        overrides={path: [rr.LineStrips3D.from_fields(colors=color), history]
                   for path, color in [("/gt/traj", 0x50AAF0FF), ("/vio/traj", 0xFFA040FF),
                                       ("/vio/map/traj", 0xE66EE0FF)]},
    )
    cameras = bp.Tabs(
        bp.Spatial2DView(name="Down camera / landing", origin="/cameras/down"),
        bp.Vertical(
            bp.Spatial2DView(name="Left camera / tracks", origin="/cameras/left"),
            bp.Spatial2DView(name="Right camera / tracks", origin="/cameras/right"), name="Stereo",
        ), active_tab=0, name="Cameras",
    )
    tracking = bp.Horizontal(
        chart("Tracks · IDs", "/vio/diagnostics/tracks"),
        chart("Reprojection RMS · px", "/vio/diagnostics/residual_rms_px"),
        chart("Control tracking error · m", "/fc/debug/position_error_odom"), name="Tracking / control",
    )
    timing = bp.Horizontal(
        chart("Visual processing · ms", "/rootslam/frame_processing_ms"),
        chart("Posterior age · s", "/vio/prediction/posterior_age_s"),
        chart("Render stages · s", "/render/capture/stages_s"), name="Timing",
    )
    estimator = bp.Horizontal(
        chart("Stereo success ratio", "/vio/diagnostics/stereo_ratio"),
        chart("Position correction · m", "/vio/diagnostics/position_delta_m"),
        chart("Velocity · m/s", "/vio/prediction/velocity_body_odom"), name="Estimator",
    )
    planner = bp.Vertical(
        bp.Horizontal(
            chart("FSM · wall Hz", "/planner", contents=["/planner/loop_hz", "/planner/fsm_hz"]),
            chart("FSM · compute ms", "/planner/fsm_timing_ms", overrides={
                "/planner/fsm_timing_ms": lines(visible=[True, False, False])}),
            chart("FSM callback / period · ms", "/planner/fsm_timing_ms", overrides={
                "/planner/fsm_timing_ms": lines(visible=[False, True, True])}),
        ),
        bp.Horizontal(
            chart("Replan · submitted / adopted Hz", "/planner",
                  contents=["/planner/replan_submitted_hz", "/planner/replan_hz"]),
            chart("Replan · latency / solve ms", "/planner/replan_ms", overrides={
                "/planner/replan_ms": lines(visible=[True, True, False])}),
        ), name="Planner",
    )
    trajectory = bp.Horizontal(
        chart("Traj · sample / publish Hz", "/traj", contents=["/traj/sample_hz", "/traj/reference_hz"]),
        chart("Traj · sample / IPC ms", "/traj/timing_ms", overrides={
            "/traj/timing_ms": lines(visible=[False, True, True, False])}),
        chart("Traj callback / period · ms", "/traj/timing_ms", overrides={
            "/traj/timing_ms": lines(visible=[True, False, False, True])}), name="Traj",
    )
    landing = bp.Horizontal(
        chart("FC mode / precision stage", "/fc/debug",
              contents=["/fc/debug/state", "/fc/debug/precision"]),
        chart("Landing target · odom m", "/landing/target", overrides={
            "/landing/target": lines(visible=[True, True, True, False])}),
        chart("Vision quality · px / confidence", "/landing/quality", overrides={
            "/landing/quality": lines(visible=[False, False, True, True])}), name="Landing",
    )
    return bp.Blueprint(
        bp.Vertical(
            cards, bp.Horizontal(scene, cameras, column_shares=[.65, .35]),
            bp.Tabs(landing, tracking, timing, estimator, planner, trajectory,
                    bp.TextLogView(name="Logs", origin="/logs"), active_tab=0),
            row_shares=[.08, .57, .35],
        ),
        bp.TimePanel(state="collapsed", timeline="sim_time", play_state="following"),
        auto_layout=False, auto_views=False, collapse_panels=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "configs/firefly.rbl")
    args = parser.parse_args()
    build().save("firefly", args.output)
    print(args.output)


if __name__ == "__main__":
    main()
