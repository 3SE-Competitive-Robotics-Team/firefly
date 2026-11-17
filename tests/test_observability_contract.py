"""Registration must cover every production topic; wire definitions stay central."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def test_every_production_topic_is_documented():
    topics = set()
    for source in (ROOT / "crates/firefly-pubsub/src").glob("*.rs"):
        topics.update(re.findall(r'pub const \w+_TOPIC: &str = "(Firefly/[^"]+)"', source.read_text()))
    documented = set(re.findall(r'`(Firefly/[^\s`]+)`', (ROOT / "docs/observability.md").read_text()))
    assert topics == documented


def test_application_topics_are_not_private_contracts():
    for source in (ROOT / "apps").rglob("*.rs"):
        assert not re.search(r'const \w+_TOPIC: &str = "Firefly/', source.read_text()), source


def test_default_blueprint_only_selects_registered_observations():
    from rerun.chunk import RrdReader

    registered = set(re.findall(r'"([a-z][a-z_/]+)"', (ROOT / "apps/mimir/src/schema.rs").read_text()))
    registered.add("logs")
    reader = RrdReader(ROOT / "configs/firefly.rbl")
    [store] = reader.blueprints()
    assert store.application_id == "firefly"
    found_status = set()
    selected = set()
    for chunk in reader.stream(store=store):
        batch = chunk.to_record_batch()
        for name in ("ViewBlueprint:space_origin", "ViewContents:query"):
            if name not in batch.schema.names:
                continue
            for row in batch.column(name).to_pylist():
                for value in row:
                    if value == "$origin/**":
                        continue
                    path = value.removeprefix("+ ").removeprefix("- ").removesuffix("/**").strip("/")
                    selected.add(path)
                    assert not path or path in registered or any(item.startswith(path + "/") for item in registered), path
                    if path.startswith("mimir/status/"):
                        found_status.add(path)
    assert found_status == {path for path in registered if path.startswith("mimir/status/")}
    assert {
        "planner/loop_hz", "planner/fsm_hz", "planner/fsm_timing_ms",
        "planner/replan_submitted_hz", "planner/replan_hz", "planner/replan_ms",
        "traj/sample_hz", "traj/reference_hz", "traj/timing_ms",
    } <= selected
    assert {"cameras/down", "landing/target", "landing/quality", "fc/debug/precision"} <= selected


def test_down_camera_is_active_and_quality_keeps_session_off_the_axis():
    from rerun.chunk import RrdReader

    reader = RrdReader(ROOT / "configs/firefly.rbl")
    values = {}
    for chunk in reader.stream(store=reader.blueprints()[0]):
        batch = chunk.to_record_batch()
        for name in batch.schema.names:
            if name.startswith(("ContainerBlueprint:", "ViewBlueprint:", "SeriesLines:")):
                for row in batch.column(name).to_pylist():
                    if row is not None:
                        values.setdefault(str(chunk.entity_path), {})[name] = row
    [down] = [path.lstrip("/") for path, fields in values.items()
              if fields.get("ViewBlueprint:space_origin") == ["/cameras/down"]]
    [cameras] = [fields for fields in values.values() if fields.get("ContainerBlueprint:display_name") == ["Cameras"]]
    assert down in cameras["ContainerBlueprint:contents"]
    assert cameras["ContainerBlueprint:active_tab"] == [down]
    assert any("/landing/quality/visualizers/" in path
               and fields.get("SeriesLines:visible_series") == [False, False, True, True]
               for path, fields in values.items())
