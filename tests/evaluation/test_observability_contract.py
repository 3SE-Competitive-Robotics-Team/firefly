"""Registration must cover every production topic; wire definitions stay central."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]


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
                    assert not path or path in registered or any(item.startswith(path + "/") for item in registered), path
                    if path.startswith("mimir/status/"):
                        found_status.add(path)
    assert found_status == {path for path in registered if path.startswith("mimir/status/")}
    assert len(found_status) == 6
