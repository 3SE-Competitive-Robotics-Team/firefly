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
