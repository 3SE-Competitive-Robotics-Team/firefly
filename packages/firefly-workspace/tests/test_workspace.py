"""工作区发现与内容指纹的独立输入契约。"""
import pytest

from firefly_workspace import digest, workspace_root


def test_optional_toml_only_defaults_when_missing(tmp_path, caplog):
    import logging
    import tomllib
    from firefly_workspace import load_toml

    path = tmp_path / "optional.toml"
    with caplog.at_level(logging.INFO):
        assert load_toml(path) == {}
    assert len(caplog.records) == 1
    assert caplog.records[0].levelno == logging.INFO
    path.write_text("# defaults\n")
    assert load_toml(path) == {}
    path.write_text("rate = 50")
    assert load_toml(path) == {"rate": 50}
    path.write_text("rate = [")
    with pytest.raises(tomllib.TOMLDecodeError):
        load_toml(path)
    with pytest.raises(IsADirectoryError):
        load_toml(tmp_path)


def test_workspace_discovery_uses_workspace_manifest(tmp_path, monkeypatch):
    (tmp_path / "Cargo.toml").write_text("[workspace]\n")
    (tmp_path / "pyproject.toml").write_text('[project]\nname = "firefly"\n[tool.uv.workspace]\nmembers = []\n')
    nested = tmp_path / "packages/example"
    nested.mkdir(parents=True)
    monkeypatch.chdir(nested)
    assert workspace_root() == tmp_path
    assert workspace_root(nested) == tmp_path
    source = nested / "tool.py"
    source.write_text("")
    assert workspace_root(source) == tmp_path
    (tmp_path / "pyproject.toml").write_text('[project]\nname = "unrelated"\n')
    with pytest.raises(ValueError, match="workspace not found"):
        workspace_root()


@pytest.mark.parametrize(("contents", "expected"), [
    (b"", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"),
    (b"abc", "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"),
])
def test_digest_matches_known_sha256_vectors(tmp_path, contents, expected):
    path = tmp_path / "asset.bin"
    path.write_bytes(contents)
    assert digest(path) == expected
    assert digest(str(path)) == expected
    assert path.read_bytes() == contents
    path.unlink()
    with pytest.raises(FileNotFoundError):
        digest(path)
