"""工作区发现与内容指纹的独立输入契约。"""
import pytest

from firefly_workspace import digest, workspace_root


def test_optional_toml_distinguishes_missing_from_empty_and_propagates_errors(tmp_path, caplog):
    import logging
    import tomllib
    from firefly_workspace import load_toml

    path = tmp_path / "optional.toml"
    caplog.set_level(logging.INFO)
    assert load_toml(path) == {}
    assert any(record.levelno == logging.INFO and str(path) in record.getMessage()
               and "使用默认值" in record.getMessage() for record in caplog.records)
    caplog.clear()
    path.write_text("# defaults\n")
    assert load_toml(path) == {}
    assert any("已加载配置" in message and str(path) in message for message in caplog.messages)
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
    for manifest in ('[project]\nname = "unrelated"\n[tool.uv.workspace]\n',
                     '[project]\nname = "firefly"\n'):
        (tmp_path / "pyproject.toml").write_text(manifest)
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
    path.unlink()
    with pytest.raises(FileNotFoundError):
        digest(path)
