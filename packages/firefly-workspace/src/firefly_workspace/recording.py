"""Read indexed RRD files and locked, ordered Firefly run archives."""

from contextlib import ExitStack, contextmanager
import fcntl
import json
import os
from pathlib import Path


def _manifest_path(directory):
    path = directory / "manifest.json"
    legacy = directory / "run.json"
    return legacy if not path.exists() and legacy.is_file() else path


@contextmanager
def _read_lock(directory):
    path = directory / ".read.lock"
    with path.open("rb") as lock:
        fcntl.flock(lock, fcntl.LOCK_SH)
        if not os.path.samestat(os.fstat(lock.fileno()), path.stat()):
            raise FileNotFoundError(f"Archive number was reused: {directory}")
        yield


@contextmanager
def recording_files(path, *, complete=True):
    """Hold the archive read lock until every consumer has finished with the files."""
    path = Path(path)
    if path.is_file():
        if path.suffix == ".partial":
            raise ValueError(f"Unfinished recording: {path}")
        if path.parent.name == "rrd" and _manifest_path(path.parent.parent).is_file():
            with _read_lock(path.parent.parent):
                yield [path]
        else:
            yield [path]
        return
    with _read_lock(path), ExitStack() as locks:
        writer_lock = path / ".rrd.lock"
        if complete and writer_lock.exists():
            writer = locks.enter_context(writer_lock.open("rb"))
            fcntl.flock(writer, fcntl.LOCK_SH | fcntl.LOCK_NB)
        manifest = json.loads(_manifest_path(path).read_text())
        if manifest["format"] != 1:
            raise ValueError(f"Unknown run archive format: {path}")
        directory = path / "rrd"
        segments = []
        for segment in directory.iterdir():
            if segment.suffix == ".credit" or (not complete and segment.suffix == ".partial"):
                continue
            if not (segment.is_file() and not segment.is_symlink() and segment.suffix == ".rrd"
                    and segment.stem.isascii() and segment.stem.isdecimal()
                    and segment.name == f"{int(segment.stem):06d}.rrd"):
                raise ValueError(f"Unfinished or invalid segment: {segment}")
            segments.append(segment)
        segments.sort(key=lambda file: int(file.stem))
        if not segments:
            raise ValueError(f"No sealed recording in {path}")
        if complete and ("rrd" in manifest["gaps"] or "rrd" in manifest["evicted"]
                         or len(segments) != manifest["rrd_last"]
                         or any(int(s.stem) != i for i, s in enumerate(segments, start=1))):
            raise ValueError(f"Run has missing recording evidence: {path}")
        locks.close()
        yield segments


def recording_chunks(path, *, content=None, complete=True):
    """Stream each recorded chunk once, including static baselines shared by segments."""
    from rerun.chunk import RrdReader

    seen = set()
    with recording_files(path, complete=complete) as files:
        for file in files:
            reader = RrdReader(file)
            [recording] = reader.recordings()
            if complete:
                reader.store(store=recording)
            stream = reader.stream(store=recording)
            if content is not None:
                stream = stream.filter(content=content)
            for chunk in stream:
                if chunk.id not in seen:
                    seen.add(chunk.id)
                    yield chunk
