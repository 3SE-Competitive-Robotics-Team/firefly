# Mimir

Native IPC recorder with bounded pending data and columnar RRD output.

```sh
cargo build --release -p mimir
./target/release/mimir --save logs/run.rrd
uv run rerun logs/run.rrd configs/firefly.rbl
```

Start recording before producers and stop it after they exit. Output files must be new.
Configuration overrides live in `configs/mimir.toml`; defaults and budgets are in `src/config.rs`.
Images are lossless grayscale. Mimir writes files; the Rerun viewer opens the resulting RRD.
The shared layout is embedded in new recordings; loading its RBL explicitly also replaces any cached
viewer layout. See the [viewer layout contract](../../docs/observability.md#default-viewer-layout).

Follow [observability](../../docs/observability.md) when adding or changing observations.
Build and lifecycle rules are in [development](../../docs/development.md).
