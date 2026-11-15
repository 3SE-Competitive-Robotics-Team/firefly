# Mimir

Native IPC recorder with bounded pending data and columnar RRD output.

```sh
cargo build --release -p mimir
./target/release/mimir
uv run rerun 'logs/<yyyymmdd_hhmmss>/log.rrd' configs/firefly.rbl
```

Start recording before producers and stop it after they exit. Mimir creates
`logs/<yyyymmdd_hhmmss>/log.rrd` in local time and prints its absolute path.
All callers follow the [recording contract](../../docs/mimir.md).
Configuration overrides live in `configs/mimir.toml`; defaults and budgets are in `src/config.rs`.
Images are lossless grayscale. Mimir writes files; the Rerun viewer opens the resulting RRD.
The shared layout is embedded in new recordings; loading its RBL explicitly also replaces any cached
viewer layout. See the [viewer layout contract](../../docs/observability.md#default-viewer-layout).

Follow [observability](../../docs/observability.md) when adding or changing observations.
Build and lifecycle rules are in [development](../../docs/development.md).
