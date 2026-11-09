use firefly_error::{Error, ErrorKind, Result};
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    pub queue_messages: usize,
    pub queue_bytes: usize,
    pub batch_rows: usize,
    /// Total pending typed-row memory, independent of the IPC ingress queue.
    pub batch_bytes: usize,
    pub flush_ms: u64,
    pub max_entities: usize,
    pub max_publishers: usize,
    pub max_voxels: usize,
    pub images: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            queue_messages: 1024,
            queue_bytes: 8 * 1024 * 1024,
            batch_rows: 64,
            batch_bytes: 2 * 1024 * 1024,
            flush_ms: 200,
            max_entities: 256,
            max_publishers: 64,
            max_voxels: 100_000,
            images: true,
        }
    }
}

impl Options {
    /// Check pending-data and flushing budgets.
    /// # Errors
    /// A budget is zero or outside its supported bounds.
    pub fn validate(&self) -> Result<()> {
        if self.queue_messages == 0
            || self.queue_bytes < 256 * 1024
            || !(1..=4096).contains(&self.batch_rows)
            || self.batch_bytes == 0
            || !(1..=1000).contains(&self.flush_ms)
            || self.max_entities == 0
            || self.max_publishers == 0
            || self.max_voxels == 0
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid recorder memory or flush budget",
            ));
        }
        Ok(())
    }
}
