//! Solver-owned, bounded persistent workers. One-thread execution stays on the caller.
use super::{Result, graph_error};
use std::sync::Arc;

#[derive(Default)]
pub(super) struct Execution {
    pool: Option<Arc<rayon::ThreadPool>>,
}

impl Execution {
    pub fn install<T: Send>(&self, task: impl FnOnce() -> T + Send) -> T {
        let span = fastrace::Span::enter_with_local_parent("solver-execution");
        let run = move || {
            let _parent = span.set_local_parent();
            task()
        };
        match &self.pool {
            Some(pool) => pool.install(run),
            None => run(),
        }
    }
    pub fn configure(&mut self, threads: usize) -> Result<()> {
        if threads == 0 {
            return Err(graph_error("solver thread count must be positive"));
        }
        if threads == 1 {
            self.pool = None;
        } else if self
            .pool
            .as_ref()
            .is_none_or(|pool| pool.current_num_threads() != threads)
        {
            self.pool = Some(Arc::new(
                rayon::ThreadPoolBuilder::new()
                    .num_threads(threads)
                    .thread_name(|i| format!("okvis-solver-{i}"))
                    .build()
                    .map_err(|e| graph_error(&format!("solver worker creation failed: {e}")))?,
            ));
        }
        Ok(())
    }

    #[cfg(test)]
    pub fn pool(&self) -> Option<Arc<rayon::ThreadPool>> {
        self.pool.clone()
    }
}
