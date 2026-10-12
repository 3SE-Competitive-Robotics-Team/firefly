//! 有界像素缓冲池；任务完成、被替换或丢弃时自动归还缓冲。

use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex};

#[derive(Clone, Debug)]
pub struct BufferPool {
    free: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl BufferPool {
    pub fn new(count: usize, bytes: usize) -> Self {
        Self {
            free: Arc::new(Mutex::new((0..count).map(|_| vec![0; bytes]).collect())),
        }
    }

    pub fn take(&self) -> Option<PooledBuffer> {
        self.free
            .lock()
            .expect("buffer pool poisoned")
            .pop()
            .map(|bytes| PooledBuffer {
                bytes,
                pool: self.clone(),
            })
    }
}

#[derive(Debug)]
pub struct PooledBuffer {
    bytes: Vec<u8>,
    pool: BufferPool,
}

impl Deref for PooledBuffer {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.bytes
    }
}

impl DerefMut for PooledBuffer {
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.bytes
    }
}

impl Drop for PooledBuffer {
    fn drop(&mut self) {
        self.pool
            .free
            .lock()
            .expect("buffer pool poisoned")
            .push(std::mem::take(&mut self.bytes));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exhausted_pool_reuses_storage_after_task_is_dropped() {
        let pool = BufferPool::new(1, 8);
        let mut buffer = pool.take().unwrap();
        let address = buffer.as_ptr();
        buffer.copy_from_slice(&[1; 8]);
        assert!(pool.take().is_none());
        drop(buffer);
        let buffer = pool.take().unwrap();
        assert_eq!(buffer.as_ptr(), address);
        assert_eq!(&*buffer, &[1; 8]);
    }
}
