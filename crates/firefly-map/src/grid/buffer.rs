//! Paged copy-on-write storage: snapshots share pages; writers detach only touched pages.
use std::{
    ops::{Index, IndexMut},
    sync::Arc,
};

const PAGE_LEN: usize = 4096;

#[derive(Clone, Debug, PartialEq)]
struct Page<T> {
    data: [T; PAGE_LEN],
    // Conservative summary: arbitrary element writes invalidate it.
    uniform: Option<T>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Buffer<T> {
    pages: Arc<Vec<Arc<Page<T>>>>,
    len: usize,
}

impl<T: Copy> Buffer<T> {
    /// Update selected cells page by page, detaching each shared page once.
    #[cfg(feature = "depth")]
    pub(crate) fn fill_masked(&mut self, mask: &[u64], value: T) {
        assert_eq!(mask.len(), self.len.div_ceil(64));
        let pages = Arc::make_mut(&mut self.pages);
        for (page, words) in pages.iter_mut().zip(mask.chunks(PAGE_LEN / 64)) {
            if words.iter().all(|&bits| bits == 0) {
                continue;
            }
            let page = Arc::make_mut(page);
            page.uniform = None;
            let (groups, _) = page.data.as_chunks_mut::<64>();
            for (cells, &bits) in groups.iter_mut().zip(words) {
                if bits == u64::MAX {
                    cells.fill(value);
                } else {
                    let mut selected = bits;
                    while selected != 0 {
                        cells[selected.trailing_zeros() as usize] = value;
                        selected &= selected - 1;
                    }
                }
            }
        }
    }

    pub(crate) fn new(value: T, len: usize) -> Self {
        let page = Arc::new(Page {
            data: [value; PAGE_LEN],
            uniform: Some(value),
        });
        Self {
            pages: Arc::new(vec![page; len.div_ceil(PAGE_LEN)]),
            len,
        }
    }

    #[cfg(feature = "depth")]
    pub(crate) fn shares_data(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.pages, &other.pages)
    }

    pub(crate) const fn len(&self) -> usize {
        self.len
    }

    pub(crate) fn fill(&mut self, value: T) {
        // Resetting a shared buffer needs no copies of discarded contents.
        if Arc::strong_count(&self.pages) > 1 {
            *self = Self::new(value, self.len);
            return;
        }
        let mut replacement = None;
        for page in Arc::make_mut(&mut self.pages) {
            if let Some(data) = Arc::get_mut(page) {
                data.data.fill(value);
                data.uniform = Some(value);
            } else {
                *page = Arc::clone(replacement.get_or_insert_with(|| {
                    Arc::new(Page {
                        data: [value; PAGE_LEN],
                        uniform: Some(value),
                    })
                }));
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn iter(&self) -> impl Iterator<Item = &T> {
        self.pages
            .iter()
            .flat_map(|page| page.data.iter())
            .take(self.len)
    }
}

impl<T: Copy + PartialEq> Buffer<T> {
    #[cfg(feature = "depth")]
    pub(crate) fn fill_range(&mut self, mut start: usize, end: usize, value: T) {
        while start < end {
            let offset = start % PAGE_LEN;
            let len = (end - start).min(PAGE_LEN - offset);
            let page = &self.pages[start / PAGE_LEN];
            if page.uniform != Some(value)
                && page.data[offset..offset + len].iter().any(|&v| v != value)
            {
                let page = Arc::make_mut(&mut Arc::make_mut(&mut self.pages)[start / PAGE_LEN]);
                page.data[offset..offset + len].fill(value);
                page.uniform = (len == PAGE_LEN).then_some(value);
            }
            start += len;
        }
    }

    pub(crate) fn is_known_uniform(&self, value: T) -> bool {
        self.pages.iter().all(|page| page.uniform == Some(value))
    }

    /// Known uniform suffix of the containing page, clipped to the logical buffer.
    pub(crate) fn uniform_len(&self, index: usize, value: T) -> usize {
        if self.pages[index / PAGE_LEN].uniform.as_ref() == Some(&value) {
            (PAGE_LEN - index % PAGE_LEN).min(self.len - index)
        } else {
            0
        }
    }
}

impl<T> Index<usize> for Buffer<T> {
    type Output = T;
    #[inline]
    fn index(&self, index: usize) -> &T {
        assert!(index < self.len);
        &self.pages[index / PAGE_LEN].data[index % PAGE_LEN]
    }
}

impl<T: Copy> IndexMut<usize> for Buffer<T> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut T {
        assert!(index < self.len);
        let page = Arc::make_mut(&mut Arc::make_mut(&mut self.pages)[index / PAGE_LEN]);
        page.uniform = None;
        &mut page.data[index % PAGE_LEN]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "depth")]
    #[test]
    fn masked_updates_preserve_snapshot_and_untouched_pages() {
        let len = 2 * PAGE_LEN + 5;
        let mut buffer = Buffer::new(3, len);
        let old = buffer.clone();
        let mut mask = vec![0; len.div_ceil(64)];
        mask[0] = u64::MAX;
        mask[PAGE_LEN / 64 - 1] = 1 << 63;
        *mask.last_mut().unwrap() = 1 << 4;
        buffer.fill_masked(&mask, 7);
        assert!(Arc::ptr_eq(&buffer.pages[1], &old.pages[1]));
        for i in 0..len {
            assert_eq!(old[i], 3);
            let selected = i < 64 || i == PAGE_LEN - 1 || i == len - 1;
            assert_eq!(buffer[i], if selected { 7 } else { 3 });
        }
    }

    #[test]
    fn moving_an_empty_window_does_not_detach_voxel_pages() {
        let mut map = crate::GridMapBuilder::new(0.1, [160, 160, 80])
            .build()
            .unwrap();
        let snapshot = map.clone();
        map.move_ring_buffer(nalgebra::Vector3::new(1., 4., 1.));
        assert!(Arc::ptr_eq(&map.occupancy.pages, &snapshot.occupancy.pages));
        assert!(Arc::ptr_eq(&map.inflate.pages, &snapshot.inflate.pages));
        assert!(Arc::ptr_eq(
            &map.inflation_pending.pages,
            &snapshot.inflation_pending.pages
        ));
    }

    #[test]
    fn snapshot_shares_storage_and_detaches_only_written_page() {
        let mut live = Buffer::new(0_u16, 3 * PAGE_LEN + 1);
        live[1] = 7;
        let snapshot = live.clone();
        assert!(Arc::ptr_eq(&live.pages, &snapshot.pages));
        live[PAGE_LEN + 2] = 9;
        assert_eq!(snapshot[PAGE_LEN + 2], 0);
        assert_eq!(live[PAGE_LEN + 2], 9);
        for i in [0, 2, 3] {
            assert!(Arc::ptr_eq(&live.pages[i], &snapshot.pages[i]));
        }
        let page = Arc::as_ptr(&live.pages[1]);
        live[PAGE_LEN + 3] = 10;
        assert_eq!(page, Arc::as_ptr(&live.pages[1]));
        live.fill(0);
        assert_eq!(snapshot[1], 7);
        assert!(live.iter().all(|value| *value == 0));
        assert_eq!(live.iter().count(), 3 * PAGE_LEN + 1);
    }
}
