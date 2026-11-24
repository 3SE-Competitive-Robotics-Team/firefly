use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use firefly_error::{Error, ErrorKind, Result};
use firefly_map::GridMap;
use firefly_pubsub::node::create_node;
use firefly_pubsub::viz::{VOXELS_MAX, VizPublisher, kind};
use nalgebra::Vector3;

pub(super) const VOXEL_RESYNC_TICKS: usize = 50;
const PERCEIVED_VIEW_RADIUS_M: f64 = 4.;

pub(super) struct Request {
    pub map: GridMap,
    pub center: Vector3<f64>,
    pub timestamp: f64,
    pub force_full: bool,
}

enum Mailbox {
    Open(Box<Option<Request>>),
    Stopped,
    Failed(Error),
}

pub(super) struct DisplayWorker {
    shared: Arc<(Mutex<Mailbox>, Condvar)>,
    thread: Option<JoinHandle<()>>,
}

impl DisplayWorker {
    pub fn start() -> Result<Self> {
        let shared = Arc::new((Mutex::new(Mailbox::Open(Box::default())), Condvar::new()));
        let state = Arc::clone(&shared);
        let (ready, startup) = std::sync::mpsc::sync_channel(1);
        let thread = std::thread::Builder::new()
            .name("voxel-display".into())
            .spawn(move || {
                let result = (|| -> Result<()> {
                    let node = create_node()?;
                    let mut renderer = Renderer::new(VizPublisher::with_voxels(&node)?);
                    let _ = ready.send(Ok(()));
                    loop {
                        let request = {
                            let (mutex, wake) = &*state;
                            let mut shared = mutex
                                .lock()
                                .map_err(|_| internal("display state poisoned"))?;
                            loop {
                                match &mut *shared {
                                    Mailbox::Open(pending) => {
                                        if let Some(request) = pending.take() {
                                            break request;
                                        }
                                    }
                                    Mailbox::Stopped | Mailbox::Failed(_) => return Ok(()),
                                }
                                shared = wake
                                    .wait_timeout(shared, Duration::from_millis(100))
                                    .map_err(|_| internal("display state poisoned"))?
                                    .0;
                                if node.wait(Duration::ZERO).is_err() {
                                    *shared = Mailbox::Stopped;
                                }
                            }
                        };
                        let started = std::time::Instant::now();
                        renderer.render(&request)?;
                        super::log_slow_stage("display", started);
                    }
                })();
                if let Ok(mut shared) = state.0.lock() {
                    *shared = match result {
                        Ok(()) => Mailbox::Stopped,
                        Err(error) => {
                            let _ = ready.send(Err(error.clone()));
                            Mailbox::Failed(error)
                        }
                    };
                }
            })
            .map_err(|e| Error::from(e).with_operation("spawn voxel display"))?;
        let receiver = Self {
            shared,
            thread: Some(thread),
        };
        startup
            .recv()
            .map_err(|e| Error::caused_by(ErrorKind::Unavailable, "start voxel display", e))??;
        Ok(receiver)
    }

    /// One pending, unrendered view; preserve any pending full resynchronization.
    pub fn submit(&self, mut request: Request) -> Result<()> {
        let mut shared = self
            .shared
            .0
            .lock()
            .map_err(|_| internal("display state poisoned"))?;
        let pending = match &mut *shared {
            Mailbox::Failed(error) => return Err(error.clone()),
            Mailbox::Stopped => return Err(internal("display worker stopped")),
            Mailbox::Open(pending) => pending.as_mut(),
        };
        if self.thread.as_ref().is_some_and(JoinHandle::is_finished) {
            return Err(internal("display worker stopped"));
        }
        if let Some(previous) = pending {
            request.force_full |= previous.force_full;
        }
        *pending = Some(request);
        self.shared.1.notify_one();
        Ok(())
    }
}

impl Drop for DisplayWorker {
    fn drop(&mut self) {
        if let Ok(mut shared) = self.shared.0.lock() {
            *shared = Mailbox::Stopped;
        }
        self.shared.1.notify_one();
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            log::error!("voxel display worker panicked");
        }
    }
}

fn internal(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into())
}

pub(super) struct Renderer {
    viz_pub: VizPublisher,
    voxel_state: HashMap<String, HashSet<[i32; 3]>>,
    voxel_geometry: HashMap<String, ([f32; 3], [f32; 3])>,
    voxel_current: HashSet<[i32; 3]>,
    voxel_added: Vec<[i32; 3]>,
    voxel_removed: Vec<[i32; 3]>,
    voxel_indices: Vec<[i32; 3]>,
}

impl Renderer {
    fn new(publisher: VizPublisher) -> Self {
        Self {
            viz_pub: publisher,
            voxel_state: HashMap::new(),
            voxel_geometry: HashMap::new(),
            voxel_current: HashSet::new(),
            voxel_added: Vec::new(),
            voxel_removed: Vec::new(),
            voxel_indices: Vec::new(),
        }
    }

    fn render(&mut self, request: &Request) -> Result<()> {
        let mut indices = std::mem::take(&mut self.voxel_indices);
        let (size, origin) =
            Self::occupied_indices(&request.map, Some(request.center), &mut indices);
        self.publish_voxel_delta(
            "plan/collision",
            &indices,
            size,
            origin,
            request.timestamp,
            request.force_full,
        )?;
        self.voxel_indices = indices;
        Ok(())
    }

    /// 局部窗口内的占据体素索引（`center` 为空则全量栅格），附带网格尺寸与原点。
    ///
    /// 局部窗口以 `center` 为中心，沿各轴取前后 4 米。
    fn occupied_indices(
        map: &GridMap,
        center: Option<Vector3<f64>>,
        indices: &mut Vec<[i32; 3]>,
    ) -> ([f32; 3], [f32; 3]) {
        let origin = map.origin();
        let dims = map.dims();
        let size = [map.resolution() as f32; 3];
        let at = [origin.x as f32, origin.y as f32, origin.z as f32];
        indices.clear();
        let (lo, hi) = match center {
            Some(c) => match Self::window_bounds(map, c) {
                Some(bounds) => bounds,
                None => return (size, at),
            },
            None => ([0, 0, 0], [dims[0] - 1, dims[1] - 1, dims[2] - 1]),
        };
        map.visit_occupied(lo, hi.map(|i| i + 1), |index| {
            indices.push(index.map(|i| i as i32));
        });
        let size =
            Self::coarsen_display_voxels(indices, size, lo.map(|i| i as i32), hi.map(|i| i as i32));
        (size, at)
    }

    /// Coalesce occupied display cells without dropping any occupied volume.
    pub(super) fn coarsen_display_voxels(
        indices: &mut Vec<[i32; 3]>,
        mut size: [f32; 3],
        mut low: [i32; 3],
        mut high: [i32; 3],
    ) -> [f32; 3] {
        if indices.len() <= VOXELS_MAX {
            indices.sort_unstable();
            return size;
        }
        let mut words = Vec::new();
        while indices.len() > VOXELS_MAX {
            low = low.map(|v| v.div_euclid(2));
            high = high.map(|v| v.div_euclid(2));
            let dims: [usize; 3] = std::array::from_fn(|i| (high[i] - low[i] + 1) as usize);
            words.clear();
            words.resize(dims.iter().product::<usize>().div_ceil(64), 0u64);
            for cell in indices.drain(..) {
                let [x, y, z]: [usize; 3] =
                    std::array::from_fn(|i| (cell[i].div_euclid(2) - low[i]) as usize);
                let at = (x * dims[1] + y) * dims[2] + z;
                words[at / 64] |= 1 << (at % 64);
            }
            for (word, &bits) in words.iter().enumerate() {
                let mut bits = bits;
                while bits != 0 {
                    let at = word * 64 + bits.trailing_zeros() as usize;
                    indices.push([
                        low[0] + (at / dims[2] / dims[1]) as i32,
                        low[1] + (at / dims[2] % dims[1]) as i32,
                        low[2] + (at % dims[2]) as i32,
                    ]);
                    bits &= bits - 1;
                }
            }
            size = size.map(|value| value * 2.);
        }
        size
    }

    /// 体素层按**增量**发布：与上次发布的集合做差，只发新增/删除。首次发布、
    /// `force_full` 或差集超单条消息上限时发分块全量快照；消费端原子替换集合。
    fn publish_voxel_delta(
        &mut self,
        entity: &str,
        indices: &[[i32; 3]],
        size: [f32; 3],
        origin: [f32; 3],
        t: f64,
        force_full: bool,
    ) -> Result<()> {
        let previous_geometry = self
            .voxel_geometry
            .insert(entity.to_owned(), (size, origin));
        let force_full = force_full || previous_geometry != Some((size, origin));
        self.voxel_current.clear();
        self.voxel_current.extend(indices.iter().copied());
        let (first, full) = {
            match self.voxel_state.get(entity) {
                None => (true, true),
                Some(prev) => {
                    self.voxel_added.clear();
                    self.voxel_removed.clear();
                    self.voxel_added
                        .extend(self.voxel_current.difference(prev).copied());
                    self.voxel_removed
                        .extend(prev.difference(&self.voxel_current).copied());
                    (
                        false,
                        force_full
                            || self.voxel_added.len() > VOXELS_MAX
                            || self.voxel_removed.len() > VOXELS_MAX,
                    )
                }
            }
        };
        if first || full {
            self.publish_voxels(kind::VOXELS, entity, indices, size, origin, t)?;
        } else {
            if !self.voxel_added.is_empty() {
                self.publish_voxels(kind::VOXELS_ADD, entity, &self.voxel_added, size, origin, t)?;
            }
            if !self.voxel_removed.is_empty() {
                self.publish_voxels(
                    kind::VOXELS_REMOVE,
                    entity,
                    &self.voxel_removed,
                    size,
                    origin,
                    t,
                )?;
            }
        }
        match self.voxel_state.get_mut(entity) {
            Some(prev) => std::mem::swap(prev, &mut self.voxel_current),
            None => {
                self.voxel_state
                    .insert(entity.to_string(), std::mem::take(&mut self.voxel_current));
            }
        }
        Ok(())
    }

    /// 连续发布完整更新的所有分块；空快照通过一个空块表达。
    fn publish_voxels(
        &self,
        kind: u32,
        entity: &str,
        indices: &[[i32; 3]],
        size: [f32; 3],
        origin: [f32; 3],
        t: f64,
    ) -> Result<()> {
        let chunks = indices.len().div_ceil(VOXELS_MAX).max(1);
        for chunk in 0..chunks {
            let begin = chunk * VOXELS_MAX;
            let end = (begin + VOXELS_MAX).min(indices.len());
            self.viz_pub.publish_voxels(kind, t, entity, |msg| {
                msg.voxel_count = (end - begin) as u32;
                msg.voxels[..end - begin].copy_from_slice(&indices[begin..end]);
                msg.voxel_size = size;
                msg.voxel_origin = origin;
                msg.chunk_index = chunk as u32;
                msg.chunk_count = chunks as u32;
            })?;
        }
        Ok(())
    }

    /// `center` 周围显示范围对应的窗口索引（闭区间，已裁剪
    /// 到栅格内）；越界返回 `None`。
    fn window_bounds(map: &GridMap, center: Vector3<f64>) -> Option<([usize; 3], [usize; 3])> {
        let range = Vector3::new(
            PERCEIVED_VIEW_RADIUS_M,
            PERCEIVED_VIEW_RADIUS_M,
            PERCEIVED_VIEW_RADIUS_M,
        );
        let origin = map.origin();
        let res = map.resolution();
        let dims = map.dims();
        let mut lo = [0usize; 3];
        let mut hi = [0usize; 3];
        for axis in 0..3 {
            let min = ((center[axis] - range[axis] - origin[axis]) / res).floor();
            let max = ((center[axis] + range[axis] - origin[axis]) / res).ceil();
            let min = min.max(0.0) as usize;
            let max = (max.max(0.0) as usize).min(dims[axis].saturating_sub(1));
            if min > max || dims[axis] == 0 {
                return None;
            }
            lo[axis] = min;
            hi[axis] = max;
        }
        Some((lo, hi))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_pubsub::viz::VoxelMessage;
    use firefly_pubsub::{
        subscriber::{Subscriber, SubscriberOptions},
        viz::{VOXEL_BUFFER_SIZE, VOXEL_TOPIC},
    };

    fn subscribe(node: &firefly_pubsub::node::IpcNode) -> Subscriber<VoxelMessage> {
        Subscriber::with_options(
            node,
            VOXEL_TOPIC,
            SubscriberOptions {
                buffer_size: VOXEL_BUFFER_SIZE,
                service_max_buffer_size: Some(VOXEL_BUFFER_SIZE),
                max_publishers: Some(1),
            },
        )
        .unwrap()
    }

    #[test]
    #[ignore = "requires exclusive voxel IPC topic"]
    fn periodic_full_refresh_updates_geometry_before_delta_resumes() {
        let node = create_node().unwrap();
        let subscriber = subscribe(&node);
        let mut renderer = Renderer::new(VizPublisher::with_voxels(&node).unwrap());
        for (i, (origin, force, expected)) in [
            ([0.; 3], false, kind::VOXELS),
            ([2.; 3], true, kind::VOXELS),
            ([0.; 3], false, kind::VOXELS),
            ([0.; 3], false, kind::VOXELS_ADD),
        ]
        .into_iter()
        .enumerate()
        {
            let cells: Vec<_> = (0..=i as i32).map(|x| [x, 0, 0]).collect();
            renderer
                .publish_voxel_delta("plan/collision", &cells, [1.; 3], origin, i as f64, force)
                .unwrap();
            let message = subscriber
                .receive()
                .unwrap()
                .expect("complete voxel update");
            assert_eq!(message.kind, expected);
            assert_eq!(message.voxel_origin, origin);
            assert_eq!(message.timestamp, i as f64);
            assert_eq!(message.chunk_count, 1);
            assert!(subscriber.receive().unwrap().is_none());
        }
    }

    #[test]
    #[ignore = "requires exclusive voxel IPC topic"]
    fn worker_publishes_the_requested_view_and_joins() {
        let node = create_node().unwrap();
        let subscriber = subscribe(&node);
        let mut map = firefly_map::GridMapBuilder::new(1., [6; 3])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        map.set_collision_snapshot(firefly_map::CollisionSnapshot::from_runs(
            [0; 3],
            [6; 3],
            [([2; 3], [3; 3])],
        ))
        .unwrap();
        let worker = DisplayWorker::start().unwrap();
        worker
            .submit(Request {
                map,
                center: Vector3::repeat(3.),
                timestamp: 42.,
                force_full: true,
            })
            .unwrap();
        let started = std::time::Instant::now();
        loop {
            if let Some(message) = subscriber.receive().unwrap() {
                assert_eq!(message.timestamp, 42.);
                assert_eq!(message.voxel_count, 1);
                assert_eq!(message.voxels[0], [2; 3]);
                break;
            }
            assert!(started.elapsed() < Duration::from_secs(2));
            std::thread::sleep(Duration::from_millis(2));
        }
        drop(worker);
    }
}
