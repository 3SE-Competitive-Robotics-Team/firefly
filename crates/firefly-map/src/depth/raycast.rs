//! `raycast.cpp`: integer-cell direction, strict tie comparisons, end cell excluded.
use nalgebra::Vector3;

/// Fully traversed cells need one free update. Only endpoints can receive both
/// occupied and free votes, so only they need exact counts.
pub(super) struct FrameRays {
    endpoint_slots: Vec<u32>,
    endpoints: Vec<Endpoint>,
    /// After traversal, the mapper narrows this mask to the cells receiving free evidence.
    pub(super) visited: Vec<u64>,
    /// Persists across frames. A clear bit means a miss will leave this slot at its free clamp.
    probability_cells: Vec<u64>,
    misses_saturate_at_free: bool,
    pub(super) updates: Vec<usize>,
    pub(super) cell_count: usize,
}

struct Endpoint {
    start: Vector3<f64>,
    address: usize,
    hits: usize,
    misses: usize,
}

impl FrameRays {
    pub(super) fn new(capacity: usize, misses_saturate_at_free: bool) -> Self {
        Self {
            endpoint_slots: vec![0; capacity],
            endpoints: Vec::new(),
            visited: vec![0; capacity.div_ceil(64)],
            probability_cells: vec![
                if misses_saturate_at_free { 0 } else { u64::MAX };
                capacity.div_ceil(64)
            ],
            misses_saturate_at_free,
            updates: Vec::new(),
            cell_count: 0,
        }
    }

    pub(super) fn clear(&mut self) {
        for endpoint in self.endpoints.drain(..) {
            self.endpoint_slots[endpoint.address] = 0;
        }
        self.visited.fill(0);
        self.updates.clear();
        self.cell_count = 0;
    }

    pub(super) fn require_update(&mut self, address: usize) {
        self.probability_cells[address / 64] |= 1 << (address % 64);
    }

    pub(super) fn saturated_free(&mut self, address: usize) {
        if self.misses_saturate_at_free {
            self.probability_cells[address / 64] &= !(1 << (address % 64));
        }
    }

    fn first_visit(&mut self, address: usize) -> bool {
        let bit = 1 << (address % 64);
        let word = &mut self.visited[address / 64];
        if *word & bit != 0 {
            return false;
        }
        *word |= bit;
        self.cell_count += 1;
        if self.probability_cells[address / 64] & bit != 0 {
            self.updates.push(address);
        }
        true
    }

    pub(super) fn endpoint(&mut self, address: usize, start: Vector3<f64>, hit: bool) {
        if hit {
            self.require_update(address);
        }
        let slot = &mut self.endpoint_slots[address];
        if *slot == 0 {
            self.endpoints.push(Endpoint {
                start,
                address,
                hits: 0,
                misses: 0,
            });
            *slot = u32::try_from(self.endpoints.len()).expect("map capacity fits i32");
        }
        let end = &mut self.endpoints[*slot as usize - 1];
        end.hits += usize::from(hit);
        end.misses += usize::from(!hit);
    }

    pub(super) fn trace(&mut self, camera: Vector3<f64>, address: impl Fn(Vector3<i32>) -> usize) {
        let camera_cell = camera.map(|v| v.floor() as i32);
        let camera_address = address(camera_cell);
        for i in 0..self.endpoints.len() {
            let end_address = self.endpoints[i].address;
            if end_address == camera_address {
                // The reverse DDA excludes the camera cell, even for a zero-length ray.
                self.first_visit(camera_address);
                continue;
            }
            self.endpoints[i].misses += 1;
            if !self.first_visit(end_address) {
                continue;
            }
            for cell in RayCaster::new(self.endpoints[i].start, camera_cell).skip(1) {
                let a = address(cell);
                let slot = self.endpoint_slots[a];
                if slot != 0 {
                    self.endpoints[slot as usize - 1].misses += 1;
                }
                if !self.first_visit(a) {
                    break;
                }
            }
        }
    }

    pub(super) fn votes(&self, address: usize) -> (usize, usize) {
        match self.endpoint_slots[address] {
            0 => (0, 1),
            slot => {
                let end = &self.endpoints[slot as usize - 1];
                (end.hits, end.misses)
            }
        }
    }
}

struct RayCaster {
    cell: Vector3<i32>,
    end: Vector3<i32>,
    step: Vector3<i32>,
    t_max: Vector3<f64>,
    t_delta: Vector3<f64>,
}
fn intbound(s: f64, ds: f64) -> f64 {
    if ds < 0. {
        return intbound(-s, -ds);
    }
    let s = (s.fract() + 1.).fract();
    (1. - s) / ds
}
impl RayCaster {
    fn new(start: Vector3<f64>, end: Vector3<i32>) -> Self {
        let cell = start.map(|v| v.floor() as i32);
        let direction = end - cell;
        let step = direction.map(i32::signum);
        Self {
            cell,
            end,
            step,
            t_max: Vector3::from_fn(|i, _| intbound(start[i], f64::from(direction[i]))),
            t_delta: Vector3::from_fn(|i, _| f64::from(step[i]) / f64::from(direction[i])),
        }
    }
}
impl Iterator for RayCaster {
    type Item = Vector3<i32>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.cell == self.end {
            return None;
        }
        let result = self.cell;
        let axis = if self.t_max.x < self.t_max.y {
            if self.t_max.x < self.t_max.z { 0 } else { 2 }
        } else if self.t_max.y < self.t_max.z {
            1
        } else {
            2
        };
        self.cell[axis] += self.step[axis];
        self.t_max[axis] += self.t_delta[axis];
        Some(result)
    }
}
