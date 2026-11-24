use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};

pub const SIDE: usize = 320;

#[derive(Clone, Copy)]
struct Rows {
    a: usize,
    b: usize,
    mix: f32,
}

impl Rows {
    fn new(position: f32, size: usize) -> Self {
        let p = position.clamp(0., (size - 1) as f32);
        let a = p as usize;
        Self {
            a,
            b: (a + 1).min(size - 1),
            mix: p - a as f32,
        }
    }

    fn sample(self, a: f32, b: f32) -> f32 {
        a + (b - a) * self.mix
    }
}

pub struct Pixels {
    gray: Vec<u8>,
    input_rows: Vec<Rows>,
    restore: OutputGrid,
}

pub struct OutputGrid {
    expanded: Vec<[f32; IMAGE_WIDTH]>,
    head_axis: Vec<Rows>,
    output_rows: Vec<Rows>,
}

impl Pixels {
    pub fn new() -> Self {
        let aligned = |size: usize, n: usize| {
            (0..n)
                .map(|i| Rows::new((i * (size - 1)) as f32 / (n - 1) as f32, size))
                .collect()
        };
        Self {
            gray: vec![0; SIDE * SIDE],
            input_rows: (0..SIDE)
                .map(|i| {
                    Rows::new(
                        (i as f32 + 0.5) * IMAGE_HEIGHT as f32 / SIDE as f32 - 0.5,
                        IMAGE_HEIGHT,
                    )
                })
                .collect(),
            restore: OutputGrid {
                expanded: vec![[0.; IMAGE_WIDTH]; SIDE / 4],
                head_axis: aligned(SIDE / 4, SIDE),
                output_rows: aligned(SIDE, IMAGE_HEIGHT),
            },
        }
    }

    /// Stretch the entire left image with `OpenCV`'s half-pixel input coordinates.
    pub fn input(&mut self, frame: &[u8]) -> (&[u8], &mut OutputGrid) {
        for (row, &r) in self
            .gray
            .as_chunks_mut::<SIDE>()
            .0
            .iter_mut()
            .zip(&self.input_rows)
        {
            let a = &frame[r.a * IMAGE_WIDTH..][..IMAGE_WIDTH];
            let b = &frame[r.b * IMAGE_WIDTH..][..IMAGE_WIDTH];
            for ((out, &a), &b) in row.iter_mut().zip(a).zip(b) {
                *out = r.sample(f32::from(a), f32::from(b)).round() as u8;
            }
        }
        (&self.gray, &mut self.restore)
    }
}

impl OutputGrid {
    /// Restore the compact NPU head to the ONNX output grid, then to camera pixels.
    pub fn rows<'a>(
        &'a mut self,
        side: usize,
        values: &'a [f32],
    ) -> impl Fn(usize) -> [f32; IMAGE_WIDTH] + 'a {
        if side != SIDE {
            for (row, source) in self.expanded.iter_mut().zip(values.chunks_exact(side)) {
                for (out, &x) in row.iter_mut().zip(&self.head_axis) {
                    *out = x.sample(source[x.a], source[x.b]);
                }
            }
        }
        let full = values.as_chunks::<IMAGE_WIDTH>().0;
        let expanded = &self.expanded;
        let axis = &self.head_axis;
        let output_rows = &self.output_rows;
        // Preserve the two vertical interpolation steps of the exported ONNX grid.
        move |index| {
            let y: Rows = output_rows[index];
            if side == SIDE {
                let (a, b) = (&full[y.a], &full[y.b]);
                std::array::from_fn(|i| y.sample(a[i], b[i]))
            } else {
                let upper = axis[y.a];
                let lower = axis[y.b];
                let upper_rows = (&expanded[upper.a], &expanded[upper.b]);
                let lower_rows = (&expanded[lower.a], &expanded[lower.b]);
                std::array::from_fn(|x| {
                    y.sample(
                        upper.sample(upper_rows.0[x], upper_rows.1[x]),
                        lower.sample(lower_rows.0[x], lower_rows.1[x]),
                    )
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_heads_preserve_metric_plane_and_camera_edges() {
        let mut pixels = Pixels::new();
        for side in [SIDE, SIDE / 4] {
            let plane: Vec<_> = (0..side * side)
                .map(|i| 1. + (i / side + i % side) as f32 / (side - 1) as f32)
                .collect();
            let row = pixels.restore.rows(side, &plane);
            let restored: Vec<_> = (0..IMAGE_HEIGHT).flat_map(&row).collect();
            assert_eq!(restored.len(), IMAGE_WIDTH * IMAGE_HEIGHT);
            for (i, &v) in restored.iter().enumerate() {
                let expected = 1.
                    + (i % IMAGE_WIDTH) as f32 / (IMAGE_WIDTH - 1) as f32
                    + (i / IMAGE_WIDTH) as f32 / (IMAGE_HEIGHT - 1) as f32;
                assert!((v - expected).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn compact_head_matches_two_successive_resizes_bit_for_bit() {
        let side = SIDE / 4;
        let values: Vec<_> = (0..side * side)
            .map(|i| ((i * 193 + i / 31) % 719) as f32 * 0.013)
            .collect();
        let coordinate = |i: usize, from: usize, to: usize| {
            Rows::new((i * (from - 1)) as f32 / (to - 1) as f32, from)
        };
        let square_pixel = |x: usize, y: usize| {
            let x = coordinate(x, side, SIDE);
            let y = coordinate(y, side, SIDE);
            let horizontal = |r| x.sample(values[r * side + x.a], values[r * side + x.b]);
            y.sample(horizontal(y.a), horizontal(y.b))
        };
        let mut pixels = Pixels::new();
        let row = pixels.restore.rows(side, &values);
        for y in 0..IMAGE_HEIGHT {
            let r = coordinate(y, SIDE, IMAGE_HEIGHT);
            for (x, value) in row(y).into_iter().enumerate() {
                let expected = r.sample(square_pixel(x, r.a), square_pixel(x, r.b));
                assert_eq!(value.to_bits(), expected.to_bits());
            }
        }
    }
}
