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
    square: Vec<f32>,
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
                square: vec![0.; SIDE * SIDE],
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
    pub fn restore<'a>(
        &'a mut self,
        side: usize,
        values: &'a [f32],
    ) -> (impl Iterator<Item = f32> + 'a, f32) {
        let full = if side == SIDE {
            values
        } else {
            for (row, &y) in self
                .square
                .as_chunks_mut::<SIDE>()
                .0
                .iter_mut()
                .zip(&self.head_axis)
            {
                let a = &values[y.a * side..][..side];
                let b = &values[y.b * side..][..side];
                for (out, &x) in row.iter_mut().zip(&self.head_axis) {
                    *out = y.sample(x.sample(a[x.a], a[x.b]), x.sample(b[x.a], b[x.b]));
                }
            }
            &self.square
        };
        let center = self.output_rows[IMAGE_HEIGHT / 2];
        let center_depth = center.sample(
            full[center.a * SIDE + IMAGE_WIDTH / 2],
            full[center.b * SIDE + IMAGE_WIDTH / 2],
        );
        let pixels = self.output_rows.iter().flat_map(move |&y| {
            let a = &full[y.a * SIDE..][..IMAGE_WIDTH];
            let b = &full[y.b * SIDE..][..IMAGE_WIDTH];
            a.iter().zip(b).map(move |(&a, &b)| y.sample(a, b))
        });
        (pixels, center_depth)
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
            let (pixels, center) = pixels.restore.restore(side, &plane);
            let restored: Vec<_> = pixels.collect();
            assert_eq!(restored.len(), IMAGE_WIDTH * IMAGE_HEIGHT);
            assert_eq!(
                center,
                restored[IMAGE_WIDTH * (IMAGE_HEIGHT / 2) + IMAGE_WIDTH / 2]
            );
            for (i, &v) in restored.iter().enumerate() {
                let expected = 1.
                    + (i % IMAGE_WIDTH) as f32 / (IMAGE_WIDTH - 1) as f32
                    + (i / IMAGE_WIDTH) as f32 / (IMAGE_HEIGHT - 1) as f32;
                assert!((v - expected).abs() < 1e-5);
            }
        }
    }
}
