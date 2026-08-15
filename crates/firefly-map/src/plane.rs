//! 平面障碍模型 `(x − s)ᵀ v = 0`。

use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct Plane {
    s: Vector3<f64>,
    v: Vector3<f64>,
}

impl Plane {
    #[must_use]
    pub fn new(s: Vector3<f64>, v: Vector3<f64>) -> Self {
        Self {
            s,
            v: v.normalize(),
        }
    }

    #[must_use]
    pub fn point(&self) -> Vector3<f64> {
        self.s
    }

    #[must_use]
    pub fn normal(&self) -> Vector3<f64> {
        self.v
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlaneDistance {
    value: f64,
}

impl PlaneDistance {
    #[must_use]
    pub fn new(plane: &Plane, p: Vector3<f64>) -> Self {
        Self {
            value: (p - plane.s).dot(&plane.v),
        }
    }

    #[must_use]
    pub fn value(self) -> f64 {
        self.value
    }

    #[must_use]
    pub fn gradient(&self, plane: &Plane) -> Vector3<f64> {
        plane.v
    }
}

impl Plane {
    #[must_use]
    pub fn distance(&self, p: Vector3<f64>) -> PlaneDistance {
        PlaneDistance::new(self, p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_distance() {
        let plane = Plane::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let d = plane.distance(Vector3::new(2.0, 0.0, 0.0));
        assert!((d.value() - 1.0).abs() < 1e-12);
        assert_eq!(d.gradient(&plane), Vector3::new(1.0, 0.0, 0.0));
        let d = plane.distance(Vector3::new(0.0, 0.0, 0.0));
        assert!((d.value() + 1.0).abs() < 1e-12);
    }

    #[test]
    fn normal_is_normalized() {
        let plane = Plane::new(Vector3::zeros(), Vector3::new(2.0, 0.0, 0.0));
        assert!((plane.normal().norm() - 1.0).abs() < 1e-12);
    }
}
