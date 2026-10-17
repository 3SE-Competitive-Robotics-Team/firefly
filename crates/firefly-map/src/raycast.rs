//! 体素射线遍历（逐行移植 `EGO-Planner-v2 plan_env/src/raycast.cpp`）。

/// 符号函数（官方 `signum(int)`）。
fn signum(x: i32) -> i32 {
    x.signum()
}

/// 非负取模（官方 `mod`：`fmod(fmod(value, modulus) + modulus, modulus)`）。
fn mod_positive(value: f64, modulus: f64) -> f64 {
    (value % modulus + modulus) % modulus
}

/// 最小正 `t` 使 `s + t·ds` 为整数（官方 `intbound`）。
fn intbound(s: f64, ds: f64) -> f64 {
    if ds < 0.0 {
        return intbound(-s, -ds);
    }
    let s = mod_positive(s, 1.0);
    (1.0 - s) / ds
}

/// Amanatides & Woo 三维 DDA（官方 `RayCaster`）。
///
/// `start` 为射线起点（官方调用为命中点），`end` 为终点（相机）；
/// [`Self::step`] 先返回当前体素，再判断是否已到终点体素（到终点即停止，
/// 因此终点体素本身不会被调用方消费）。
pub(crate) struct RayCaster {
    x: i32,
    y: i32,
    z: i32,
    end_x: i32,
    end_y: i32,
    end_z: i32,
    step_x: i32,
    step_y: i32,
    step_z: i32,
    t_max_x: f64,
    t_max_y: f64,
    t_max_z: f64,
    t_delta_x: f64,
    t_delta_y: f64,
    t_delta_z: f64,
}

impl RayCaster {
    /// 官方 `setInput`；三轴整数位移都为零时返回 `None`（无步可走）。
    pub(crate) fn new(start: nalgebra::Vector3<f64>, end: nalgebra::Vector3<f64>) -> Option<Self> {
        let x = start.x.floor() as i32;
        let y = start.y.floor() as i32;
        let z = start.z.floor() as i32;
        let end_x = end.x.floor() as i32;
        let end_y = end.y.floor() as i32;
        let end_z = end.z.floor() as i32;
        let dx = f64::from(end_x - x);
        let dy = f64::from(end_y - y);
        let dz = f64::from(end_z - z);
        let step_x = signum(dx as i32);
        let step_y = signum(dy as i32);
        let step_z = signum(dz as i32);
        if step_x == 0 && step_y == 0 && step_z == 0 {
            return None;
        }
        Some(Self {
            x,
            y,
            z,
            end_x,
            end_y,
            end_z,
            step_x,
            step_y,
            step_z,
            t_max_x: intbound(start.x, dx),
            t_max_y: intbound(start.y, dy),
            t_max_z: intbound(start.z, dz),
            t_delta_x: f64::from(step_x) / dx,
            t_delta_y: f64::from(step_y) / dy,
            t_delta_z: f64::from(step_z) / dz,
        })
    }

    /// 官方 `step`：输出当前体素，再向终点前进一步；到终点时返回 `false`。
    pub(crate) fn step(&mut self, out: &mut [i32; 3]) -> bool {
        *out = [self.x, self.y, self.z];
        if self.x == self.end_x && self.y == self.end_y && self.z == self.end_z {
            return false;
        }
        if self.t_max_x < self.t_max_y {
            if self.t_max_x < self.t_max_z {
                self.x += self.step_x;
                self.t_max_x += self.t_delta_x;
            } else {
                self.z += self.step_z;
                self.t_max_z += self.t_delta_z;
            }
        } else if self.t_max_y < self.t_max_z {
            self.y += self.step_y;
            self.t_max_y += self.t_delta_y;
        } else {
            self.z += self.step_z;
            self.t_max_z += self.t_delta_z;
        }
        true
    }
}
