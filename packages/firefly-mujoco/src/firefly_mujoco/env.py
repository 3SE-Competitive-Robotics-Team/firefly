"""MuJoCo 无人机环境：物理步进 + 传感器提取（IMU/双目灰度/深度）。

传感器语义（实测验证）：
- 陀螺仪：体坐标系角速度（rad/s）；
- 加速度计：体坐标系比力（m/s²，悬停读 +9.81，自由落体读 0），即真实 IMU 语义。
- 深度：米制 f32，离屏渲染（macOS OpenGL 无 ARB_clip_control，远距精度有限）。

控制：`xfrc_applied` 施加体坐标系/世界系力 + 扭矩（PD 位置跟踪 + 姿态阻尼/
水平回正），世界系与 demo 地图系一致。
"""

from __future__ import annotations

import numpy as np

import mujoco

from .messages import IMAGE_HEIGHT, IMAGE_WIDTH
from .scene import SCENE_XML

#: PD 位置增益
KP_POS = 20.0
#: PD 速度增益
KD_VEL = 10.0
#: 姿态角速度阻尼
KD_ATT = 4.0
#: 水平回正增益（使机体 z 轴对齐世界 z 轴）
KP_LEVEL = 30.0


class DroneEnv:
    """MuJoCo 无人机环境。

    参数：
        timestep: 物理步长（秒）。
        gyro_noise: 陀螺仪白噪声标准差（rad/s）。
        accel_noise: 加速度计白噪声标准差（m/s²）。
    """

    def __init__(
        self,
        timestep: float = 0.005,
        gyro_noise: float = 0.002,
        accel_noise: float = 0.02,
    ) -> None:
        self.model = mujoco.MjModel.from_xml_string(SCENE_XML)
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)
        self._gyro_noise = gyro_noise
        self._accel_noise = accel_noise

        self._drone_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "drone"
        )
        gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro")
        accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "accel")
        self._gyro_adr = int(self.model.sensor_adr[gyro_id])
        self._accel_adr = int(self.model.sensor_adr[accel_id])

        self._renderer = mujoco.Renderer(
            self.model, height=IMAGE_HEIGHT, width=IMAGE_WIDTH
        )
        mujoco.mj_forward(self.model, self.data)

    @property
    def time(self) -> float:
        """当前仿真时刻（秒）。"""
        return float(self.data.time)

    @property
    def mass(self) -> float:
        """无人机质量（kg）。"""
        return float(self.model.body_mass[self._drone_id])

    def reset(self, pos: np.ndarray, quat_xyzw: np.ndarray) -> None:
        """重置位姿（`quat_xyzw`：MuJoCo wxyz 顺序，此处接收 xyzw 并转 wxyz）。"""
        self.data.qpos[:] = np.concatenate([pos, quat_xyzw[[3, 0, 1, 2]]])
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ---- 控制 ----

    def apply_pd(self, ref_pos: np.ndarray, ref_vel: np.ndarray) -> None:
        """PD 位置跟踪 + 重力补偿 + 姿态阻尼/水平回正，写入 `xfrc_applied`。"""
        d = self.data
        bid = self._drone_id
        pos = d.body("drone").xpos.copy()
        # freejoint 全局线/角速度（`cvel` 对 freejoint 不可靠，实测漂移；qvel 正确）
        vel = d.qvel[0:3].copy()
        angvel = d.qvel[3:6].copy()

        force = KP_POS * (np.asarray(ref_pos, dtype=float) - pos) + KD_VEL * (
            np.asarray(ref_vel, dtype=float) - vel
        )
        force[2] += self.mass * 9.81  # 重力补偿

        # 水平回正：body z 轴 → world z 轴
        R = d.body("drone").xmat.reshape(3, 3)
        z_body = R[:, 2]
        level_torque = KP_LEVEL * np.cross(z_body, np.array([0.0, 0.0, 1.0]))
        torque = level_torque - KD_ATT * angvel

        d.xfrc_applied[bid, 0:3] = force
        d.xfrc_applied[bid, 3:6] = torque

    def step(self) -> None:
        """推进一个物理步。"""
        mujoco.mj_step(self.model, self.data)

    # ---- 传感器 ----

    def imu(self) -> tuple[np.ndarray, np.ndarray]:
        """IMU 测量：`(gyro, accel)`，体坐标系，带高斯噪声。"""
        d = self.data
        gyro = d.sensordata[self._gyro_adr : self._gyro_adr + 3].copy()
        accel = d.sensordata[self._accel_adr : self._accel_adr + 3].copy()
        if self._gyro_noise > 0:
            gyro += np.random.normal(0.0, self._gyro_noise, 3)
        if self._accel_noise > 0:
            accel += np.random.normal(0.0, self._accel_noise, 3)
        return gyro, accel

    def render_left(self) -> np.ndarray:
        """左目灰度（H×W uint8）。"""
        self._renderer.update_scene(self.data, camera="cam_left")
        rgb = self._renderer.render()
        return self._to_gray(rgb)

    def render_right(self) -> np.ndarray:
        """右目灰度（H×W uint8）。"""
        self._renderer.update_scene(self.data, camera="cam_right")
        rgb = self._renderer.render()
        return self._to_gray(rgb)

    def render_depth(self) -> np.ndarray:
        """深度（H×W float32，米）。"""
        self._renderer.update_scene(self.data, camera="cam_depth")
        self._renderer.enable_depth_rendering()
        depth = self._renderer.render().copy()
        self._renderer.disable_depth_rendering()
        return depth

    def gt_pose(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """真值：`(pos, quat_xyzw, vel_world)`。"""
        d = self.data
        pos = d.body("drone").xpos.copy()
        quat_wxyz = d.body("drone").xquat.copy()
        vel = d.cvel[self._drone_id, 0:3].copy()
        return pos, quat_wxyz[[1, 2, 3, 0]], vel

    @staticmethod
    def _to_gray(rgb: np.ndarray) -> np.ndarray:
        """RGB → 灰度（BT.601 加权）。"""
        return (
            0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
        ).astype(np.uint8)
