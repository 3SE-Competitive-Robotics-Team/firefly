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
#: PD 速度增益（ζ=KD/(2√(KP·m))≈0.82，近临界阻尼：原 KD=10 时 ζ≈0.37
#: 欠阻尼，无人机对台阶参考 overshoot ~0.5m 造成来回摆动）
KD_VEL = 22.0
#: 偏航 PD 增益（绕世界 z 轴；转动惯量 Izz≈0.2，KP=6/KD=2.5 近临界阻尼）
KP_YAW = 6.0
#: 偏航角速度阻尼
KD_YAW = 2.5
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
        depth_noise: 深度噪声强度（视差域 σ_disp = 4·depth_noise px，σ_z≈z²·σ_disp/(f·B)
            ∝z²，空区/远平面不加噪；另含 5~15% 随机丢点与 1px 边缘膨胀）。
    """

    def __init__(
        self,
        timestep: float = 0.005,
        gyro_noise: float = 0.002,
        accel_noise: float = 0.02,
        depth_noise: float = 0.02,
    ) -> None:
        self.model = mujoco.MjModel.from_xml_string(SCENE_XML)
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)
        self._gyro_noise = gyro_noise
        self._accel_noise = accel_noise
        self._depth_noise = depth_noise

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

    def apply_pd(
        self,
        ref_pos: np.ndarray,
        ref_vel: np.ndarray,
        ref_yaw: float = 0.0,
        ref_yaw_rate: float = 0.0,
    ) -> None:
        """PD 位置跟踪 + 重力补偿 + 姿态阻尼/水平回正 + 偏航跟踪，写入 `xfrc_applied`。

        偏航：机体 x 轴相对世界 x 轴的转角（`atan2(R[1,0], R[0,0])`），扭矩绕
        世界 z 轴（小倾角下体 z 角速度 ≈ 世界 yaw 率，PD 可用）。
        """
        d = self.data
        bid = self._drone_id
        pos = d.body("drone").xpos.copy()
        # freejoint 速度：线速度世界系，角速度为机体系（实测：yaw=90° 时加
        # 世界系 x 扭矩，qvel 角速度读数为机体 y 轴——与 R 列向量一致）；
        # 用到世界系时须经 R 转系（`cvel` 对 freejoint 不可靠，漂移）
        vel = d.qvel[0:3].copy()
        angvel_body = d.qvel[3:6].copy()

        force = KP_POS * (np.asarray(ref_pos, dtype=float) - pos) + KD_VEL * (
            np.asarray(ref_vel, dtype=float) - vel
        )
        force[2] += self.mass * 9.81  # 重力补偿

        # 水平回正：body z 轴 → world z 轴
        R = d.body("drone").xmat.reshape(3, 3)
        z_body = R[:, 2]
        level_torque = KP_LEVEL * np.cross(z_body, np.array([0.0, 0.0, 1.0]))
        # 姿态阻尼必须在世界系（机体角速度经 R 转系；偏航旋转时直接用机体系
        # 会把阻尼方向转起来反充能量，持续偏航即翻滚发散）；偏航轴交偏航 PD
        w_world = R @ angvel_body
        torque = level_torque - KD_ATT * np.array([w_world[0], w_world[1], 0.0])

        # 偏航跟踪（误差折叠到 [-π, π]；偏航率用世界系 z 分量）
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        yaw_err = (float(ref_yaw) - yaw + np.pi) % (2.0 * np.pi) - np.pi
        torque[2] += KP_YAW * yaw_err + KD_YAW * (float(ref_yaw_rate) - w_world[2])

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
        """深度（H×W float32，米）。

        离屏渲染（macOS OpenGL 无 ARB_clip_control，远距精度有限）；噪声模型：

        1. 视差域高斯（σ_z∝z²）：disp=f·B/z，σ_disp=4·depth_noise px，
           σ_z≈z²·σ_disp/(f·B)，比线性模型远距更狠；
        2. 边缘膨胀 1px：深度不连续处前景向背景扩 1 像素（模拟飞点/增胖）；
        3. 随机丢点 5~15%：有效像素置 0（planner 视为无效，z≤0.05）。
        仅对有效命中（0.05<z<100m）处理，空区/远平面保持原值。
        """
        self._renderer.update_scene(self.data, camera="cam_depth")
        self._renderer.enable_depth_rendering()
        depth = self._renderer.render().copy()
        self._renderer.disable_depth_rendering()
        if self._depth_noise <= 0:
            return depth
        valid = (depth > 0.05) & (depth < 100.0) & np.isfinite(depth)
        if not np.any(valid):
            return depth
        # 1. 视差域高斯：f≈168.6 (fovy 70.88°, H=240), B=0.05m, f·B≈8.43
        focal = 120.0 / np.tan(np.deg2rad(70.88 / 2.0))
        baseline = 0.05
        fb = focal * baseline
        disp = np.zeros_like(depth, dtype=np.float64)
        disp[valid] = fb / depth[valid].astype(np.float64)
        sigma_disp = float(self._depth_noise) * 4.0
        disp_noise = np.random.normal(0.0, sigma_disp, size=depth.shape)
        disp_noisy = disp + disp_noise
        disp_noisy = np.maximum(disp_noisy, 0.1)
        depth_noisy = depth.astype(np.float64)
        depth_noisy[valid] = fb / disp_noisy[valid]
        depth = depth_noisy.astype(depth.dtype, copy=False)
        # 2. 边缘膨胀：深度不连续 > max(0.12, 0.04·z) 判为边缘，前景向外扩 1px
        # 有效性掩码参与判断，避免无效区干扰
        d = depth.astype(np.float64)
        v = valid
        # 阈值：近距 12cm，远距 4%·z
        thresh = np.maximum(0.12, 0.04 * np.maximum(d, 0.0))
        # 四邻差分
        pad_d = np.pad(d, 1, mode="edge")
        pad_v = np.pad(v, 1, mode="constant", constant_values=False)
        # 中心切片
        c = pad_d[1:-1, 1:-1]
        c_v = pad_v[1:-1, 1:-1]
        # 邻域
        up = pad_d[0:-2, 1:-1]
        up_v = pad_v[0:-2, 1:-1]
        down = pad_d[2:, 1:-1]
        down_v = pad_v[2:, 1:-1]
        left = pad_d[1:-1, 0:-2]
        left_v = pad_v[1:-1, 0:-2]
        right = pad_d[1:-1, 2:]
        right_v = pad_v[1:-1, 2:]
        edge = np.zeros_like(v, dtype=bool)
        for nb, nb_v in [(up, up_v), (down, down_v), (left, left_v), (right, right_v)]:
            edge |= c_v & nb_v & (np.abs(c - nb) > thresh)
        if np.any(edge):
            # 前景深度在边缘处取较小值（近处）
            edge_depth = np.where(edge, d, np.inf)
            pad_e = np.pad(edge_depth, 1, constant_values=np.inf)
            # 3×3 最小值（前景向外扩）
            min3 = np.full_like(edge_depth, np.inf)
            h, w = edge_depth.shape
            for di in range(3):
                for dj in range(3):
                    cand = pad_e[di : di + h, dj : dj + w]
                    np.minimum(min3, cand, out=min3)
            dilated = np.isfinite(min3)
            # 仅在非边缘、有效且背景更远的像素上膨胀
            fatten = (~edge) & dilated & v & (d > min3 + 1e-9)
            depth[fatten] = min3[fatten].astype(depth.dtype, copy=False)
            # 更新有效掩码（膨胀后仍有效）
            valid = (depth > 0.05) & (depth < 100.0) & np.isfinite(depth)
        # 3. 随机丢点 5~15%（仅有效像素）
        hole_rate = float(np.random.uniform(0.05, 0.15))
        hole = (np.random.random(depth.shape) < hole_rate) & valid
        depth[hole] = 0.0
        return depth

    def gt_pose(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """真值：`(pos, quat_xyzw, vel_world)`。"""
        d = self.data
        pos = d.body("drone").xpos.copy()
        quat_wxyz = d.body("drone").xquat.copy()
        # freejoint 的线速度：`cvel` 对 freejoint 不可靠（实测恒 0，误导 GT
        # 初始化速度先验，vio 以 v=0 起飞 → 初始速度误差 ~0.7 m/s 种下漂移）；
        # qvel[0:3] 即 freejoint 世界系线速度（与 apply_pd 同源，正确）。
        vel = d.qvel[0:3].copy()
        return pos, quat_wxyz[[1, 2, 3, 0]], vel

    @staticmethod
    def _to_gray(rgb: np.ndarray) -> np.ndarray:
        """RGB → 灰度（BT.601 加权）。"""
        return (
            0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
        ).astype(np.uint8)
