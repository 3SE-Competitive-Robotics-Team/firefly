"""MuJoCo 无人机环境：物理步进 + 传感器提取（IMU/双目灰度/深度）。

传感器语义（实测验证）：
- 陀螺仪：体坐标系角速度（rad/s）；
- 加速度计：体坐标系比力（m/s²，悬停读 +9.81，自由落体读 0），即真实 IMU 语义。
- 深度：米制 f32，离屏渲染（macOS OpenGL 无 ARB_clip_control，远距精度有限）。

本模块是**被控对象**（plant）：物理步进、传感器、施加旋翼推力。控制律不在这里
（唯一实现在 `firefly-flight`）：飞控进程经 `Firefly/Control` 给出 4 电机推力，
本环境写入模型执行器的 `ctrl`，由 `MuJoCo` 按 site 上的 gear（力 + 反扭矩）合成
刚体 wrench——对照 `mujoco_menagerie/skydio_x2/x2.xml` 的旋翼建法。`apply_pd` 仅供
`--script`（VIO 验证的轨迹跟踪夹具）使用。
"""

from __future__ import annotations

import numpy as np

import mujoco

from .messages import IMAGE_HEIGHT, IMAGE_WIDTH
from .scene import ROTORS, build_scene

#: 脚本跟踪夹具（`--script`）PD：**加速度域**增益（位置 1/s²、速度 1/s，姿态 1/s²、
#: 1/s），与质量/惯量无关。数值是旧力域增益按被测对象（9kg 机体，Ixx≈0.06/Izz≈0.22）
#: 归一的等效值——闭环带宽/阻尼与原口径一致：位置 ω≈1.5 rad/s、ζ≈0.82，
#: 回正 ω≈11.6 rad/s、ζ≈0.77，偏航 ω≈5.2 rad/s、ζ≈1.07。
KP_POS = 2.22
KD_VEL = 2.44
#: 水平回正（角加速度域）
KP_LEVEL = 134.0
KD_ATT = 17.8
#: 偏航跟踪（角加速度域）
KP_YAW = 26.8
KD_YAW = 11.2


class DroneEnv:
    """MuJoCo 无人机环境。

    参数：
        timestep: 物理步长（秒）。
        gyro_noise: 陀螺仪白噪声标准差（rad/s）。
        accel_noise: 加速度计白噪声标准差（m/s²）。
        depth_noise: 深度噪声强度（视差域 σ_disp = 4·depth_noise px，σ_z≈z²·σ_disp/(f·B)
            ∝z²，空区/远平面不加噪；另含 5~15% 随机丢点与 1px 边缘膨胀）。
        scene: 场景名（`warehouse` / `boxes` / `rmuc2026`，见 `scene.build_scene`）。
    """

    def __init__(
        self,
        timestep: float = 0.005,
        gyro_noise: float = 0.002,
        accel_noise: float = 0.02,
        depth_noise: float = 0.02,
        scene: str = "rmuc2026",
    ) -> None:
        self.model = mujoco.MjModel.from_xml_string(build_scene(scene))
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
        self._rotor_positions = np.array(
            [
                self.model.site_pos[
                    mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"rotor{i}")
                ]
                for i in range(len(ROTORS))
            ]
        )
        # 旋翼执行器：actuator 必须挂在同名 site 上（顺序即 rotor0..3 = 飞控电机编号），
        # 上限/旋向/反扭矩系数全部从模型读回（MJCF 是唯一来源，见 scene._rotor_actuators_xml）
        self._rotor_ids: list[int] = []
        for i in range(len(ROTORS)):
            aid = int(
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"rotor{i}")
            )
            if aid < 0:
                raise ValueError(f"模型缺少旋翼执行器 rotor{i}（见 scene._rotor_actuators_xml）")
            site_id = int(self.model.actuator_trnid[aid, 0])
            want = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"rotor{i}"))
            if site_id != want:
                raise ValueError(f"执行器 rotor{i} 未挂在同名 site 上（会与飞控的电机编号错位）")
            self._rotor_ids.append(aid)
        self._max_thrust_per_motor = float(
            self.model.actuator_ctrlrange[self._rotor_ids[0], 1]
        )
        gears = self.model.actuator_gear[self._rotor_ids]
        self._rotor_spins = np.sign(gears[:, 5])
        self._torque_coefficient = float(np.abs(gears[0, 5]))
        mujoco.mj_forward(self.model, self.data)

    @property
    def time(self) -> float:
        """当前仿真时刻（秒）。"""
        return float(self.data.time)

    @property
    def mass(self) -> float:
        """无人机质量（kg）。"""
        return float(self.model.body_mass[self._drone_id])

    def inertia_body(self) -> np.ndarray:
        """转动惯量在**机体系**的对角元（kg·m²）。

        `mjModel.body_inertia` 是主轴系下的角元，主轴相对机体系可能旋转
        （MuJoCo 按惯量大小重排，此处是绕 y 的 90°）——用 `body_iquat`
        转回机体系。非对称布局会留下非对角元，此处直接报错（契约只承载
        对角元，要么改回对称布局，要么扩展契约）。
        """
        principal = np.asarray(self.model.body_inertia[self._drone_id], dtype=float)
        w, x, y, z = (float(v) for v in self.model.body_iquat[self._drone_id])
        rot = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )
        full = rot @ np.diag(principal) @ rot.T
        off_diag = float(np.abs(full - np.diag(np.diag(full))).max())
        if off_diag > 1e-9:
            raise ValueError(
                f"机体惯量在机体系非对角（最大 {off_diag:.3e} kg·m²）："
                "Airframe 消息只承载对角元——改回对称布局，或扩展消息契约"
            )
        return np.diag(full).copy()

    def state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """真值状态 `(pos, vel_world, quat_xyzw, angvel_body)`（飞控的 PlantState）。"""
        d = self.data
        quat_wxyz = d.body("drone").xquat
        return (
            d.body("drone").xpos.copy(),
            d.qvel[0:3].copy(),
            np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]),
            d.qvel[3:6].copy(),
        )

    def airframe(self) -> dict:
        """机体/执行器描述（`Firefly/Airframe` 的唯一内容来源）。

        质量/惯量/旋翼位置读自 `mjModel`，旋向/反扭矩系数/单电机上限读自 actuator 的
        gear 与 ctrlrange——改 MJCF 即改被控对象，无第二处同步；阻尼为 0
        （`MuJoCo` 侧无气动模型）。
        """
        return {
            "mass": self.mass,
            "inertia": self.inertia_body(),
            "linear_drag": 0.0,
            "angular_drag": 0.0,
            "rotor_positions": self._rotor_positions.copy(),
            "rotor_spins": self._rotor_spins.copy(),
            "max_thrust_per_motor": self._max_thrust_per_motor,
            "torque_coefficient": self._torque_coefficient,
        }

    def reset(self, pos: np.ndarray, quat_xyzw: np.ndarray) -> None:
        """重置位姿（`quat_xyzw`：MuJoCo wxyz 顺序，此处接收 xyzw 并转 wxyz）。"""
        self.data.qpos[:] = np.concatenate([pos, quat_xyzw[[3, 0, 1, 2]]])
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ---- 控制 ----

    def apply_motor_thrusts(self, thrusts: np.ndarray) -> None:
        """4 电机推力（N，顺序 = rotor0..3）→ 模型执行器的 `ctrl`。

        旋翼的力/力矩由 `MuJoCo` 按 site 上的 gear 合成（力随机体 z，反扭矩随机体
        z 按旋向），单电机上限由 `ctrlrange` 在引擎侧夹（`autolimits`）——与飞控
        `Airframe` 消息里的上限同源（都读自模型）。
        """
        t = np.asarray(thrusts, dtype=float)
        if t.shape != (len(self._rotor_ids),):
            raise ValueError(f"电机推力形状应为 ({len(self._rotor_ids)},)，收到 {t.shape}")
        self.data.ctrl[self._rotor_ids] = t

    def apply_hover_hold(self) -> None:
        """飞控缺席/指令陈旧时的兜底：等推力抵消重力（含倾斜补偿）。

        这是**失效保护**（不主动动作、不掉高），不是跟踪律——跟踪律唯一家是
        `firefly-flight`。
        """
        up_z = float(np.clip(self.data.body("drone").xmat.reshape(3, 3)[2, 2], 0.3, 1.0))
        total = self.mass * 9.81 / up_z
        self.apply_motor_thrusts(np.full(4, total / 4.0))

    def apply_pd(
        self,
        ref_pos: np.ndarray,
        ref_vel: np.ndarray,
        ref_yaw: float = 0.0,
        ref_yaw_rate: float = 0.0,
        est_pos: np.ndarray | None = None,
        est_vel: np.ndarray | None = None,
    ) -> None:
        """轨迹跟踪夹具（`--script` 专用）：PD + 重力补偿 + 姿态回正/阻尼 + 偏航跟踪。

        不是飞控路径：正式控制由 `apps/fc` 经 `Firefly/Control` 给出。本夹具以真值
        姿态/角速度闭环、直接写 wrench（不经电机分配与限幅），用于 VIO 验证时
        沿指定轨迹飞行。

        偏航：机体 x 轴相对世界 x 轴的转角（`atan2(R[1,0], R[0,0])`），扭矩绕
        世界 z 轴（小倾角下体 z 角速度 ≈ 世界 yaw 率，PD 可用）。

        位置/速度反馈可取外部估计（`est_pos`/`est_vel`，世界系）：飞控只看得到
        估计；`None` 回落真值。姿态项取真值（估计姿态是 JPL 约定，与机体→世界
        旋转的换算未经验证，不得混入控制）。
        """
        d = self.data
        bid = self._drone_id
        if est_pos is None:
            pos = d.body("drone").xpos.copy()
        else:
            pos = np.asarray(est_pos, dtype=float)
        # freejoint 速度：线速度世界系，角速度为机体系（实测：yaw=90° 时加
        # 世界系 x 扭矩，qvel 角速度读数为机体 y 轴——与 R 列向量一致）；
        # 用到世界系时须经 R 转系（`cvel` 对 freejoint 不可靠，漂移）
        if est_vel is None:
            vel = d.qvel[0:3].copy()
        else:
            vel = np.asarray(est_vel, dtype=float)
        angvel_body = d.qvel[3:6].copy()
        inertia = self.inertia_body()

        # 加速度域 PD → 力（与质量无关：改机体不需要重调增益）
        a_des = KP_POS * (np.asarray(ref_pos, dtype=float) - pos) + KD_VEL * (
            np.asarray(ref_vel, dtype=float) - vel
        )
        force = self.mass * (a_des + np.array([0.0, 0.0, 9.81]))

        # 水平回正：body z 轴 → world z 轴（误差向量即小角旋转向量，乘惯量得力矩）
        R = d.body("drone").xmat.reshape(3, 3)
        z_body = R[:, 2]
        level_err = np.cross(z_body, np.array([0.0, 0.0, 1.0]))
        # 姿态阻尼必须在世界系（机体角速度经 R 转系；偏航旋转时直接用机体系
        # 会把阻尼方向转起来反充能量，持续偏航即翻滚发散）；偏航轴交偏航 PD
        w_world = R @ angvel_body
        rate_err = -np.array([w_world[0], w_world[1], 0.0])

        # 偏航跟踪（误差折叠到 [-π, π]；偏航率用世界系 z 分量）
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        yaw_err = (float(ref_yaw) - yaw + np.pi) % (2.0 * np.pi) - np.pi
        rate_err[2] = float(ref_yaw_rate) - w_world[2]
        gains = np.array([KP_LEVEL, KP_LEVEL, KP_YAW])
        rate_gains = np.array([KD_ATT, KD_ATT, KD_YAW])
        torque = inertia * (gains * level_err + rate_gains * rate_err)

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
