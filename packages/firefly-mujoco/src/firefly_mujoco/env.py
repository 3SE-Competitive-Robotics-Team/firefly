"""RMUC 无人机被控对象：MuJoCo 物理、IMU 与旋翼推力。

IMU 输出机体系角速度（rad/s）与比力（m/s²，静止读 +9.81）。
图像与深度由 Bevy render 提供，控制律由 firefly-flight 提供。
"""

from __future__ import annotations

import numpy as np

import mujoco

from .scene import ROTORS, build_scene


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
        self.model = mujoco.MjModel.from_xml_string(build_scene())
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
        # IMU 噪声发生器：default_rng 比全局 RandomState 快约 3 倍（同分布）。
        self._rng = np.random.default_rng()

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
        """真值状态 `(pos, vel_world, quat_xyzw, angvel_body)`（仅供评测的 PlantState）。"""
        d = self.data
        quat_wxyz = d.xquat[self._drone_id]
        return (
            d.xpos[self._drone_id].copy(),
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
            gyro += self._rng.normal(0.0, self._gyro_noise, 3)
        if self._accel_noise > 0:
            accel += self._rng.normal(0.0, self._accel_noise, 3)
        return gyro, accel

    def gt_pose(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """真值：`(pos, quat_xyzw, vel_world)`。"""
        d = self.data
        pos = d.xpos[self._drone_id].copy()
        quat_wxyz = d.xquat[self._drone_id].copy()
        # freejoint 的 qvel[0:3] 是世界系线速度。
        vel = d.qvel[0:3].copy()
        return pos, quat_wxyz[[1, 2, 3, 0]], vel
