"""RMUC 无人机被控对象：MuJoCo 物理、IMU 与旋翼推力。

IMU 输出机体系角速度（rad/s）与比力（m/s²，静止读 +9.81）。
图像与深度由 Bevy render 提供，控制律由 firefly-flight 提供。
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

import mujoco

from .scene import ROTORS, build_scene


_IMU_NOISE_BATCH_SIZE = 256


class DroneEnv:
    """MuJoCo 无人机环境。

    参数：
        timestep: 物理步长（秒）。
        imu_period: IMU 采样周期（秒）；缺省为物理步长。
        gyro_density: 陀螺仪连续白噪声密度（rad/s/√Hz）。
        accel_density: 加速度计连续白噪声密度（m/s²/√Hz）。
    """

    def __init__(
        self,
        timestep: float = 0.001,
        imu_period: float | None = None,
        gyro_density: float = 2.0e-4,
        accel_density: float = 2.0e-3,
        *,
        scene_factory: Callable[[], str] = build_scene,
        rng: np.random.Generator | None = None,
    ) -> None:
        if imu_period is None:
            imu_period = timestep
        if not np.isfinite([timestep, imu_period, gyro_density, accel_density]).all():
            raise ValueError("物理步长、IMU 周期与噪声密度必须有限")
        if timestep <= 0 or imu_period < timestep or gyro_density < 0 or accel_density < 0:
            raise ValueError("物理步长须为正，IMU 周期不得短于物理步长，噪声密度不得为负")
        self.model = mujoco.MjModel.from_xml_string(scene_factory())
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)
        # 周期平均白噪声的方差为 density²/Δt，与估计器的离散测量模型一致。
        self._gyro_noise = gyro_density / np.sqrt(imu_period)
        self._accel_noise = accel_density / np.sqrt(imu_period)

        self._drone_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "drone"
        )
        gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro")
        accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "accel")
        gyro_adr = int(self.model.sensor_adr[gyro_id])
        accel_adr = int(self.model.sensor_adr[accel_id])
        self._rng = rng if rng is not None else np.random.default_rng()
        noise_scales = [scale for scale in (self._gyro_noise, self._accel_noise) if scale > 0]
        self._noise_scales = np.repeat(noise_scales, 3)
        self._noise_batch = np.empty((_IMU_NOISE_BATCH_SIZE, len(self._noise_scales)))
        self._gyro_noise_samples = (
            tuple(self._noise_batch[i, :3] for i in range(_IMU_NOISE_BATCH_SIZE))
            if self._gyro_noise > 0 else ()
        )
        self._accel_noise_samples = (
            tuple(self._noise_batch[i, -3:] for i in range(_IMU_NOISE_BATCH_SIZE))
            if self._accel_noise > 0 else ()
        )
        self._noise_index = _IMU_NOISE_BATCH_SIZE
        self._gyro = self.data.sensordata[gyro_adr : gyro_adr + 3]
        self._accel = self.data.sensordata[accel_adr : accel_adr + 3]
        self._position_world = self.data.xpos[self._drone_id]
        self._quat_wxyz = self.data.xquat[self._drone_id]
        self._velocity_world = self.data.qvel[:3]
        self._angvel_body = self.data.qvel[3:6]

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

    def state(
        self,
        *,
        out: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """真值 `(pos, vel_world, quat_xyzw, angvel_body)`，仅供评测。

        缺省返回独立快照；out 为调用方自有的 float64 缓冲，形状依次为 3、3、4、3。
        """
        if out is None:
            out = (np.empty(3), np.empty(3), np.empty(4), np.empty(3))
        pos, vel, quat, angvel = out
        np.copyto(pos, self._position_world)
        np.copyto(vel, self._velocity_world)
        quat[0], quat[1], quat[2], quat[3] = (
            self._quat_wxyz[1], self._quat_wxyz[2], self._quat_wxyz[3], self._quat_wxyz[0]
        )
        np.copyto(angvel, self._angvel_body)
        return out

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
        """推进一个物理步，并将位姿、接触与传感器缓存刷新到 data.time。"""
        mujoco.mj_step(self.model, self.data)
        # mj_step 最后才积分 qpos/qvel；发布前必须刷新依赖状态的派生量。
        mujoco.mj_forward(self.model, self.data)

    # ---- 传感器 ----

    def imu(
        self, *, out: tuple[np.ndarray, np.ndarray] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """IMU `(gyro, accel)`，体坐标系，带高斯噪声。

        缺省返回独立快照；out 为调用方自有的两个形状为 (3,) 的 float64 缓冲。
        注入的 rng 按批次提前消耗随机数；零噪声轴不消耗随机数。
        """
        if out is None:
            out = (np.empty(3), np.empty(3))
        gyro, accel = out
        np.copyto(gyro, self._gyro)
        np.copyto(accel, self._accel)
        if self._noise_batch.shape[1] == 0:
            return out
        if self._noise_index == _IMU_NOISE_BATCH_SIZE:
            self._rng.standard_normal(out=self._noise_batch)
            self._noise_batch *= self._noise_scales
            self._noise_index = 0
        if self._gyro_noise > 0:
            gyro += self._gyro_noise_samples[self._noise_index]
        if self._accel_noise > 0:
            accel += self._accel_noise_samples[self._noise_index]
        self._noise_index += 1
        return out

    def gt_pose(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """真值：`(pos, quat_xyzw, vel_world)`。"""
        d = self.data
        pos = d.xpos[self._drone_id].copy()
        quat_wxyz = d.xquat[self._drone_id].copy()
        # freejoint 的 qvel[0:3] 是世界系线速度。
        vel = d.qvel[0:3].copy()
        return pos, quat_wxyz[[1, 2, 3, 0]], vel
