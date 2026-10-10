# FC 本地姿态估计

## 职责与接线

`firefly-imu` 为纯算法 crate；`apps/fc/src/imu.rs` 拥有独立工作线程、IMU 和
VIO 外援订阅端。控制线程通过非阻塞内存快照读取姿态与去偏低通角速度。
原始 `Firefly/Imu` 同时供 VIO 使用，FC 的姿态输出不回流 VIO。

- 角速度反馈：`gyro - estimated_bias`，按测量间隔的一阶 30Hz 低通。
- 姿态反馈：本地六误差状态 ESKF，Hamilton body→odom。
- 位置/速度反馈：`Firefly/Odometry`。地图融合只通过坐标树转换规划参考。
- 原始 IMU 仍为 100Hz、物理 200Hz、控制 1kHz；独立线程不合成额外 IMU。
  算法测试包含 100Hz / 1kHz 采样，但不构成 1kHz 仿真系统性能验收。

`configs/fc.toml [imu]` 可覆盖 `firefly_imu::Options`；缺键使用代码默认值。
角速度 rad/s，比力 m/s²，时间秒，噪声配置为连续时间标准差密度。

## 初始化与可观测性

上锁状态允许收集连续 2 秒静止窗口，检查三轴方差、平均角速度、加计模长。
重力方向给初始倾角，平均陀螺给三轴零偏；航向是自由规范。
静止窗口有 4096 样本上限，覆盖不足保持未就绪；仅靠 IMU 的静止检测不能排除
所有匀加速情况，因此要求部署时确实将机体放稳。真值不参与此判定。

首次有效 VIO 外援确定 odom 规范，采用其姿态/零偏联合边缘分布，然后重放 IMU。
飞控同时要求本地姿态可用、odom 对齐与新鲜 VIO，才可解锁。

加计创新检验每个 IMU 样本都可计算，但只有上锁且完整静止窗口通过时才允许校正。
飞行中不根据 NIS 通过就强制使用重力假设。无磁力计/外部观测时，绝对航向不可观测；
持续平移加速也不能由六轴 IMU 与倾斜完全区分。VIO 失联时继续陀螺传播，零偏随机游走
使不确定性增长；只提供短时姿态支持，不承诺定点、保高或长期姿态精度。

## 方程与卡方检验

名义状态 `(R,bg)`；右误差 `R_true = R Exp(δθ)`，排列 `[δθ,δbg]`。
令 `ω = gyro - bg`，则：

\[
R^+=R\operatorname{Exp}(\omega\Delta t),\qquad
\Phi=\begin{bmatrix}
\operatorname{Exp}(-\omega\Delta t)&-\Delta t J_r(\omega\Delta t)\\
0&I
\end{bmatrix}.
\]

过程噪声使用五点 Gauss–Legendre 对 `∫Φ(s)QcΦ(s)ᵀ ds` 积分，保留姿态/零偏
交叉项；零角速度有闭式测试。加计使用未归一化的三轴原始比力：

\[
h=R^T(0,0,g)^T,\quad H=([h]_\times,0),\quad
r=a_m-h,\quad S=HPH^T+R_a,\quad \mathrm{NIS}=r^TS^{-1}r.
\]

`R_a = (σ_accel²/Δt + σ_model²) I`。静止高斯线性化假设下使用 χ²(3) 的
99% 分位数 **11.344866730144373**；三维残差包含模长信息，不套用归一化方向
的两自由度检验。模型余量为工程参数，门限概率不是实际飞行误接受率保证。
范数与静止条件仍须同时满足。大协方差可能让错误加速度通过 NIS，测试显式覆盖该反例。
拒绝只保持预测，不因连续拒绝超过某次数而强制放行。

接受时均值使用完整 `Kr`，协方差用同一 K 的 Joseph 形式。零创新仍更新协方差。
注入后用 `G=diag(Jr(δθ_hat),I)` 重置右误差协方差；一阶姿态块为
`I - 0.5[δθ_hat]×`，含全部交叉协方差换基。

## VIO 外援与相关性

`Firefly/VioAttitudeAid` 包含：状态的 IMU 时间、非零会话号、Hamilton xyzw、
三轴陀螺零偏以及按行展开的 6×6 右误差协方差。
VIO 从完整滤波协方差提取姿态/零偏联合边缘块（含交叉项），不使用单位阵或
把相机状态协方差贴到预测里程计时刻。在线时延标定不能重贴已积分状态的时间戳。
OpenVINS JPL 的左误差在转换为 Hamilton body→odom 后等于同符号右误差，
该转换有独立增量测试。

两估计器共享 IMU，禁止作为独立高斯测量做普通 EKF 更新。后续外援采用局部
切空间的协方差交集（CI）：

\[
P_c^{-1}=wP_l^{-1}+(1-w)P_v^{-1},\qquad
\delta_c=P_c(1-w)P_v^{-1}\delta_v.
\]

VIO 误差协方差先用 `Jr(log(Rl⁻¹Rv))⁻¹` 转到当前切空间，交叉块同步变换。
在 `[0,1]` 的 21 个权重中最小化 `log det(Pc)`，包含两个端点；相同相关估计
不会把协方差减半。此保守性依赖输入边缘协方差可信与局部线性化有效，不是消除
VIO 模型错误的保证。旋转差超过 0.5rad 拒绝；六维差异另用任意未知相关下的
协方差上界 `2(Pl+Pv)` 与 χ²(6) 99.9% 门控。

重放缓存上限 2048 个 IMU 样本。支持落在两个样本之间的外援时间，在该时刻融合，
再按相同右端零阶保持积分到当前时刻。乱序、重复和缓存覆盖外的外援不更新状态。
VIO 会话改变锁存未对齐，需重启 FC；IMU 间隔超过 50ms 锁存不连续，禁止补造测量。

## 可观测性与证据

- `fc/imu/health`：本地可用、odom 已对齐、最后接受外援的测量年龄（秒）。
- `fc/imu/accel_gate`：NIS、是否实际应用加计校正。
- `fc/imu/gyro_bias`：三轴 rad/s。
- `fc/debug/tilt_deg` 与 `fc/debug/tilt_err_deg`：估计倾角与仅供评测的真值倾角误差。

所有运行记录由 `firefly-viz` 写入 `logs/*.rrd`。原始 IMU 不经本地 ESKF 改写。

## 参考与测试

- Robo-Rust `eskf_imu.rs`：`cdd5611c327ee1637464663392bc928f178f6d20`；名义/误差状态结构参考。
  数学审查：[issue #4](https://github.com/XiaoPengYouCode/Robo-Rust/issues/4)。
- WangHongxi2001/RoboMaster-C-Board-INS-Example：
  `4361e22fc5c7fa26d2a136548c0f8c4e6f8ff1d2`，
  `Components/Algorithm/QuaternionEKF.c::IMU_QuaternionEKF_xhatUpdate`：创新统计量参考。
  本项目使用六维乘法误差状态、三轴零偏和上述统计门限，不移植经验强制放行逻辑。
- [Solà, Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)，第 5–6 节。
- [Julier/Uhlmann, Using covariance intersection for SLAM](https://www.sciencedirect.com/science/article/abs/pii/S0921889006001436)。

`cargo test -p firefly-imu -p fc -p vio` 包含解析解、中心差分、输出契约：
零时间不变性、常角速度、多采样率、F/H/reset 雅可比、零残差更新与 yaw 零空间、
倾角纠正方向、运动加速拒绝、相同相关估计不重复计数、延迟重放、会话/间断拒绝。
雅可比差分步长 1e-6，误差容差 1e-8 或更小；闭式协方差比较为 1e-14 的绝对数值容差。

实际进程验收：先构建 release `vio/fc/render/ffctl`，停止其他闭环进程，再运行
`FIREFLY_RUN_IMU_FLIGHT=1 uv run --all-packages --extra test pytest tests/system/test_imu_flight.py -q`。
两例分别验证起飞/悬停/降落和关闭 VIO 后的三秒有限姿态支持；后者通过不代表
能够保高或安全降落。各例的 RRD、固定阈值、二进制/配置指纹和阶段报告均在 `logs/`。
