# FC 本地姿态估计

## 职责与接线

`firefly-imu` 为纯算法 crate；`apps/fc/src/imu.rs` 拥有独立工作线程、IMU 和
VIO 外援订阅端。控制线程通过非阻塞内存快照读取姿态与去偏低通角速度。
原始 `Firefly/Imu` 同时供 VIO 使用，FC 的姿态输出不回流 VIO。

- 角速度反馈：`gyro - estimated_bias`，按测量间隔的一阶 30Hz 低通。
- 姿态反馈：本地六误差状态 ESKF，Hamilton body→odom。
- 位置/速度反馈：`Firefly/Odometry`。地图融合只通过坐标树转换规划参考。
- IMU 100Hz、物理 200Hz；FC 按墙钟 1kHz 调度，样本间保持最近姿态与角速度。
  ESKF 只按新测量的时间戳传播，不在控制 tick 上重复积分或外推。

`configs/fc.toml [imu]` 可覆盖 `firefly_imu::Options`；缺键使用代码默认值。
角速度 rad/s，比力 m/s²，时间秒，噪声配置为连续时间标准差密度。

## 初始化与可观测性

上锁状态允许收集连续 2 秒静止窗口，检查三轴方差、平均角速度、加计模长。
重力方向给初始倾角，平均陀螺给三轴零偏；航向是自由规范。
静止窗口有 4096 样本上限，覆盖不足保持未就绪；仅靠 IMU 的静止检测不能排除
所有匀加速情况，因此要求部署时确实将机体放稳。真值不参与此判定。

首次有效 VIO 外援只确定 odom 航向规范：左乘世界竖直轴旋转，保留静止标定的倾角、
陀螺零偏与机体系右误差协方差；随后与外援做 CI 并重放 IMU。首次外援与后续外援
使用相同创新门，禁止用视觉启动分布覆盖静止标定。
飞控同时要求本地姿态可用、odom 对齐与新鲜 VIO，才可解锁。

VIO 静态初始化的陀螺零偏协方差由同一静止窗口计算，使用与传播器相同的白噪声
密度和随机游走。N 个样本均值的白噪声方差为 `Σ(σg²/Δti)/N²`；均值到窗口末端
零偏的游走方差为 `Σ σbg² Δti (i/N)²`（i=1…N−1）。额外加入样本协方差除以 N
作为模型余量，低样本散布不能压低已标定噪声。首样本周期采用相邻周期。
该模型依赖真正静止、采样白噪声与已标定的密度；不能识别所有缓慢转动或温漂。
姿态、位置、速度和加计偏置先验仍按 OpenVINS 静态初始化约定。

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

本地 IMU 分支和其历史只接收 IMU 传播及已确认的地面静止校正。CI 后验单独作为
控制输出并传播，不回写本地分支、静止零偏或重放历史；每次外援均与其测量时刻
的本地分布比较。这样视觉修正不能成为下一次视觉创新检验的本地基准。

两估计器共享 IMU，禁止作为独立高斯测量做普通 EKF 更新。所有外援采用局部
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
再按相同右端零阶保持积分到当前时刻。同一 IMU 区间内多份外援分别与本地分支
对齐，禁止累计之前的视觉后验。乱序、重复和缓存覆盖外的外援不更新状态。
VIO 会话改变锁存未对齐，需重启 FC；IMU 间隔超过 50ms 锁存不连续，禁止补造测量。

## 运行诊断

- `fc/imu/quaternion_xyzw`：控制实际使用的 Hamilton body→odom 姿态（10Hz）。
- `fc/debug/position_target_odom` / `position_error_odom`：odom 系目标与位置误差（10Hz）。
- `fc/imu/health`：本地可用、odom 已对齐、最后接受外援的测量年龄（秒）。
- `fc/imu/accel_gate`：NIS、是否实际应用加计校正。
- `fc/imu/gyro_bias`：控制输出采用的三轴零偏，rad/s。
- `fc/imu/aid_gate`：最近一次 CI 的接受标记、六维 NIS、本地权重；未计算为 -1。
- `fc/imu/attitude_variance` / `bias_variance`：输出协方差对角，rad² / (rad/s)²。
- `fc/debug/tilt_deg` 与 `fc/debug/tilt_err_deg`：估计倾角与仅供评测的真值倾角误差。

运行记录由 `firefly-viz` 写入 `logs/*.rrd`。

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
`uv run pytest tests/system/test_imu_flight.py -q`。
三例分别验证起飞/悬停/降落、持续悬停门控和关闭 VIO 后的三秒有限姿态支持；失联测试通过不代表
能够保高或安全降落。各例的 RRD、固定阈值、二进制/配置指纹和阶段报告均在 `logs/`。

持续悬停门控单独运行：

```bash
uv run pytest tests/system/test_imu_flight.py -q -k imu_hover
```

初始悬停就绪后固定保持 20 秒，整段逐样本检查真实水平误差 <0.5m、垂直误差 <0.15m、
速度 <0.2m/s；目标为初始位置上方 1m，禁止重新锚定到漂移后的位置。随后降落并独立评分：
检测 FC 上锁后固定等待 1 秒，再录制连续 1 秒的静止窗口，要求速度 <0.1m/s、
水平误差 <0.5m、高度误差 <0.08m。窗口由上锁时刻固定，禁止根据真值等待到满足阈值才结束；
整个降落阶段仍受 20 秒仿真时间上限约束。等待期的触地过程由全过程接触策略评分，
碰撞/重置不能因最终静止而被忽略。等待与静止时长在 `Options` 中固定并随报告保存。
飞控姿态与真值的重力方向夹角须 ≤2°，已接受姿态外援的测量年龄须在 [0,0.5] 秒，
本地姿态及 odom 对齐标志须持续有效。姿态/真值证据最大间隔 0.2 秒，1Hz 健康诊断
最大间隔 1.2 秒；缺失或断流不能当成通过。这些检查不修改在线创新门或失联控制策略。
真值仅从落盘 RRD 读取评分；碰撞、重置、阶段未完成或退出残留均使门控失败。

`uv run pytest tests/evaluation/test_hover_metrics.py` 只使用构造数据验证评分规则，不启动仿真。
上面的 system 用例每次启动真实 sim/render/vio/fc/viz，经 IPC 闭环飞行并从本次 RRD 评分，
不依赖聊天会话、已有录制或手动启动的进程。修改 Rust 源码后须重新构建 release 程序。
显式指定测试文件或其 pytest 节点即执行；缺少运行条件时报错，不以 skip 代替验收。

普通 CI 执行数学契约与评分反例测试；全仓自动收集时跳过真实进程测试，
可设置 `FIREFLY_RUN_IMU_FLIGHT=1` 一并执行。
CI 绿色不能代替上述命令的通过报告。修改姿态融合、初始化、坐标、前端或仿真传感器后，
必须补跑该门控；悬停通过也不能代替 22m 往返精度与回环验收。
