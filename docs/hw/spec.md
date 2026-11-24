# 硬件设计规格

## 开发板

实板为 Orange Pi Zero 3W。台架上，Mac 通过 USB 2.0 Device 口管理 Orange Pi，Matrix 接 Orange Pi 的 USB 3.0 Host 口。[部署和台架检查](../development.md#验证-matrix-imu)

| 项目 | 本机规格 |
|---|---|
| SoC | Allwinner A733，6 核 Cortex-A55（cpu0–5，上限 1.794 GHz）和 2 核 Cortex-A76（cpu6–7，上限 2.002 GHz） |
| 内存与存储 | 4 GB LPDDR5，可用 3.8 GiB。系统装在 59.7 GB TF 卡，eMMC 和 UFS 均未贴片 |
| 图形和推理 | Imagination BXM-4-64，板上可见 `/dev/dri`。NPU 标称 3 TOPS |
| 摄像头与 USB | 两路 4-lane MIPI-CSI。一个 Type-C USB 3.0 Host，一个 Type-C USB 2.0 Host/Device |
| 供电与尺寸 | Type-C 5 V / 3 A，65 × 30 mm，14 g |

## 动力

| 项 | 规格 |
|---|---|
| 电机 | 4× BETAFPV 0802（2026 款 Racing，25000 KV），额定 4.2 V（1S），JST 1.25 mm 三针插头。[厂商规格](https://betafpv.com/products/0802-brushless-motors-2026) |
| 桨叶 | 1207 |

## 飞控电调板

| 项 | 规格 |
|---|---|
| 电池 | 1S 18650 或 21700，满充 4.2 V |
| 飞控电调板 | BETAFPV Matrix 1S 5IN1 II 免焊版（Solder-free，`01040020_1`），电池直供，四个 JST 1.25 mm 三针电机座。[厂商规格](https://betafpv.com/products/matrix-1s-5in1-ii-brushless-flight-controller) |
| 机载电源 | 独立升压支路与电调共地，经 Type-C 输出 5 V / 3 A。输入 3.0 V、效率 90% 时约取 5.6 A，电池预算另计电机 |
| 飞控固件 | [3SE 的 Betaflight fork](https://github.com/3SE-Competitive-Robotics-Team/betaflight/tree/6942d620870821b382aeeb036444f28a90b6eba7) 以实板原厂版本 `2026.6.0-alpha`（`e92c10887`）和板级配置 `BETAFPVG473_V2`（`6703db5`）为基线。该批次使用 BMI270、内部时钟和 `align_board_yaw = -135`。[厂商 BMI270 固件说明](https://support.betafpv.com/hc/en-us/articles/54655791332761-Firmware-for-Matrix-1S-Brushless-Flight-Controller-5IN1-II) |
| 主板通信 | Orange Pi 以 USB Host 连接 Matrix 的 USB 数据口。飞控默认提供 Betaflight 配置，处于上锁状态时可通过专用 MSPv2 请求切换当前 USB 会话 |
| 串口协议 | Orange Pi 收到 ACK 后使用 [`firefly_serial_protocol`](https://github.com/3SE-Competitive-Robotics-Team/betaflight/blob/6942d620870821b382aeeb036444f28a90b6eba7/docs/firefly_serial_protocol.md) 接收 IMU 和 10 Hz 飞控诊断快照，发送控制帧或心跳。断开、重启或上行静默 1 秒后恢复配置模式。板端 pilot 独占 USB，RootSLAM 订阅 `Firefly/Imu` |
| IMU | Matrix 随 BMI270 加速度计更新主动上报三轴加速度、三轴角速度和飞控读取快照的时间。任务标称 800 Hz，每帧 20 字节，本批次在 125 秒 USB 台架测试中由 RootSLAM 接收 99,810 个样本，约 798 Hz |
| 电机控制 | Orange Pi 下发横滚、俯仰、偏航和油门指令。输入为四个 1000–2000 µs 杆量。ANGLE 下横滚和俯仰控制倾角，偏航控制转动快慢，油门控制推力。Betaflight 负责自稳、混控和 DShot 输出 |
| 控制权 | ELRS 遥控器负责解锁。将 FIREFLY CONTROL 的开关区间同时分配给 ANGLE，或将其限制在已有 ANGLE 区间内。USB 下行只写四个主通道。通道映射、人工接管和串口断线行为须通过拆桨验证 |
| 定型依据 | 实测 1207 桨的电流与静推力、整机悬停余量，以及低电量下四电机同时加速时的 5 V 电压 |

## 相机

| 项 | 规格 |
|---|---|
| 前视双目 | 左右各接一路 MIPI-CSI，同步和并发采集待板端验证 |
| 下视相机 | OV2640，CIF 352×288，用于精降。通过 DVP 转 USB 桥接入 USB 3.0 Host，需验证与 Matrix 共用 Host 口时的连接方式、桥接驱动及 V4L2 YUYV 输出 |
| 下视安装 | 光轴向下，光心位于机体原点后方 3 cm，外参以装机标定为准 |

## Planner 深度

| 项 | 设计规格 |
|---|---|
| 深度来源 | 板端 YOLO26n-Depth 以左目灰度图和 320×320 INT8 输入推理米制深度。深度供 Planner 建图，VIO 使用双目原图与 IMU |
| 建图输入 | 深度帧沿用现有 Planner 接口，按采集时间匹配里程计，按参考相机内外参投影 |
| 单目测距 | 320×240 左目灰度在 A733 上以 320×320 INT8 推理，输出恢复为左目分辨率。前视相机实拍的距离误差待测。[模型文档](https://docs.ultralytics.com/tasks/depth/) |
