# 硬件设计规格

## SoC

| 项 | 规格 |
|---|---|
| SoC | Allwinner A733，12nm |
| CPU | 8 核：2× Arm Cortex-A76 + 6× Arm Cortex-A55 |
| 主频 | A76 最高 2.0GHz，A55 最高 1.8GHz；板端 cpu6/7 上限 2.002GHz，cpu0–5 上限 1.794GHz |
| 协处理器 | RISC-V E902 @ 200MHz |
| GPU | Imagination BXM-4-64 |
| NPU | 3 TOPS |
| 内存 | LPDDR5，4GB |
| 视频 | 解码最高 8K@24fps（H.265/VP9/AVS2），编码最高 4K@30fps（H.265/H.264） |

## 接口

| 项 | 规格 |
|---|---|
| 存储 | eMMC 8/16/32GB、UFS 32/64/128GB 可选（均空贴）；TF 卡最高 128GB |
| 无线 | Wi-Fi 6 + 蓝牙 5.4（含 BLE） |
| 显示 | HDMI 2.0（4K@60）、MIPI-DSI 4-lane、DP 1.4 |
| 摄像头 | 2× 4-lane MIPI-CSI |
| USB | 1× Type-C USB 3.0 Host；1× Type-C USB 2.0（Host/Device） |
| 扩展 | 40Pin（GPIO/UART/I2C/SPI/PWM）、3Pin DEBUG 串口、风扇接口 |
| 供电 | Type-C 5V/3A |
| 尺寸 | 65 × 30mm，14g |
| 系统 | Ubuntu / Debian / Android 13 |

## 开发板

| 项 | 规格 |
|---|---|
| 型号 | Orange Pi Zero 3W |
| 内存 | 3.8GiB 可用 |
| 存储 | 仅 TF 卡（59.7GB），eMMC/UFS 未贴片 |
| GPU | `/dev/dri` 存在（DRM） |

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
| 飞控固件 | [3SE 的 Betaflight fork](../../apps/third-party/betaflight) 以实板原厂版本 `2026.6.0-alpha`（`e92c10887`）和板级配置 `BETAFPVG473_V2`（`6703db5`）为基线。该批次使用 BMI270、内部时钟和 `align_board_yaw = -135`。[厂商 BMI270 固件说明](https://support.betafpv.com/hc/en-us/articles/54655791332761-Firmware-for-Matrix-1S-Brushless-Flight-Controller-5IN1-II) |
| 主板通信 | 拟用 Matrix 的 UART1 与 Orange Pi 双向传输。TX1 接 Orange Pi 的 RX，RX1 接 TX，两板共地。接线前核对焊盘、逻辑电平和端口占用 |
| 串口协议 | 自研 [`firefly_serial_protocol`](../../apps/third-party/betaflight/docs/firefly_serial_protocol.md)。第一版通过 UART1 以 460800 baud、8N1 传输 IMU 样本和遥控通道指令。Orange Pi 端接入和实板联调待完成 |
| IMU | Matrix 随 BMI270 加速度计更新主动上报三轴加速度、三轴角速度和飞控读取快照的时间。任务标称 800 Hz，每帧 20 字节，实际到达率和抖动待板测 |
| 电机控制 | Orange Pi 下发横滚、俯仰、偏航和油门指令。Betaflight 负责姿态控制、混控和板内 DShot 输出 |
| 控制权 | ELRS 遥控器保留解锁和人工接管权。通道映射、人工接管和串口断线行为须通过拆桨验证 |
| 定型依据 | 实测 1207 桨的电流与静推力、整机悬停余量，以及低电量下四电机同时加速时的 5 V 电压 |

## 相机

| 项 | 规格 |
|---|---|
| 前视双目 | 左右各接一路 MIPI-CSI，同步和并发采集待板端验证 |
| 下视相机 | OV2640，CIF 352×288，用于精降。通过 DVP 转 USB 桥接入 USB 3.0 Host，待验证桥接驱动及 V4L2 YUYV 输出 |
| 下视安装 | 光轴向下，光心位于机体原点后方 3 cm，外参以装机标定为准 |

## Planner 深度

| 项 | 设计规格 |
|---|---|
| 深度来源 | 板端 YOLO26n-Depth 以左目灰度图和 320×320 INT8 输入推理米制深度。深度供 Planner 建图，VIO 使用双目原图与 IMU |
| 建图输入 | 深度帧沿用现有 Planner 接口，按采集时间匹配里程计，按参考相机内外参投影 |
| 单目测距 | 320×240 左目灰度在 A733 上以 320×320 INT8 推理，输出恢复为左目分辨率。前视相机实拍的距离误差待测。[模型文档](https://docs.ultralytics.com/tasks/depth/) |
