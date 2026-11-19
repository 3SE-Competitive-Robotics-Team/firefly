# 硬件规格

机载计算平台是 Orange Pi Zero 3W（Allwinner A733）。本文只记录硬件事实和与项目直接相关的约束；
数据来源为厂商《Orange Pi Zero 3W 用户手册 v1.0》与板端实测，冲突时以实测为准。

## SoC 与算力

| 项 | 规格 |
|---|---|
| SoC | Allwinner A733，12nm |
| CPU | 8 核：2× Arm Cortex-A76 + 6× Arm Cortex-A55 |
| 主频 | A76 最高 2.0GHz，A55 最高 1.8GHz |
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

## 板端实测

| 项 | 实测 |
|---|---|
| CPU | 8 核；cpu6/7 上限 2.002GHz，cpu0–5 上限 1.794GHz，与 2+6 大小核一致 |
| 内存 | 3.8GiB 可用 |
| 存储 | 仅 TF 卡（59.7GB），eMMC/UFS 未贴片 |
| GPU | `/dev/dri` 存在（DRM） |

## 对项目的约束

### 下视相机选型

下视相机拟选 OV2640 模组，采用 CIF 352×288 输出，光心在机体原点后方 3 cm、光轴向下。
相机消息与默认检测网络使用相同宽高；图像进入模型时转换为三通道 NCHW `[1,3,288,352]`。
镜头内参、畸变和完整安装外参以装机标定为准。

OV2640 使用 DVP 并口，接入需要适配的采集桥/驱动；现有采集进程使用 V4L2，设备节点在
模组及转接硬件接入后通过枚举确定。双目的 MIPI-CSI 接口和设备配置独立维护。

### 板端资源

- **机载降落识别**使用 A733 VIPLite 和 INT8 NBG 模型。系统镜像提供 `libNBGlinker.so`、`libVIPhal.so` 和 `/dev/vipcore`。CPU 负责框解码、轮廓和位姿计算，桌面仿真从 ONNX 运行同一检测网络。
- **双目直接用两路 MIPI-CSI**，不占用 USB；USB 3.0 口留给外设。
- **系统只跑在 TF 卡上**：随机 IO 和掉电可靠性都受卡影响，根分区为 `commit=600`，关键文件写完必须 `sync` 再断电。
- **供电 5V/3A**；串口、USB、网口同时满载时需按峰值留余量。
