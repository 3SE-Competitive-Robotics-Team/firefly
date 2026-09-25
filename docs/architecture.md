# firefly 自主无人机系统架构

```mermaid
flowchart TD
    subgraph SIM_SRC["仿真源：firefly-sim（MuJoCo 200Hz 物理，被控对象）"]
        SIMP["发布 IMU / 双目 / 深度 / 真值<br/>发布状态 PlantState + 机体 Airframe<br/>订阅飞控指令 Control（按机体几何合成 wrench）"]
    end

    subgraph FC_APP["apps/fc（飞控进程，1kHz）"]
        FLIGHT["firefly-flight（唯一实现）<br/>姿态内环 + 位置外环<br/>Airframe::allocate → 4 电机推力（唯一饱和点）"]
    end

    subgraph HW_SRC["实机源（同一端口，待接入）"]
        CAM["Intel RealSense D430<br/>双目灰度 + 深度"]
        IMU_SENSOR["板载 IMU<br/>加速度计 + 陀螺仪"]
        DRV["realsense-rust / 串口驱动<br/>发布同一组 Firefly/* 话题"]
        CAM --> DRV
        IMU_SENSOR --> DRV
    end

    subgraph PUBSUB["firefly-pubsub（iceoryx2 zero-copy，双路共用话题层）"]
        TOPIC_SENS["Firefly/Imu · CameraLeft/Right"]
        TOPIC_DEPTH["Firefly/Depth"]
        TOPIC_ODOM["Firefly/Odometry<br/>位置/速度/姿态"]
        TOPIC_REF["Firefly/Reference"]
        TOPIC_PLANT["Firefly/PlantState 200Hz<br/>Firefly/Airframe 1Hz"]
        TOPIC_CTRL["Firefly/Control 1kHz<br/>4 电机推力"]
    end

    SIMP -->|发布| TOPIC_SENS
    SIMP -->|发布| TOPIC_DEPTH
    SIMP -->|发布| TOPIC_PLANT
    DRV -->|发布| TOPIC_SENS
    DRV -->|发布| TOPIC_DEPTH

    subgraph VIO_APP["apps/vio（VIO 进程）"]
        INPUT["IceoryxInput<br/>SensorInput 端口适配 · IMU/双目时间戳配对"]
        subgraph VIO["firefly-vio*（对照 OpenVINS 分层）"]
            TYPES["firefly-vio-types<br/>JPL 四元数 / SO(3) / SE(3) 纯类型"]
            CORE["firefly-vio-core<br/>传感器数据 · IMU 标定 · KLT 前端 · 传播/更新数学"]
            INIT["firefly-vio-init<br/>静态/动态初始化 + 外参时延标定"]
            MSCKF["firefly-vio<br/>MSCKF 编排：State 滑动窗口<br/>UpdaterMSCKF · VioManager"]
            TYPES --> CORE
            CORE --> INIT
            CORE --> MSCKF
            INIT --> MSCKF
        end
        INPUT --> VIO
    end

    TOPIC_SENS -->|订阅| INPUT

    subgraph PLAN_APP["apps/planner（规划进程）"]
        subgraph MAP["firefly-map"]
            GRID["GridMap 占据体素 + 膨胀层 + 虚拟地面/天花板"]
            RAY["深度 raycast 在线更新"]
        end
        subgraph PLAN["firefly-planner"]
            ASTAR["firefly-search<br/>A* 引导（膨胀层 26 邻域）"]
            MINCO["firefly-trajectory<br/>MINCO 参数化（段长自适应 + 拐点 waypoint）"]
            OPT["LBFGS + 双层 clearance<br/>(硬 0.1m / 软 0.5m)"]
            ROUGH["roughlyCheck 内循环<br/>碰撞段局部 A* 绕行约束"]
            COST["firefly-cost<br/>平滑/时间/可行/障碍/集群"]
            ASTAR --> MINCO --> OPT
            COST --> OPT
            OPT <--> ROUGH
        end
        FSM["10Hz 重规划状态机<br/>EXEC/REPLAN/GEN + 安全检查"]
        MAP --> PLAN
        PLAN --> FSM
    end

    subgraph SWARM["集群（可选）"]
        PEER["其他机轨迹<br/>iceoryx2 广播"]
    end

    MSCKF -->|Odom| TOPIC_ODOM
    TOPIC_ODOM -->|订阅| FSM
    TOPIC_DEPTH -->|订阅| RAY
    PEER -->|peer 轨迹| COST
    FSM -->|MINCO 轨迹| TOPIC_REF
    TOPIC_REF -->|订阅| FLIGHT
    TOPIC_PLANT -->|订阅| FLIGHT
    TOPIC_ODOM -->|订阅| FLIGHT
    FLIGHT -->|控制指令| TOPIC_CTRL
    TOPIC_CTRL -->|订阅·取最新| SIMP
    TOPIC_REF -.->|仅日志/互锁| SIMP
    DRV -.->|实机：同一话题层| FLIGHT
```

## 控制链（`firefly-flight` + `apps/fc`）

推力只能沿机体 `+Z`（4 旋翼共同作用）：滚转/俯仰力矩来自推力差，偏航力矩来自
旋翼反扭矩，水平加速必须靠倾斜——四旋翼与“世界系全驱动扳手”模型的根本区别。

```
参考（Firefly/Reference）
  → position_mode/angle_mode（期望力/力矩，不限幅）
  → Airframe::allocate（逐电机限幅，全栈唯一饱和点）→ 4 电机推力
  → Firefly/Control（1kHz，被控对象每步取最新）
  → 被控对象按机体几何合成 wrench（Firefly/Airframe 给的唯一一份几何）
```

- **参数归属**：质量/惯量/气动阻尼/旋翼位置/旋向/单电机推力上限/反扭矩系数
  由被控对象经 `Firefly/Airframe` 发布（1Hz 电平，改几何不动飞控）；
  增益/倾角限幅在 `configs/fc.toml`（缺键回落 `firefly-flight` 默认值）。
- **反馈分工**：内环姿态由飞控自估（陀螺积分 + 加速度计水平修正，航向取
  `Firefly/Odometry` 的姿态，JPL `q_GtoI` 的换算见 `apps/fc/src/vio.rs` 与其单测）；
  位置/速度取 VIO 估计，未到达时回落 `PlantState` 真值（起飞前正常）。
- **节拍**：控制律无积分项，dt 不进控制；被控对象状态陈旧（>50ms）时飞控
  **不发指令**，由被控对象悬停兜底（失效保护，不是第二套跟踪律）。
- **待观测**：侧向漂移的量化只在闭环实测（`fc/debug/*` 进 rrd：推力/倾角/
  姿态误差/tick 率/晚到/饱和）。当前闭环仍受 VIO 估计限制：悬停时估计自漂
  ~0.4m/s（`--script` 路径同现象，与飞控无关），被控对象会跟随估计漂移；
  用真值反馈的 FC 闭环已实测悬停稳态（推力恒 = mg、倾角 <0.1°、tick 1kHz）。

## VIO 单帧算法流程

```mermaid
flowchart TD
    subgraph FRONTEND["KLT 双目前端（feed_stereo · 每帧循环）"]
        PRE["读入当前帧<br/>img_curr_l / img_curr_r"]
        DETECT["补点检测（跑在上一帧 img_last 上）：<br/>左目网格 FAST 提取 → 新点左→右 KLT 同 id 配对"]
        EXTRAP["运动外推：LK 初值 = 上帧位置 + 最近两帧位移"]
        TLK["时间 LK：img_last → img_curr<br/>(4 层金字塔 + 21×21 窗口)"]
        RANSAC["基础矩阵 RANSAC 剔除误匹配"]
        PAIR["按 id 合并左右结果：<br/>双目成对 / 左单目 / 右单目"]
        DB["测量入库（去畸变归一化坐标）"]
        ADVANCE["Move forward in time：<br/>pts/img/mask/ids_last ← 当前帧<br/>（曾漏写此段 → 跟踪器永远停在首帧）"]
        PRE --> DETECT --> EXTRAP --> TLK --> RANSAC --> PAIR --> DB --> ADVANCE
    end

    subgraph MSCKF["MSCKF 估计（VioManager · 每相机时刻）"]
        PROPAG["IMU 传播至图像时刻 + 克隆增广"]
        COLLECT["收集候选：<br/>丢失特征 + 滑出克隆窗口的特征"]
        RGATE["重投影门控：<br/>z>0、深度 ≤ 上限、残差阈值"]
        TRIANG["多帧 DLT 三角化求解特征 3D 位置<br/>cond 数 / 深度上下限门控"]
        JRES["逐特征：残差 + 雅可比（FEJ 线性化点）<br/>零空间投影 → chi² 检验<br/>深度自适应噪声行归一化"]
        COMPRESS["Givens 测量压缩降维"]
        UPDATE["EKF 更新（位置/速度修正限幅）<br/>删除已用特征 + 边缘化最旧克隆"]
        PROPAG --> COLLECT --> RGATE --> TRIANG --> JRES --> COMPRESS --> UPDATE
    end

    DB -.->|"跟踪中断的特征成为下一帧候选"| COLLECT
    ADVANCE ==>|"推进 last 状态 · 进入下一帧循环"| PRE

    classDef critical fill:#fff3cd,stroke:#b8860b,color:#000;
    class ADVANCE critical;
```
