# Firefly 系统架构

```mermaid
flowchart TB
    sensors["`传感器输入
实机 camera、imu
仿真 render、firefly-sim`"]
    operator["地面站 ffctl"]
    task["任务脚本 FlightRunner"]

    subgraph flight["机载计算 · iceoryx2 IPC"]
        vio["`firefly-rootslam
视觉惯性里程计`"]
        landing["landing · 下视精降识别"]
        planner["`planner
深度建图与轨迹规划`"]
        traj["traj · 轨迹采样"]
        fc["fc · 飞行模式与控制"]

        vio -->|里程计| planner
        vio -->|里程计| landing
        vio -->|里程计时钟| traj
        vio -->|里程计与姿态| fc
        planner -->|轨迹与心跳| traj
        traj -->|飞控参考| fc
        landing -->|降落目标| fc
    end

    sensors -->|双目与 IMU| vio
    sensors -->|深度| planner
    sensors -->|下视图像| landing
    sensors -->|IMU| fc
    operator -->|航点| planner
    task -->|任务指令| operator
    fc -->|FcStatus 模式快照| task
    fc -.->|FcStatus 模式快照| mimir
    operator -->|模式指令| fc
    fc -->|电机推力| actuators["`执行端
实机 motor
仿真 firefly-sim`"]
    flight -.-> mimir["`mimir · 观测录制
log.rrd`"]
```

`planner`、`traj` 和 `landing` 各自在进程内通过 `firefly-odometry` 接收并校验里程计。
`landing` 按图像时间查询历史样本，`traj` 按样本时间采样轨迹，`planner` 将实测位置和速度转换为规划初态，并为这个起点指定零加速度。

## 航点执行

```mermaid
flowchart LR
    task["任务航点序号、位置和到达方式"] --> cli["ffctl planner goal / pass"]
    cli -->|当前点、到达方式、后继位置| planner["planner"]
    odom["局部里程计"] --> planner
    planner -->|可制停的轨迹| traj["traj"] -->|参考状态| fc["FC TRACK"]
    planner -->|目标身份、passed、finished| task
    task -->|停稳点确认后 HOLD| fc
```

停稳点等待规划完成、里程计进入 0.15 米且速度不超过 0.12 m/s，持续 0.5 秒后切入 HOLD。途经点和后继位置一起投递。planner 在实测位置进入途经点 0.35 米球内时切换到后继目标并锁存 `passed`，任务在 TRACK 下继续。后继目标默认停稳；连续途经时任务用下一条带后继位置的命令接续。目标身份采用命令时间戳，任务按访问序号区分相同坐标。
