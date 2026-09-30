# 坐标与定位数学契约

基础实现：`firefly-base::{FrameId, RigidTransform, FrameTree}`。
跨模块、跨传感器的坐标变换使用这一组类型；数值优化器内部允许矩阵运算。

## 方向与树

`T_target_source` 将 source 中的列向量坐标转换到 target：

```text
p_target = R_target_source p_source + t_target_source
T_a_c = T_a_b T_b_c
T_b_a = inverse(T_a_b)

map
 └─ odom          全局定位给出的修正，可变化
     └─ body      VIO 给出的连续局部位姿
         ├─ left_camera
         ├─ right_camera
         └─ depth_camera
```

长度为米，角度为弧度，四元数为 Hamilton xyzw、source→target 主动旋转。
IPC 中 JPL world→body 四元数的同一组分量按 Hamilton 解释恰为 body→world；
`OdomMessage::body_pose(parent)` 集中处理这一边界。消息布局不变。

- `compose` 检查中间坐标系，不能把 map 位姿与 odom 位姿直接混乘。
- `FrameTree::set` 拒绝自环、闭环、多父节点；错误不修改树。
- `lookup(target, source)` 经公共祖先组合，未知/不连通坐标系返回错误。
- 点应用旋转和平移；物理速度等自由向量只旋转换基。
- 扭量顺序为 `[rotation, translation]`；`Ad(T) = [[R,0],[skew(t)R,R]]`，
  协方差换基为 `P_target = Ad(T) P_source Ad(T)ᵀ`。
- 矩阵构造拒绝非有限值、缩放、反射和非法齐次行；四元数必须单位化。

树是一个几何快照，不承担时钟、缓存、插值或自动外推。调用方负责让每条边对应
查询所需时刻。不得因为两条消息都很新，就把不同采样时刻的位姿直接相除。
这项约束也适用于 VIO 初始化/重启：更换 odom 原点必须重建整条坐标链。

## 控制边界

飞控位置、速度、姿态反馈以及起飞原点、Hold 锚点均保持在 odom 系。
校正里程计仅与同时间戳原始里程计配对，计算：

```text
T_map_odom = T_map_body inverse(T_odom_body)
p_odom_ref = inverse(T_map_odom) p_map_ref
v_odom_ref = R_odom_map v_map_ref
```

配对历史有界（2 秒、最多 256 帧），不做跨时刻近似；对齐的新鲜度取两条样本接收
时间中较早者。未配对、重复或过期样本不能维持地图参考有效性。
参考航向通过旋转航向向量转换，角速度通过该向量在水平面投影的导数转换。

速度转换描述同一物理速度，不把全局定位修正的跳变当成真实速度。
校正不会直接移动局部反馈或 Hold 锚点；Track 的地图目标误差仍可能随重定位改变，
因此大幅重定位的参考平滑与规划重启属于额外的运行策略。

## 固定启动条件

`configs/gicp.toml` 的 `[origin]` 显式声明局部 VIO 原点在地图中的 `position` 和
`yaw`。RMUC 默认位置为 `[-13,0,0.405]` 米、航向零；这些是固定部署的已知先验，
不是从在线 GroundTruth 取出的估计器初值。改变启动位置或初始航向必须同步配置。

原始 VIO 仍用传感器初始化，位置在局部系归零。地图查询只消费经过上述变换的
CorrectedOdometry，不能在校正流缺失时用原始局部位置查询地图。该配置不提供任意
位置冷启动或绑架重定位能力。

## 与 VINS-Fusion 的关系

本机参考：`~/Projects/VINS-Fusion`，版本
`be55a937a57436548ddfb1bd324bc1e9a9e828e0`。

[VINS-Fusion pose_graph.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/loop_fusion/src/pose_graph.cpp)
的 `optimize4DoF` 用航向差建立漂移旋转，`optimize6DoF` 使用
`R_drift = R_optimized R_vioᵀ`，两者均以
`t_drift = p_optimized - R_drift p_vio` 建立坐标变换。
`loop_fusion` 通过位姿图优化获得校正位姿；本项目的误差态 EKF 是独立的融合实现，
不能将其残差、增益或 Joseph 更新称为上述源码的直接移植。

## 融合误差约定

令 `D=T_map_odom`、`V=T_odom_body`、观测 `Y=T_map_body`。
统一使用 odom 系右扰动：

```text
D_true = D Exp(e_odom)
D_observed = Y inverse(V)
z = Log(inverse(D) D_observed)
R_odom = Ad(V) R_body Ad(V)ᵀ
S = P + R_odom
K = P inverse(S)
delta = K z
D_new = D Exp(delta)
P_posterior = (I-K) P (I-K)ᵀ + K R_odom Kᵀ
P_new = Jr(delta) P_posterior Jr(delta)ᵀ
```

GICP 的 `J=[R skew(p), -R]` 与右乘增量对应，Hessian 的逆是 body 右扰动协方差。
视觉位姿的经验相机协方差先经 `Ad(T_body_camera)` 转到 body，再进入相同融合入口。
若修正被限幅，增益的相应行同步缩放后参与协方差更新。

这是小误差线性化的松耦合模型：把 VIO 位姿作为给定输入，没有传播其全部不确定度
及与视觉观测的相关性。视觉协方差也是经验模型；通过代数回归不等于统计一致性已验收。

## 回归约束

- 任意链组合、逆查询、错误坐标拒绝、树无环；点与向量分开变换。
- SE(3) 共轭与伴随一致，右雅可比由中心有限差分校验。
- 航向为 0°、±90°、170° 时，世界 +X 观测应沿正确方向减小误差。
- 更换地图坐标原点/姿态不改变融合结果的物理意义。
- 地图目标、速度和航向正确转换；不同时刻及重复消息不能刷新对齐。
- Hold 受垂向位移扰动后仍保持进入模式时的目标高度，控制力指向恢复方向。
