# firefly-void 总体规划（唯一综合文档）

> 状态：进行中 · 协调者：Hermes · 实施者：Command Code CLI（deepseek-v4-flash，--yolo）
> 本文是本项目唯一的过程/决策/规划文档；完成后的演进史归 git log，不写散文档。

## 1. 目标

以 FAST-LIVO2 为技术蓝本，在 firefly 仓库内新建独立模块 **firefly-void**（app `void` + 内部 crates），实现完整的里程计与建图算法（紧耦合深度-惯性-视觉，ESIKF），交付：

1. 可编译、可运行、有测试的完整 Rust 源码（`apps/void` + `crates/firefly-void-*`）
2. 与现有 sim / viz 的集成接线（订阅 sim 全部话题，发布结果供 viz 订阅）
3. Typst 学术论文（`thesis/`），章节结构完全对齐 FAST-LIVO2 论文，达到可投稿状态

## 2. 硬约束

- **不动现有代码**：VIO、GICP 及其他现有模块一律不修改不删除；集成只增不改。唯一例外：`Cargo.toml` workspace.members 用 `crates/*`、`apps/*` 通配，新 crate 自动纳入，**无需改任何现有文件**
- 技术路线严格对照 FAST-LIVO2 论文（arXiv:2408.14035）与源码（`~/Projects/fast_livo2/`，含论文 PDF）
- 官方源码与论文放 `~/Projects/fast_livo2/`（主仓库外），不污染项目根目录
- 遵守仓库 AGENTS.md 全部规范：注释中文、禁过程叙事、禁 println!、错误用 firefly-error、依赖进 workspace.dependencies、`unsafe_code = forbid`、clippy pedantic deny
- 新 crate 命名 `firefly-void-*`，edition 2024，DDD 分层，领域层不依赖应用层
- 代码实现全部经 Command Code CLI 派发（`--yolo`，nohup 后台 + 日志轮询），协调者只写 brief、review、给意见

## 3. 传感器适配（FAST-LIVO2 → firefly 仿真环境的关键差异）

FAST-LIVO2 原始输入 = LiDAR + 相机 + IMU。firefly 仿真（MuJoCo）提供的输入：

| FAST-LIVO2 | firefly-void 对应实现 |
|---|---|
| LiDAR 点云（scan 100~500k 点/s） | **深度图 `Firefly/Depth` 反投影成结构化点云**（320×240 @10Hz，每帧最多 ~76.8k 点，下采样后与 LiDAR scan 等价；深度噪声模型已有：disparity σ∝z²、5-15% 空洞、边缘 1px 加粗） |
| 相机（全局快门/卷帘） | 双目左目灰度 `Firefly/CameraLeft`（320×240 @10Hz，配 `Firefly/CameraPair` 事件同步） |
| IMU 100~250Hz | `Firefly/Imu` @100Hz |
| LiDAR-相机外参 | 深度相机与左目共面（MuJoCo 同刚体），外参近似单位阵，参数保留可配 |

firefly-void 的测量模型因此命名为**深度-惯性-视觉里程计（DIVO）**：深度点云替代 LiDAR 点云走「点-平面残差 + 束发散角噪声模型」（深度相机无束发散角，该项简化为深度不确定度 σ(z) 模型，直接用仿真噪声参数），其余（ESIKF 顺序更新、统一体素地图、视觉地图点、参考补丁更新、法向量精化、按需光线投射、曝光时间估计）完整对照论文。

## 4. 架构与数据流

```
Firefly/Imu(100Hz) ─┐
Firefly/CameraPair ─┼─► apps/void（iceoryx2 订阅）
Firefly/Depth ──────┘        │
                             ▼
        firefly-void-esikf: ESIKF（19 维流形状态：R,p,v,bg,ba,g,τ）
          ├─ 前向/后向传播（IMU）
          ├─ 顺序更新：①深度点-平面残差迭代 ②金字塔稀疏直接视觉迭代
          └─ 收敛后更新地图
        firefly-void-map: 自适应体素地图（哈希+八叉，0.5m 根体素，
          叶体素=局部平面 q/n/Σ，视觉地图点挂三层 8×8 补丁金字塔，
          参考补丁 NCC+视角评分，法向量离线精化，环形缓冲滑窗）
                             │
                             ▼
   发布：Firefly/VoidOdom（OdomMessage @10Hz，对齐现有 odometry 消费端）
        Firefly/Viz（复用现有 VizMessage：POSE/LINE_STRIP/POINTS/SCALARS
          实体前缀 void/*：void/odom、void/traj、void/map_points、
          void/patches、void/health，sim_time 时间轴对齐）
```

- viz 侧：firefly-viz 已把 VizMessage 全 kind 写 rerun，`void/*` 实体**零改动**直接显示（kind 是通用编码）
- 算法模块可替换性：`void` app 依赖 `firefly-void::Odometry` trait（propagate/update/map 接口），实现与接线分离

## 5. Crates 拆分（DDD）

| crate | 职责 | 依赖 |
|---|---|---|
| `firefly-void-types` | 状态流形（SO(3)×R¹⁹）、boxplus/boxminus、传感器数据结构、外参/标定配置 | firefly-error, nalgebra |
| `firefly-void-esikf` | ESIKF 核心：传播、顺序更新、迭代收敛、协方差（Algorithm 1） | void-types |
| `firefly-void-map` | 自适应体素地图：几何构建/更新/成熟判定、视觉地图点生成、参考补丁评分、法向量精化（独立线程）、按需光线投射 | void-types, purecv |
| `firefly-void-measure` | 测量模型：深度点-平面残差+噪声（VI-A/B 节）、稀疏直接视觉残差+仿射扭曲+光度/曝光（VII 节）、外点剔除（遮挡/深度不连续） | void-types, void-map, purecv |
| `firefly-void`（lib+app 接口层，供 app 用） | Odometry trait + 管线组装（scan 重组合→传播→深度更新→视觉更新→建图） | 上述全部 |
| `apps/void` | iceoryx2 接线：订阅 Imu/CameraPair/Depth，发布 OdomMessage/Viz，配置加载 configs/void.toml，Ctrl-C 优雅退出 | firefly-pubsub, void |

## 6. 实施阶段与 brief（串行，前一批 review 通过并 commit 后才派下一批）

brief 文件：`/tmp/firefly_void/brief_N.md`（只给官方源码位置与差距，让实施者自己读）

- **P1** 骨架+核心：全部 6 个 crate 空壳（能编译、clippy 干净）+ void-types 完整 + esikf 完整（含单元测试：传播/boxplus/顺序更新收敛性）
- **P2** 体素地图：map crate 完整（几何/视觉点/参考补丁/法向精化/光线投射，单元测试用合成平面）
- **P3** 测量模型：measure crate 完整（深度残差/视觉对齐/曝光估计/外点剔除，单元测试用合成数据）
- **P4** 管线+集成：void lib 组装 + apps/void 接线 + configs/void.toml；MuJoCo 闭环 e2e（sim+void+viz 三进程，验证 odom 与 GT 误差 < 阈值、viz 正常显示）
- **P5** 论文：Typst，`thesis/`，章节目录 = FAST-LIVO2 原文（I Introduction / II Related Works / III System Overview / IV ESIKF with Sequential Update / V Local Mapping / VI Depth Measurement Model / VII Visual Measurement Model / VIII Experiments / IX Conclusion），内容替换为 DIVO 实现；`thesis/output/` 入 .gitignore；实验数据来自 P4 e2e 实测
- **P6** 终审：全仓库 fmt/clippy/test + e2e 复跑 + 论文编译验证 + 交付清单核对

## 7. 派发协议（每个阶段相同）

1. 协调者写 brief_N.md（含：官方源码路径+行号区间、本阶段验收标准、禁止事项：不动现有文件/不写过程叙事注释/不加依赖到非 workspace）
2. `nohup cmd -p "$(cat brief_N.md)" --yolo --session firefly-void-pN > /tmp/firefly_void/pN.log 2>&1 &`（脱离会话，防 exec 超时）
3. 日志轮询直至完成（-p 模式结束才写日志属正常）
4. 协调者亲自验收：`cargo fmt --check && cargo clippy --workspace --all-targets && cargo test`（PATH 含 ~/.cargo/bin）+ 读 diff 对照官方源码抽查
5. 问题 → review 意见写 /tmp/firefly_void/review_pN.md → `cmd --resume <session> -p "$(cat review_pN.md)"` 喷回原 session 修
6. 通过 → 用户确认后中文 conventional commit（`feat(void): ...`）

## 8. 验收清单（完成时逐项核对）

- [x] `cargo build --workspace` 编译通过，`cargo clippy --workspace --all-targets` 零 deny，`cargo test` 全绿
- [x] apps/void 能跑 MuJoCo 闭环（sim → void → viz 三进程），odom 发布、viz 显示 void/* 实体
- [x] 现有代码零改动（`git diff --stat` 仅新增文件 + .gitignore）
- [x] thesis/ Typst 源码入库可编译，章节目录与 FAST-LIVO2 一致，thesis/output/ 已 gitignore
- [x] 过程文档仅本文档一份
- [x] `~/Projects/fast_livo2/` 含源码+论文，主仓库根无污染

## 9. 进度日志

- 2026-08-31 05:00 侦察完成（topics/cmd/论文/源码）；本文档建立
- 2026-08-31 05:10 P1 派发（cmd --yolo，session firefly-void-p1，日志 /tmp/firefly_void/p1.log）
- 2026-08-31 05:12 第一次派发失败（--session 误用于新会话，应为 -n）；05:13 修正重派，进程 84838 确认运行（session 文件 d020632e，transcript 持续增长）
- 2026-08-31 05:50 84838 撞 100 轮上限退出（exit 8）：骨架+workspace 编译已通过，剩 state.rs 测试类型错误；05:52 --resume 续跑（PID 86129，--max-turns 200）只修测试
- 2026-08-31 06:05 P1 验收通过（fmt/clippy 干净、21 测试全绿）→ commit b3fd676（含 DESIGN.md）；06:08 P2 派发（体素地图，brief_2.md，同 session 续，PID 89300，--max-turns 200）
- 2026-08-31 06:55 P2 撞累计轮数上限（exit 8）——教训：--resume 轮数预算按 session 累计，后续每阶段独立开新 session（-n）而非 --resume。现场良好：map lib clippy 0 命中、测试 9/9 已过，仅剩测试 lint 尾巴；06:57 续跑收尾（PID 93112，--max-turns 400）
- 2026-08-31 07:10 P2 验收通过（fmt/clippy 干净、21 测试全绿、公式抽查与论文 (12) 式一致）→ commit e96a846；07:12 P3 派发（测量模型，brief_3.md，**新 session** firefly-void-p3，PID 94432，--max-turns 400）
- 2026-08-31 08:05 P3 撞 400 轮上限（exit 8）：fmt 已过、47 测试全绿，仅剩 measure 库 11 个 clippy 错误（单字符名/相似名/复杂类型等风格项）；08:07 review_p3.md 派出小 session 专修（PID 2597，120 轮）
- 2026-08-31 08:45 P3 终验通过（fmt OK、clippy 0、全仓 484 测试全绿、雅可比有限差分对拍 <1e-6 为真断言）→ commit 49c126a；08:50 P4 派发（管线组装+apps/void 接线+e2e，brief_4.md，新 session firefly-void-p4，PID 4459，400 轮；发布 topic 定为 Firefly/VoidOdom 避免与 vio 冲突）
- 2026-08-31 09:40 P4 撞 400 轮上限：接线+配置+e2e 脚本全部就位、全仓编译过；e2e 调试抓到真实算法问题——首个深度平面建立时单帧更新打飞状态（跳 0.5m）。09:42 派 P4fix（新 session，PID 20461）：管线层加深度更新门控（0.1m/3° 每帧）+健康计数+e2e 复测
- 2026-08-31 10:15 P4 终验通过（fmt OK、clippy 0、493 测试全绿、门控实现抽查符合 brief、iceoryx 残留已清理）→ commit 131f09c。e2e 75s：ATE-RMS 0.1782m、7s 爆点消除、6 次启动期门控拒绝。P4 四阶段全部入库，P5 论文派发
- 2026-08-31 10:55 P5 终验通过（typst compile 0 error、14 页 PDF、9 章节标题与 FAST-LIVO2 目录对齐、ATE 三数字与实测一致、bib 26 条真实文献）→ commit 68d7556（prek 钩子格式修正一并入库）。10:58 P6 终审派发（brief_6.md，新 session PID 26869）：全仓一致性+验收清单勾选+e2e 复跑+边界复查
- 2026-08-31 11:40 P6 终审结论：代码/论文/边界全 PASS，但 e2e 复跑 2 轮 FAIL（0.3966/0.5989m vs P4 0.1782m）——P4 是幸运抽样（sim 无种子），启动期首个深度平面的慢速偏置注入绕过单帧门控。协调者决策：修算法稳健性而非锁种子（不动 sim 边界）。11:45 派 P6fix（PID 29115）：①深度门控加速度度通道（主修复，两轮失败均经深度注入）②启动期 10 帧加严（内点×2+门控×0.5）③连续拒绝保护④e2e --runs 3 多轮统计，硬验收=3 轮全 <0.3m；论文 VIII 节同步改三轮统计
- 2026-08-31 11:00 P6 终审：fmt/clippy/test 四门全绿（493 测试）、边界零触碰、论文数字与 P4 实测逐项对得上（0.1782m 复算一致）、验收清单勾选；e2e 复跑两轮均 FAIL（ATE-RMS 0.3966m / 0.5989m，>20% 偏差）——归因：仿真噪声无固定种子，启动期首个深度平面偏置注入速度被滤波器当真实运动跟踪，收敛依赖噪声抽样；§4 图 stale crate 名修正为 firefly-void-esikf。待协调者验收后统一提交
