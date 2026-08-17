# ARUCO 标签支持：可行性评估

> 目标：评估是否需要在 `firefly-vio` 中实现 OpenVINS 的 `TrackAruco`
> （ArUco 标签检测 → 作为 SLAM 特征参与估计），以及外部库 vs 自研的取舍。
> 结论先行：**现阶段不引入 ARUCO**，理由见下；若未来需要，优先自研轻量
> 检测器，而非引入 OpenCV 绑定。

## 1. OpenVINS 中 ARUCO 的作用

C++ 侧 `TrackAruco`（`ov_core/src/track/TrackAruco.cpp`，约 240 行）本身很薄：
它调用 OpenCV 的 `cv::aruco::detectMarkers` 做检测，把每个标签的**一个角点**
当作一个"特征"写入 `FeatureDatabase`，标签 id 直接复用为特征 id（`featid`），
后续完全走 SLAM 更新管线（`UpdaterSLAM` 中按 `featid < max_aruco_features`
区分 aruco/普通特征的 sigma 与 chi2 参数）。

收益：

- 标签是**高置信度、可复识别**的陆标，抗跟踪漂移；
- 与 SLAM 特征共享同一套 EKF 更新（无需新更新器）；
- 对静止初始化/回环场景提供稳定锚点。

代价：

- 需要一个完整的 ArUco 检测器（OpenCV `detectMarkers` 内部约数千行：
  自适应阈值 → 轮廓提取 → 四边形拟合 → 透视矫正 → 字典解码）；
- 传感器上必须实际贴有标签，否则纯增开销。

## 2. Rust 生态现状（2025 年）

| 方案 | 说明 | 结论 |
|---|---|---|
| `opencv` crate（`opencv = "0.9x"`） | Rust 绑定，链接系统 OpenCV；功能完整（含 aruco 模块），但与仓库"纯 Rust、无 C++ 运行时"的依赖原则冲突；CI/交叉编译变重 | 不推荐 |
| `imageproc` + 自研 | 仓库已自研 FAST-9/LK/CLAHE/高斯金字塔等 OpenCV 等价物（`firefly-vio-core/src/track/`），再自研 ArUco 检测器是同一思路的延续；需要补：轮廓提取（connected components）、四边形拟合（approxPolyDP 等价）、透视变换、字典解码 | 可行，工作量约 1.5–2k 行 |
| 不引入 | `max_aruco_features = 0` 时 OpenVINS 语义等价于无 aruco（本仓库当前实现即此状态），SLAM 更新器已完整移植 | **当前推荐** |

## 3. 自研检测器的技术要点（若未来实施）

对照 OpenCV `aruco::detectMarkers`（DICT_4X4_50 等）：

1. **预处理**：灰度 → 直方图均衡（已有 `track/histogram.rs`）→ 自适应阈值
   （`cv::adaptiveThreshold` 的 ADAPTIVE_THRESH_MEAN_C，块大小 ~7）；
2. **候选四边形**：二值图 connected components → 轮廓 → `approxPolyDP`
   等价（Douglas–Peucker，tol = 0.03 × 周长）→ 恰好 4 个顶点且凸；
3. **透视矫正**：4 点 homography（DIP 标准解，SVD 求 H）→ 固定 50×50 网格
   重采样 → 二值化；
4. **字典解码**：50×50 → 7×7（DICT_4X4）单元取中值 → 与字典模板按
   Hamming 距离匹配（阈值 ≤ 2 bit）→ 得到 id 与角点；
5. **去重/拒绝**：同 id 多候选取最小重投影误差（对应 OpenCV 的
   `refineDetectedMarkers` 可后置）。

依赖：全部可用 nalgebra（SVD/homography）+ 现有图像工具完成，无新 crate。

## 4. 与现有移植的衔接（若实施）

- `firefly-vio-core`：新增 `track/aruco.rs`（检测器）与 `track_aruco.rs`
  （`TrackAruco` 等价：`feed_new_camera` → 检测 → `update_feature` 入库，
  `change_feat_id`/`database` 等复用 `TrackerBase`）；
- `firefly-vio`：`StateOptions.max_aruco_features > 0` 时，
  `VioManager` 需要第二特征库（`track_aruco`）并把它并入
  `do_feature_propagate_update` 的 `feats_slam` 收集（对照 C++ 的
  `feat1`/`feat2` 双库逻辑）；`UpdaterSLAM` 的 aruco sigma/chi2 分支
  （按 `featid < max_aruco_features` 选择 `_options_aruco`）同步补上；
- 初始化：动态初始化器（`firefly-vio-init`）的 `camera_extrinsics` 等
  已就绪，标签作为 SLAM 特征参与无额外改动。

## 5. 建议

1. **当前（本文档落盘时）**：维持 `max_aruco_features = 0`，不引入 ARUCO；
   文档与代码中的 TODO 保持现状即可。
2. **若竞赛/产品需求落地**：优先走"自研轻量检测器 + DICT_4X4_50"路线，
   一次性把 `detectMarkers` 的核心路径移植进 `firefly-vio-core`（复用既有
   RANSAC/直方图/金字塔模块），预计 2 人周；避免引入 OpenCV 系统依赖。
3. 在决定实施前，先用真机标签数据验证检测率/误检率是否满足需求
   （ArUco 的误检会直接进入 EKF，chi2 能挡一部分但非全部）。
