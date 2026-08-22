"""firefly 无人机 MuJoCo 场景（MJCF）。

世界系 = demo 地图系：无人机起点 (1, 4, 1)，沿 +x 飞行到目标。
相机（双目 + 深度）前向 +x，给 KLT 提供特征；地面棋盘纹理提供纹理特征。

灯光约定：**全部用方向光**（`type="directional"`）。此前用带 `pos` 的默认
定点光，光强随距离衰减——无人机沿 +x 飞到 27m 后地面亮度从均值 42 跌到
10（近乎全黑）。方向光无距离衰减，全程光照均匀（实测地面均值 140~160，
无饱和），保证整条任务路径上双目/深度画面可读。
"""

SCENE_XML = r"""
<mujoco model="firefly">
  <option timestep="0.005" gravity="0 0 -9.81"/>

  <asset>
    <!-- 地面棋盘：texrepeat 35 → 约 2m 一格。曾用 10（7m 大格），
         相机高度 1~1.5m 时画面内地格过大、FAST 角点整幅不足 30 个，
         KLT 轨迹寿命骤减、VIO 无更新可用。 -->
    <texture name="checker" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.45 0.45 0.45" rgb2="0.65 0.65 0.65"/>
    <material name="ground" texture="checker" texrepeat="35 35"/>
    <!-- 立柱棋盘贴面：纯色盒只有直线棱边（无角点），贴上高频棋盘后
         每根柱面提供数十个 FAST 角点，是前向视差的主要来源 -->
    <texture name="checker_pillar" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.85 0.55 0.15" rgb2="0.15 0.25 0.45"/>
    <material name="pillar_a" texture="checker_pillar" texrepeat="3 3"/>
    <texture name="checker_pillar2" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.80 0.20 0.20" rgb2="0.90 0.90 0.85"/>
    <material name="pillar_b" texture="checker_pillar2" texrepeat="3 3"/>
  </asset>

  <worldbody>
    <!-- 方向光（无距离衰减，全程均匀；三个方向避免地面过平） -->
    <light name="sun_a" type="directional" dir="-0.3 -0.25 -0.92" diffuse="0.7 0.7 0.68"/>
    <light name="sun_b" type="directional" dir="-0.15 0.6 -0.78" diffuse="0.3 0.3 0.35"/>
    <light name="sun_c" type="directional" dir="0.75 0.1 -0.65" diffuse="0.22 0.22 0.25"/>

    <!-- 地面（棋盘纹理：KLT 特征来源之一） -->
    <geom name="ground" type="plane" size="35 35 0.1" material="ground"/>

    <!-- 沿途障碍（视觉特征 + 物理遮挡）：中线上一串孤立高柱（0.8~1.2m
         见方，高 3m 无法飞越），逼无人机沿 y≈4 小幅左右蛇形绕行——绕行
         单个立柱容易、不切连续墙角，规划器稳定可解（连续墙会使 MINCO
         优化卡"stuck"，见 planner 维护项）。demo 默认地图与其同构。 -->
    <geom type="box" pos="9  4.0 1.5" size="0.4 0.5 1.5" material="pillar_a"/>
    <geom type="box" pos="12 6.5 1.0" size="0.4 0.7 1.0" material="pillar_b"/>
    <geom type="box" pos="16 4.0 1.5" size="0.4 0.6 1.5" material="pillar_a"/>
    <geom type="box" pos="19 1.8 0.9" size="0.4 0.5 0.9" material="pillar_b"/>
    <geom type="box" pos="22 3.6 1.5" size="0.4 0.5 1.5" material="pillar_a"/>

    <!-- 无人机（freejoint 六自由度） -->
    <body name="drone" pos="1 4 1">
      <freejoint/>
      <geom type="box" size="0.15 0.15 0.04" rgba="0.90 0.70 0.20 1"/>
      <geom type="sphere" pos="0.25 0 0" size="0.06" rgba="0.80 0.20 0.20 1"/>
      <geom type="sphere" pos="-0.25 0 0" size="0.06" rgba="0.20 0.80 0.20 1"/>
      <!-- 双目（基线 0.1m）+ 深度相机，前向 +x，上 +z -->
      <!-- 双目（横向基线 0.05m，沿 y 侧向分开，对照 Intel RealSense D430 的结构基线
           50mm）+ 深度相机，前向 +x，上 +z。
           注意：基线必须与视线垂直（横向），前后(y=0 沿 x)分开的相机射线
           近乎共线 → 无侧向视差 → 立体无法解深度（VIO 三角化必败）。 -->
      <camera name="cam_left" pos="0 -0.025 0" xyaxes="0 -1 0  0 0 1" fovy="70.88"/>
      <camera name="cam_right" pos="0 0.025 0" xyaxes="0 -1 0  0 0 1" fovy="70.88"/>
      <camera name="cam_depth" pos="0 0 0" xyaxes="0 -1 0  0 0 1" fovy="70.88"/>
      <site name="imu_site" pos="0 0 0"/>
    </body>
  </worldbody>

  <sensor>
    <gyro name="gyro" site="imu_site"/>
    <accelerometer name="accel" site="imu_site"/>
  </sensor>
</mujoco>
"""
