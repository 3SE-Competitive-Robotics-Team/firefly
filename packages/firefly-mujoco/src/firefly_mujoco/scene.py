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
    <texture name="checker" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.45 0.45 0.45" rgb2="0.65 0.65 0.65"/>
    <material name="ground" texture="checker" texrepeat="10 10"/>
  </asset>

  <worldbody>
    <!-- 方向光（无距离衰减，全程均匀；三个方向避免地面过平） -->
    <light name="sun_a" type="directional" dir="-0.3 -0.25 -0.92" diffuse="0.7 0.7 0.68"/>
    <light name="sun_b" type="directional" dir="-0.15 0.6 -0.78" diffuse="0.3 0.3 0.35"/>
    <light name="sun_c" type="directional" dir="0.75 0.1 -0.65" diffuse="0.22 0.22 0.25"/>

    <!-- 地面（棋盘纹理：KLT 特征来源之一） -->
    <geom name="ground" type="plane" size="35 35 0.1" material="ground"/>

    <!-- 沿途障碍（视觉特征 + 物理遮挡）：横排高墙挡中线，逼迫蛇形绕行。
         布局保证每个绕行窗口 ≥ 2.4m 净宽（含膨胀余量）：
         x=8 挡 y∈[2.8,5.2]（逼走上侧）→ x=14 挡 y∈[-1.2,2.8]（上侧通行）
         → x=20 挡 y∈[5.2,8.8]（逼走下侧）→ 回到中线到达目标。
         各墙高 3m（z∈[0,3]）无法飞越，只能横向绕；demo 默认地图与其同构 -->
    <geom type="box" pos="8  4.0 1.5" size="0.8 1.2 1.5" rgba="0.20 0.40 0.80 1"/>
    <geom type="box" pos="11 6.8 1.0" size="0.4 0.7 1.0" rgba="0.60 0.20 0.80 1"/>
    <geom type="box" pos="14 0.8 1.5" size="0.6 2.0 1.5" rgba="0.80 0.20 0.20 1"/>
    <geom type="box" pos="17 4.0 0.9" size="0.4 0.5 0.9" rgba="0.90 0.70 0.20 1"/>
    <geom type="box" pos="20 7.0 1.5" size="0.6 1.8 1.5" rgba="0.20 0.80 0.30 1"/>

    <!-- 无人机（freejoint 六自由度） -->
    <body name="drone" pos="1 4 1">
      <freejoint/>
      <geom type="box" size="0.15 0.15 0.04" rgba="0.90 0.70 0.20 1"/>
      <geom type="sphere" pos="0.25 0 0" size="0.06" rgba="0.80 0.20 0.20 1"/>
      <geom type="sphere" pos="-0.25 0 0" size="0.06" rgba="0.20 0.80 0.20 1"/>
      <!-- 双目（基线 0.1m）+ 深度相机，前向 +x，上 +z -->
      <camera name="cam_left" pos="-0.05 0 0" xyaxes="0 -1 0  0 0 1" fovy="60"/>
      <camera name="cam_right" pos="0.05 0 0" xyaxes="0 -1 0  0 0 1" fovy="60"/>
      <camera name="cam_depth" pos="0 0 0" xyaxes="0 -1 0  0 0 1" fovy="60"/>
      <site name="imu_site" pos="0 0 0"/>
    </body>
  </worldbody>

  <sensor>
    <gyro name="gyro" site="imu_site"/>
    <accelerometer name="accel" site="imu_site"/>
  </sensor>
</mujoco>
"""
