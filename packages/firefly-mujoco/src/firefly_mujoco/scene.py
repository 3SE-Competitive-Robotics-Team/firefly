"""firefly 无人机 MuJoCo 场景（MJCF）。

世界系 = demo 地图系：无人机起点 (1, 4, 1)，沿 +x 飞行到目标。
相机（双目 + 深度）前向 +x，给 KLT 提供特征；地面棋盘纹理提供纹理特征。
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
    <light name="light0" pos="5 5 8" dir="-0.5 -0.5 -1"/>
    <light name="light1" pos="25 -5 6" dir="-0.5 0.5 -1"/>

    <!-- 地面（棋盘纹理：KLT 特征来源之一） -->
    <geom name="ground" type="plane" size="35 35 0.1" material="ground"/>

    <!-- 沿途障碍（视觉特征 + 物理遮挡） -->
    <geom type="box" pos="8 2.0 0.5" size="1.5 0.4 0.5" rgba="0.20 0.40 0.80 1"/>
    <geom type="box" pos="12 -2.0 1.0" size="0.4 1.2 1.0" rgba="0.80 0.20 0.20 1"/>
    <geom type="box" pos="16 2.0 0.8" size="0.3 0.3 0.8" rgba="0.20 0.80 0.30 1"/>
    <geom type="box" pos="20 -1.0 1.2" size="1.0 0.3 1.2" rgba="0.90 0.70 0.20 1"/>
    <geom type="box" pos="24 1.0 0.6" size="0.3 1.5 0.6" rgba="0.60 0.20 0.80 1"/>

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
