"""firefly 无人机 MuJoCo 场景（MJCF）。

世界系 = demo 地图系：无人机起点 (1, 4, 1)，沿 +x 飞行到目标。
相机（双目 + 深度）前向 +x，给 KLT 提供特征。

灯光约定：**全部用方向光**（`type="directional"`）。此前用带 `pos` 的默认
定点光，光强随距离衰减——无人机沿 +x 飞到 27m 后地面亮度从均值 42 跌到
10（近乎全黑）。方向光无距离衰减，全程光照均匀（实测地面均值 140~160，
无饱和），保证整条任务路径上双目/深度画面可读。

纹理约定（近场特征密度，AGENTS.md VIO 调试状态）：
- **非周期随机点阵**替代棋盘：棋盘是周期图案，LK 可整周期滑动而残差
  不变——滑格错配不会被 χ² 拒绝，作为毒数据进入更新；随机纹理无周期
  可滑。多尺度随机矩形在大中小三个距离段都提供 FAST 角点。
- 地面 texrepeat 8（一格 8.75m，纹素 ~117px/m）；掠射角下 10m 外地面
  纵向压缩到个位像素行是透视固有属性，近场 <8m 才是有效特征区。
- **`--script` VIO 验证轨迹**（x∈[-2,4]、y∈[3,5] 盒内振荡）够不到中线
  立柱（x≥9），故在盒外两侧加 6 根 3m 高柱：前飞时始终有柱在 2~8m 内
  入画（水平半 FOV≈43°，横向 2.5m 的柱从 ~2.7m 前方起可见），且柱顶
  （z=3m）在 5m 处仰角 ~22°，填充上半幅视野。demo 默认地图已同步。
"""

import struct
import tempfile
import zlib
from pathlib import Path

import numpy as np

# 随机点阵纹理缓存（确定性种子；进程并发时先写临时文件再原子改名）
_TEX_DIR = Path(tempfile.gettempdir()) / "firefly_textures"
_DOTS_TEX = _TEX_DIR / "random_dots_1024.png"


def _write_png(path: Path, img: np.ndarray) -> None:
    """最小灰度 PNG 编码器（stdlib：zlib + struct，避免引入 pillow）。"""
    h, w = img.shape
    raw = b"".join(b"\x00" + row.tobytes() for row in img)  # 每行 filter=0

    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)  # 8bit 灰度
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )
    path.write_bytes(png)


def _ensure_dots_texture() -> Path:
    """生成（或复用）非周期随机纹理：1024²灰度，三档尺度随机矩形。

    对照 OpenVINS Simulator 的随机纹理生成：大块定基调、中块造角点、
    小块补高频。固定种子保证各进程/各次运行纹理一致。
    """
    if _DOTS_TEX.exists():
        return _DOTS_TEX
    rng = np.random.default_rng(7)
    size = 1024
    img = np.full((size, size), 128, np.uint8)
    for n, lo, hi in [(24, 64, 256), (160, 16, 64), (900, 4, 16)]:
        for _ in range(n):
            y = int(rng.integers(0, size))
            x = int(rng.integers(0, size))
            hh = int(rng.integers(lo, hi))
            ww = int(rng.integers(lo, hi))
            img[y : y + hh, x : x + ww] = np.uint8(rng.integers(35, 220))
    _TEX_DIR.mkdir(parents=True, exist_ok=True)
    tmp = _DOTS_TEX.with_suffix(".tmp")
    _write_png(tmp, img)
    tmp.replace(_DOTS_TEX)
    return _DOTS_TEX


_DOTS_PATH = _ensure_dots_texture()

SCENE_XML = rf"""
<mujoco model="firefly">
  <option timestep="0.005" gravity="0 0 -9.81"/>

  <asset>
    <!-- 非周期随机点阵（运行时生成，见模块 docstring）：地面与立柱共用
         一张纹理，靠不同 texrepeat 区分表观尺度 -->
    <texture name="dots" type="2d" file="{_DOTS_PATH}"/>
    <material name="ground" texture="dots" texrepeat="8 8"/>
    <material name="pillar_a" texture="dots" texrepeat="2 2"/>
    <material name="pillar_b" texture="dots" texrepeat="4 4"/>
  </asset>

  <worldbody>
    <!-- 方向光（无距离衰减，全程均匀；三个方向避免地面过平） -->
    <light name="sun_a" type="directional" dir="-0.3 -0.25 -0.92" diffuse="0.7 0.7 0.68"/>
    <light name="sun_b" type="directional" dir="-0.15 0.6 -0.78" diffuse="0.3 0.3 0.35"/>
    <light name="sun_c" type="directional" dir="0.75 0.1 -0.65" diffuse="0.22 0.22 0.25"/>

    <!-- 地面（随机点阵：KLT 近场特征主要来源） -->
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

    <!-- VIO 验证盒两侧立柱：--script 轨迹在 x∈[-2,4]、y∈[3,5] 振荡，
         中线立柱（x≥9）全程不可见。这 6 根柱在轨迹侧翼 |y-4|=2.5m
         （柱缘距路径极端 ≥1.15m，PD 瞬态安全），前向相机在 2~8m 内
         持续可见，为 MSCKF 更新提供带视差的近场特征。demo 默认地图
         与其同构。 -->
    <geom type="box" pos="0.5 1.5 1.5" size="0.35 0.35 1.5" material="pillar_a"/>
    <geom type="box" pos="2.0 1.5 1.5" size="0.35 0.35 1.5" material="pillar_b"/>
    <geom type="box" pos="3.5 1.5 1.5" size="0.35 0.35 1.5" material="pillar_a"/>
    <geom type="box" pos="0.5 6.5 1.5" size="0.35 0.35 1.5" material="pillar_b"/>
    <geom type="box" pos="2.0 6.5 1.5" size="0.35 0.35 1.5" material="pillar_a"/>
    <geom type="box" pos="3.5 6.5 1.5" size="0.35 0.35 1.5" material="pillar_b"/>

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
      <camera name="cam_left" pos="0 -0.025 0" xyaxes="0 -1 0  0.3420 0.0000 0.9397" fovy="70.88"/>
      <camera name="cam_right" pos="0 0.025 0" xyaxes="0 -1 0  0.3420 0.0000 0.9397" fovy="70.88"/>
      <camera name="cam_depth" pos="0 0 0" xyaxes="0 -1 0  0.3420 0.0000 0.9397" fovy="70.88"/>
      <site name="imu_site" pos="0 0 0"/>
    </body>
  </worldbody>

  <sensor>
    <gyro name="gyro" site="imu_site"/>
    <accelerometer name="accel" site="imu_site"/>
  </sensor>
</mujoco>
"""
