"""firefly 无人机 MuJoCo 场景（MJCF）：矩形围墙场 + 错落箱阵。

世界系：x∈[0,34]（长边）、y∈[0,20]（短边），篮球场（28×15）放大版。
无人机起点 (1,10,1)（西底线中点内 1m），沿 +x 飞往东底线后掉头返回。

布局（单位 m）：
- 围墙：四边，高 3m、厚 0.4m（视觉远景 + 安全边界）。
- 箱阵列（老场景同款）：主街（y=10）南北两块，1.3m 网格 0.5m 见方盒，
  层高种子随机（单层 0.6m、双层 1.1m、三层 1.7m），南北不同种子。
- 飞行净空：主街半宽 2.5m、掉头点 B=(30,10)/S=(1,10) 半径 3m 内无箱。

灯光约定：**全部用方向光**（`type="directional"`，无距离衰减，全程均匀）。

纹理约定（近场特征密度）：
- **非周期随机点阵**：周期棋盘会让 LK 整周期滑动而残差不变（毒数据），
  随机纹理无此问题。多尺度随机矩形在大中小距离段都提供 FAST 角点。
- 地面 4m/格；立柱/箱体靠不同 texrepeat 区分表观尺度。
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

# 场地尺寸（m）
FIELD_X = 34.0
FIELD_Y = 20.0
# 主街：y=10（A→B 航线），半宽 2.5m 内不摆箱；掉头点 S=(1,10)/B=(30,10)
# 半径 3m 内不摆箱；围墙内收 0.8m。
STREET_Y = 10.0
STREET_HALF = 2.5
TURN_CLEAR = 3.0
WALL_MARGIN = 0.8
# 箱阵列（老场景同款）：南北两块，1.3m 网格，0.5m 见方盒；
# 层高种子随机（南北不同种子，不对称），高低错落。
_ARRAY_SOUTH = (4.0, 30.0, 2.0, 7.0)
_ARRAY_NORTH = (4.0, 30.0, 13.0, 18.0)
_ARRAY_STEP = 1.3
_ARRAY_SEED_SOUTH = 11
_ARRAY_SEED_NORTH = 77
# 层高档（m）：0.6/1.1/1.7
_TOPS = (0.6, 1.1, 1.7)


def _array_boxes() -> list[tuple[float, float, int]]:
    """箱阵列：[(x, y, 层数 1/2/3)]。

    南北两块，主街/掉头点/围墙净空（见上），块内 1.3m 网格 + ±0.3m
    抖动，层高种子随机（0.4/0.35/0.25），落点率 85%。
    """
    out: list[tuple[float, float, int]] = []
    for x0, x1, y0, y1, seed in (
        (*_ARRAY_SOUTH, _ARRAY_SEED_SOUTH),
        (*_ARRAY_NORTH, _ARRAY_SEED_NORTH),
    ):
        rng = np.random.default_rng(seed)
        gx = np.arange(x0, x1, _ARRAY_STEP)
        gy = np.arange(y0, y1, _ARRAY_STEP)
        for bx in gx:
            for by in gy:
                x = float(bx + rng.uniform(-0.3, 0.3))
                y = float(by + rng.uniform(-0.3, 0.3))
                if abs(y - STREET_Y) < STREET_HALF:
                    continue
                if np.hypot(x - 1.0, y - STREET_Y) < TURN_CLEAR:
                    continue
                if np.hypot(x - 30.0, y - STREET_Y) < TURN_CLEAR:
                    continue
                if rng.random() > 0.85:
                    continue
                n = int(rng.choice([1, 2, 3], p=[0.4, 0.35, 0.25]))
                out.append((x, y, n))
    return out


_ARRAY = _array_boxes()
# 每层 (z 中心, 半高)：单层 0.3/0.3、二层 0.7/0.4、三层 1.2/0.5
_BOX_LAYER_GEOM = ((0.3, 0.3), (0.7, 0.4), (1.2, 0.5))


def _boxes_xml() -> str:
    """箱阵列：按 `_array_boxes` 逐层出 geom，交替 pillar_a/pillar_b 材质。"""
    out = []
    for i, (x, y, n) in enumerate(_ARRAY):
        mat = "pillar_a" if i % 2 == 0 else "pillar_b"
        for z, half in _BOX_LAYER_GEOM[:n]:
            out.append(
                f'    <geom type="box" pos="{x:.2f} {y:.2f} {z}" '
                f'size="0.25 0.25 {half}" material="{mat}"/>'
            )
    return "\n".join(out)


def _walls_xml() -> str:
    """四周围墙：高 3m、厚 0.4m（东西墙沿 y、南北墙沿 x）。"""
    return "\n".join(
        [
            '    <geom type="box" pos="0 10 1.5" size="0.2 10.0 1.5" material="pillar_a"/>',
            '    <geom type="box" pos="34 10 1.5" size="0.2 10.0 1.5" material="pillar_a"/>',
            '    <geom type="box" pos="17 0 1.5" size="17.0 0.2 1.5" material="pillar_b"/>',
            '    <geom type="box" pos="17 20 1.5" size="17.0 0.2 1.5" material="pillar_b"/>',
        ]
    )


SCENE_XML = rf"""
<mujoco model="firefly">
  <option timestep="0.005" gravity="0 0 -9.81"/>

  <asset>
    <!-- 非周期随机点阵（运行时生成，见模块 docstring）：地面与箱体共用
         一张纹理，靠不同 texrepeat 区分表观尺度 -->
    <texture name="dots" type="2d" file="{_DOTS_PATH}"/>
    <material name="ground" texture="dots" texrepeat="10 6"/>
    <material name="pillar_a" texture="dots" texrepeat="2 2"/>
    <material name="pillar_b" texture="dots" texrepeat="4 4"/>
  </asset>

  <worldbody>
    <!-- 方向光（无距离衰减，全程均匀；三个方向避免地面过平） -->
    <light name="sun_a" type="directional" dir="-0.3 -0.25 -0.92" diffuse="0.7 0.7 0.68"/>
    <light name="sun_b" type="directional" dir="-0.15 0.6 -0.78" diffuse="0.3 0.3 0.35"/>
    <light name="sun_c" type="directional" dir="0.75 0.1 -0.65" diffuse="0.22 0.22 0.25"/>

    <!-- 地面（随机点阵：KLT 近场特征主要来源；覆盖全场+外延） -->
    <geom name="ground" type="plane" pos="17 10 0" size="20 12 0.1" material="ground"/>

{_walls_xml()}

{_boxes_xml()}

    <!-- 无人机（freejoint 六自由度） -->
    <body name="drone" pos="1 10 1">
      <freejoint/>
      <geom type="box" size="0.15 0.15 0.04" rgba="0.90 0.70 0.20 1"/>
      <geom type="sphere" pos="0.25 0 0" size="0.06" rgba="0.80 0.20 0.20 1"/>
      <geom type="sphere" pos="-0.25 0 0" size="0.06" rgba="0.20 0.80 0.20 1"/>
      <!-- 双目（横向基线 0.05m，沿 y 侧向分开；基线须与视线垂直，
           前后分开的相机射线近乎共线 → 无侧向视差 → 立体失效）+ 深度相机，
           前向 +x，上 +z -->
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
