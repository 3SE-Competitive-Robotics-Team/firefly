"""firefly 无人机 MuJoCo 场景（MJCF）。

场景选择：`FIREFLY_SCENE` 环境变量（缺省 `warehouse`）。

- `warehouse`：Sketchfab "Warehouse FBX Model Free"（Nicholas-3D，
  CC-BY-4.0，见 `models/warehouse/license.txt`）：46m × 16m 室内仓库，
  视觉 mesh（`models/warehouse/structure.obj`，WetConcrete 贴图）+
  碰撞盒近似（两侧货架墙/两端墙/顶/地面，见 `_WAREHOUSE_COLLIDERS`）。
  无人机沿走廊 +x 飞行（起点 (2, 0, 1)，终点 (40, 0, 1)）。
- `boxes`：旧手捏箱子阵列（25 箱 + 中线柱 + 侧翼柱），ffmap 时代残留，
  仅供回归对照。

世界系 = 无人机起点系：`warehouse` 下起点 (2, 0, 1)，沿 +x 飞行。
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
- 仓库场景用自带 WetConcrete 贴图（`models/warehouse/`）；`boxes` 场景
  用运行时生成的随机点阵（见下）。
"""

import os
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

# 前方错落箱子：5 列 × 5 行网格（x∈[5,9]、y∈[2,6]），层数近矮远高，最高
# 1.7m——给下倾 20° 前视相机提供多高度垂直侧面（横向法向平面），深度测量
# 可约束 x/y 切向（深度只约束法向，地面朝上只约束 z）。层高三档：
# 单层 0.6m、双层 1.1m、三层 1.7m（第二/三层底面与下层重叠成阶梯）。
_BOX_COLS = (5.0, 6.0, 7.0, 8.0, 9.0)
_BOX_ROWS = (2.0, 3.0, 4.0, 5.0, 6.0)
_BOX_LAYERS = (
    (1, 1, 1, 1, 1),  # x=5 近处全单层
    (1, 1, 2, 1, 1),  # x=6
    (1, 2, 1, 2, 1),  # x=7
    (2, 1, 3, 1, 2),  # x=8 中心三层
    (1, 3, 2, 2, 2),  # x=9 远处最高
)
# 每层 (z 中心, 半高)：单层 0.3/0.3、二层 0.7/0.4、三层 1.2/0.5
_BOX_LAYER_GEOM = ((0.3, 0.3), (0.7, 0.4), (1.2, 0.5))


def _boxes_xml() -> str:
    """前方错落箱子：按 _BOX_LAYERS 层数 × _BOX_LAYER_GEOM 尺寸生成，
    网格点交替 pillar_a/pillar_b 材质。"""
    out = []
    for ci, x in enumerate(_BOX_COLS):
        for ri, y in enumerate(_BOX_ROWS):
            n = _BOX_LAYERS[ci][ri]
            mat = "pillar_a" if (ci + ri) % 2 == 0 else "pillar_b"
            for z, half in _BOX_LAYER_GEOM[:n]:
                out.append(
                    f'    <geom type="box" pos="{x} {y} {z}" '
                    f'size="0.25 0.25 {half}" material="{mat}"/>'
                )
    return "\n".join(out)


# 仓库走廊碰撞盒（x, y, z 中心 + 半尺寸）：两侧货架墙（y=±6.5，货架带
# |y|∈[5,8]，厚 0.2、高 3）、两端墙（x=0/46）、顶（z=5）、地面。
# 走廊带 y∈[-4,4] 全程净空（无人机沿 (x, y=0, z=1) 飞，无人机半宽
# 0.25 + PD 瞬态余量）。视觉 mesh 全量加载（10k 面），物理只用这些盒。
# 碰撞盒视觉透明 + 物理有效（rgba=0，见 _warehouse_colliders_xml）。
_WAREHOUSE_COLLIDERS = (
    # 两端墙（厚 0.3，高 5）
    (0.0, 0.0, 2.5, 0.15, 8.1, 2.5),
    (46.0, 0.0, 2.5, 0.15, 8.1, 2.5),
    # 顶 z=5（厚 0.2）
    (23.0, 0.0, 5.0, 23.2, 8.1, 0.1),
    # 两侧货架墙（x 全长，厚 0.2，高 3）
    (23.0, -6.5, 1.5, 23.2, 0.1, 1.5),
    (23.0, 6.5, 1.5, 23.2, 0.1, 1.5),
    # 地面（z=0，厚 0.1；mesh 地面 z≈0，双层保险）
    (23.0, 0.0, -0.05, 23.2, 8.1, 0.05),
)

#: 仓库资产目录（相对仓库根；models/ 已 ignore，不进 git）。
_WAREHOUSE_DIR = (
    Path(__file__).resolve().parent.parent.parent.parent.parent
    / "models"
    / "warehouse"
)


def _warehouse_colliders_xml() -> str:
    """碰撞盒 MJCF（视觉透明 + 物理有效：`rgba=0` 全透明故不遮挡 mesh
    画面，`contype/conaffinity` 缺省参与物理碰撞）。"""
    return "\n".join(
        f'    <geom type="box" pos="{x} {y} {z}" size="{hx} {hy} {hz}" rgba="0 0 0 0"/>'
        for x, y, z, hx, hy, hz in _WAREHOUSE_COLLIDERS
    )


def _warehouse_scene_xml() -> str:
    """仓库场景：视觉 mesh + 碰撞盒 + 方向光（与旧场景同灯光约定）。"""
    meshdir = _WAREHOUSE_DIR.as_posix()
    texdir = _WAREHOUSE_DIR.as_posix()
    return rf"""<mujoco model="firefly-warehouse">
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <compiler meshdir="{meshdir}" texturedir="{texdir}"/>

  <asset>
    <texture name="concrete" type="2d" file="WetConcrete_baseColor.png"/>
    <material name="wh" texture="concrete" texrepeat="6 6"/>
    <!-- 地面特征纹理：非周期随机点阵（boxes 场景同款，见模块 docstring）：
         仓库 WetConcrete 在近场梯度稀疏（frac>20≈0.03，boxes 为 0.06），
         KLT 在走廊段无角点可跟致 VIO 发散；叠一层随机点阵地面供前端跟踪 -->
    <texture name="dots" type="2d" file="{_DOTS_PATH}"/>
    <material name="ground_dots" texture="dots" texrepeat="24 6"/>
    <mesh name="wh_structure" file="structure.obj"/>
  </asset>

  <worldbody>
    <light name="sun_a" type="directional" dir="-0.3 -0.25 -0.92" diffuse="0.7 0.7 0.68"/>
    <light name="sun_b" type="directional" dir="-0.15 0.6 -0.78" diffuse="0.3 0.3 0.35"/>
    <light name="sun_c" type="directional" dir="0.75 0.1 -0.65" diffuse="0.22 0.22 0.25"/>

    <!-- 视觉：仓库 mesh 全量（10k 面，WetConcrete 贴图）。
         glTF 世界系（trimesh 全节点变换后）：X 宽 0..16、Y 高 0..5、
         Z 长 -46..0；导出时经 Rx(+90°X) + Rz(-90°Z) + y 平移，转为
         MuJoCo 系：X 长 0..46、Y 宽 -8..8、Z 高 0..5（见导出脚本备注）。
         geom 不额外变换，直接原位加载。
         碰撞盒按真实尺寸（见 _WAREHOUSE_COLLIDERS），与此对齐。 -->
    <!-- 视觉 mesh：只渲染不碰撞（contype/conaffinity=0），物理由下方透明碰撞盒承担 -->
    <geom name="warehouse" type="mesh" mesh="wh_structure" material="wh" contype="0" conaffinity="0"/>

    <!-- 物理：碰撞盒近似（见 _WAREHOUSE_COLLIDERS）；特征地面：与物理地面盒
         同尺寸的随机点阵平面（z=0.005 高出 mesh 地面防 z-fighting，只渲染
         不碰撞，物理地面由下方透明盒承担） -->
    <geom name="feature_ground" type="plane" pos="23 0 0.005" size="23.2 8.1 0.1" material="ground_dots" contype="0" conaffinity="0"/>
 {_warehouse_colliders_xml()}

    <!-- 无人机（freejoint 六自由度） -->
    <body name="drone" pos="2 0 1">
      <freejoint/>
      <geom type="box" size="0.15 0.15 0.04" rgba="0.90 0.70 0.20 1"/>
      <geom type="sphere" pos="0.25 0 0" size="0.06" rgba="0.80 0.20 0.20 1"/>
      <geom type="sphere" pos="-0.25 0 0" size="0.06" rgba="0.20 0.80 0.20 1"/>
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


def _boxes_scene_xml() -> str:
    """旧手捏箱子阵列场景（ffmap 时代残留，仅回归对照）。"""
    return rf"""<mujoco model="firefly">
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

    <!-- 前方错落箱子（P10.6 实验）：轨迹前方 x∈[5,9] 的 25 箱高低天际线，
         给下倾 20° 前视相机多高度垂直侧面 → 深度横向法向约束补 x/y。
         净距：箱子区 x≥5，轨迹最远 x=4（净距 ≥1m）；最高 1.7m 与轨迹
         z≤1.5 错开。demo 默认地图未同步（本实验专用）。 -->
{_boxes_xml()}

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


#: 当前场景 XML（模块导入时按 `FIREFLY_SCENE` 确定；`warehouse` 缺资产
#: 文件时回退 `boxes` 并警告——`models/` 已 ignore，CI 无资产）。
def _select_scene_xml() -> str:
    name = os.environ.get("FIREFLY_SCENE", "warehouse")
    if name == "warehouse":
        if (_WAREHOUSE_DIR / "structure.obj").is_file():
            return _warehouse_scene_xml()
        print("[scene] 仓库资产缺失（models/warehouse/structure.obj），回退 boxes 场景")
    elif name != "boxes":
        print(f"[scene] 未知场景 {name}，回退 boxes 场景")
    return _boxes_scene_xml()


SCENE_XML = _select_scene_xml()
