"""RMUC2026 物理场景：聚合碰撞体、无人机、IMU 与旋翼执行器。

视觉由 Bevy 从同坐标系的 field.glb 渲染；碰撞资产必须显式存在。
"""

import json
from pathlib import Path

SCENE_NAME = "rmuc2026"

#: 机体（微型机）：质量 219g，外观对齐 `models/drone/drone.glb`（0.16×0.118×0.043 m
#: 包围盒）。旋翼 site 与 `firefly-flight` 默认 X 布局同构（半轴距 45mm、对角 127mm）——
#: 这里的几何是飞控分配与合成的唯一来源，经 `Firefly/Airframe` 发布，飞控不另抄一份。
_DRONE_MASS = 0.219
#: 单个电机质量（kg）：其余重量归机身盒（质量分布直接决定转动惯量）
_MOTOR_MASS = 0.021
#: X 布局半轴距（m）：旋翼位于 (±ARM, ±ARM, 0)
_ARM = 0.045
#: 机身盒半径（m，半尺寸）
_DRONE_BODY_SIZE = "0.055 0.042 0.021"
#: 电机球半径（m）
_MOTOR_RADIUS = 0.007

#: 4 旋翼 `(sx, sy, spin)`（顺序与 `firefly-flight` 默认一致）：0 前右、1 后左（旋向 +1），
#: 2 前左、3 后右（旋向 −1）；site 名与顺序即飞控分配的几何来源。
ROTORS = ((1, -1, 1.0), (-1, 1, 1.0), (1, 1, -1.0), (-1, -1, -1.0))

#: 反扭矩系数 `c_τ`（m）：偏航力矩 = `c_τ · 推力`，进 actuator 的 gear 得到旋翼级模型
#:（对照 `mujoco_menagerie/skydio_x2/x2.xml` 的 `gear="0 0 1 0 0 ±0.0201"`；
#: 小型桨典型 0.01~0.02 m）。
ROTOR_TORQUE_COEFFICIENT = 0.016
#: 推重比：单电机推力上限 = TWR·重量/4（写进 `ctrlrange`，飞控从模型读回）
ROTOR_THRUST_TO_WEIGHT = 3.0

#: 停机坪离地余量（m）：机体盒半高 0.021 加余量——上电停在地面且无穿模。
#: 场地表面高度见 [`PAD`]。
PAD_CLEARANCE = 0.03

#: 停机坪 `(x, y, 场地表面高度)`，世界系米；主台阶顶面为 0.375 m。
PAD = (-13.0, 0.0, 0.375)


def drone_pad() -> tuple[float, float, float]:
    """RMUC 停机坪机体原点，世界系米。"""
    x, y, surface = PAD
    return (x, y, surface + PAD_CLEARANCE)


def _drone_xml(pos: str) -> str:
    """无人机 body（freejoint 六自由度 + 4 旋翼 site + IMU）。

    质量/惯量由这里的几何与 `mass` 决定（飞控经 `Firefly/Airframe` 收到实测值）。
    变动几何即变动被控对象，无第二处需要同步。
    """
    frame_mass = _DRONE_MASS - len(ROTORS) * _MOTOR_MASS
    rotors = "".join(
        f'\n      <site name="rotor{i}" pos="{sx * _ARM:g} {sy * _ARM:g} 0"/>'
        f'\n      <geom type="sphere" pos="{sx * _ARM:g} {sy * _ARM:g} 0"'
        f' size="{_MOTOR_RADIUS:g}" mass="{_MOTOR_MASS:g}" rgba="0.25 0.25 0.28 1"/>'
        for i, (sx, sy, _) in enumerate(ROTORS)
    )
    return rf"""    <body name="drone" pos="{pos}">
      <freejoint/>
      <geom type="box" size="{_DRONE_BODY_SIZE}" mass="{frame_mass:g}" rgba="0.90 0.70 0.20 1"/>{rotors}
      <site name="imu_site" pos="0 0 0"/>
    </body>"""


def _rotor_actuators_xml() -> str:
    """4 旋翼 actuator：site 上的 6 维 wrench（推力 + 反扭矩），旋向由 gear 符号编码。

    对照 `mujoco_menagerie/skydio_x2/x2.xml`：`motor site=... gear="0 0 1 0 0 ±c_τ"`——
    旋翼力/力矩由引擎合成，单电机推力上限
    写在 `ctrlrange`（`autolimits` 负责夹）并经 `Firefly/Airframe` 发布给飞控。
    执行器滞后（`dyntype="filter"`）与气动（`fluidshape`）官方两个四旋翼模型都没建模，
    属标定项，未开启。
    """
    max_thrust = ROTOR_THRUST_TO_WEIGHT * _DRONE_MASS * 9.81 / len(ROTORS)
    actuators = "\n".join(
        f'    <motor name="rotor{i}" site="rotor{i}" gear="0 0 1 0 0 {spin * ROTOR_TORQUE_COEFFICIENT:g}"'
        f' ctrlrange="0 {max_thrust:.4f}"/>'
        for i, (*_, spin) in enumerate(ROTORS)
    )
    return f"""  <actuator>
{actuators}
  </actuator>"""


#: RMUC2026 资产目录（相对仓库根；models/ 已 ignore，不进 git）。
_RMUC_DIR = (
    Path(__file__).resolve().parent.parent.parent.parent.parent
    / "models"
    / "rmuc2026"
)


def _rmuc_collision_xml() -> str:
    """聚合碰撞盒 MJCF（视觉归 Bevy，此处只放透明碰撞体）。

    盒集合来自 `firefly-cad-collide`（实心体素 + 贪心 AABB，保留凹结构，
    对照 `models/rmuc2026/rmuc2026_collision.json`）。
    """
    path = _RMUC_DIR / "rmuc2026_collision.json"
    if not path.is_file():
        raise FileNotFoundError(f"RMUC 碰撞资产缺失：{path}；请先运行 firefly-cad-collide 生成资产")
    report = json.loads(path.read_text(encoding="utf-8"))
    if not report["boxes"]:
        raise ValueError(f"RMUC 碰撞资产为空：{path}")
    return "\n".join(
        f'    <geom type="box" pos="{cx} {cy} {cz}" size="{hx} {hy} {hz}" rgba="0 0 0 0"/>'
        for cx, cy, cz, hx, hy, hz in report["boxes"]
    )


def build_scene() -> str:
    """RMUC2026 场地：Bevy 负责视觉（`field.glb`），MuJoCo 只加载聚合碰撞盒。"""
    return rf"""<mujoco model="firefly-rmuc2026">
  <option timestep="0.005" gravity="0 0 -9.81"/>

  <worldbody>
    <!-- 视觉：Bevy 载 models/rmuc2026/field.glb；物理只有下面的透明碰撞盒 -->
{_rmuc_collision_xml()}

    <!-- 无人机（freejoint 六自由度，停机坪上电） -->
{_drone_xml("%g %g %g" % drone_pad())}
  </worldbody>

{_rotor_actuators_xml()}

  <sensor>
    <gyro name="gyro" site="imu_site"/>
    <accelerometer name="accel" site="imu_site"/>
  </sensor>
</mujoco>
"""


def config_path() -> Path:
    """仓库 `configs/scene.toml`（世界观单一来源，对照 AGENTS 配置约定）。"""
    return (
        Path(__file__).resolve().parent.parent.parent.parent.parent
        / "configs"
        / "scene.toml"
    )


def load_scene_name(path: Path | None = None) -> str:
    """读 `configs/scene.toml` 的 `scene`；缺文件即报错。"""
    import tomllib

    with open(path or config_path(), "rb") as f:
        name = tomllib.load(f)["scene"]
    if name != SCENE_NAME:
        raise ValueError(f"仅支持场景 {SCENE_NAME!r}，收到 {name!r}")
    return SCENE_NAME
