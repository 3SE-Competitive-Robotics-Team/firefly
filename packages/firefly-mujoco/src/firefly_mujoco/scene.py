"""RMUC 物理场景：按赛季与场地版本加载聚合碰撞体、无人机、IMU 与旋翼执行器。

视觉由 Bevy 从同坐标系的 field.glb 渲染；场景必须是注册表里的版本，碰撞资产显式存在。
"""

import json
from dataclasses import dataclass
from pathlib import Path

from firefly_workspace import load_toml, parse_scene, workspace_root
from firefly_workspace.game import load_team

ROOT = workspace_root(Path(__file__))

#: 默认场景；必须与 `configs/scene.toml` 的缺省一致。
DEFAULT_SCENE = "rmuc2027@1.0.0"

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
PAD_CLEARANCE = 0.03


@dataclass(frozen=True)
class Scene:
    """已注册场地；机库位置为 `(x, y, 支撑面高度)`，米。"""

    name: str
    hangars: dict[str, tuple[float, float, float]]

    def hangar(self, team: str) -> tuple[float, float, float]:
        if team not in self.hangars:
            raise ValueError("team 必须为 red 或 blue")
        return self.hangars[team]

    @property
    def pad(self) -> tuple[float, float, float]:
        return self.hangar(load_team())

    @property
    def id(self):
        return parse_scene(self.name)

    @property
    def dir(self) -> Path:
        """`models/<赛季>/v<版本>/`。"""
        return ROOT / self.id.dir

    @property
    def visual(self) -> Path:
        return self.dir / "field.glb"

    @property
    def collision(self) -> Path:
        return self.dir / self.id.collision


#: 已接入的场地版本。新增版本：目录 `models/<赛季>/v<版本>/` 加一行。
# CAD part 10, H 中心及支撑面高度。独立评分器嵌入同一份场地数据。
SCENES = {
    name: Scene(name, {team: tuple(point) for team, point in pads.items()})
    for name, pads in json.loads(Path(__file__).with_name("hangars.json").read_text()).items()
}


def resolve(name: str | None = None) -> Scene:
    """按名称取场景；缺省用默认场景，未注册的版本报错。"""
    resolved = name or DEFAULT_SCENE
    try:
        return SCENES[resolved]
    except KeyError:
        raise ValueError(
            f"未注册的场景 {resolved!r}；可用：{', '.join(SCENES)}"
        ) from None


def drone_pad(name: str | None = None, *, team: str | None = None) -> tuple[float, float, float]:
    """机体原点（世界系米）：场景停机坪表面加离地余量。"""
    x, y, surface = resolve(name).hangar(team or load_team())
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


# CAD part=10 的 z=1.900 甲板：旋转 45°、边长 0.4m、四角各切 0.04m。
# |x|,|y| <= 0.25455844，|x±y| <= 0.28284271；坐标相对机库中心。
_HANGAR_PLANES = (
    (1.,0.,0.25455844),(-1.,0.,0.25455844),
    (0.,1.,0.25455844),(0.,-1.,0.25455844),
    (1.,1.,0.28284271),(1.,-1.,0.28284271),
    (-1.,1.,0.28284271),(-1.,-1.,0.28284271),
)


def _polygon_area(points):
    return abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(points,points[1:]+points[:1])))/2


def _clip_polygon(points, nx, ny, bound):
    """凸多边形与 nx*x+ny*y <= bound 的闭半平面相交。"""
    if not points:
        return []
    output=[]
    previous=points[-1]
    before=nx*previous[0]+ny*previous[1]-bound
    for current in points:
        after=nx*current[0]+ny*current[1]-bound
        if (before<=0)!=(after<=0):
            fraction=before/(before-after)
            output.append(tuple(previous[i]+fraction*(current[i]-previous[i]) for i in range(2)))
        if after<=0:
            output.append(current)
        previous,before=current,after
    return output


def _cut_platform(polygon, low, high, x, y, surface):
    """去掉真实八边形甲板上方的体素余量；其余体积保持不变。"""
    original=[(polygon,low,high)]
    if high<=surface:
        return original,False
    planes=[(nx,ny,d+nx*x+ny*y) for nx,ny,d in _HANGAR_PLANES]
    intersection=polygon
    for nx,ny,bound in planes:
        intersection=_clip_polygon(intersection,nx,ny,bound)
    if _polygon_area(intersection)<1e-10:
        return original,False
    pieces=[]
    if low<surface:
        pieces.append((polygon,low,surface))
    remaining=polygon
    for nx,ny,bound in planes:
        outside=_clip_polygon(remaining,-nx,-ny,-bound)
        if _polygon_area(outside)>1e-10:
            pieces.append((outside,max(low,surface),high))
        remaining=_clip_polygon(remaining,nx,ny,bound)
    return pieces,True


def _collision_xml(asset_dir: Path, filename: str, scene: Scene) -> tuple[str,str]:
    """场地体素代理；甲板轮廓用凸棱柱差集保持 CAD 的支撑面。"""
    path=asset_dir/filename
    if not path.is_file():
        raise FileNotFoundError(f"RMUC 碰撞资产缺失：{path}；请先运行 firefly-cad-build 生成资产")
    report=json.loads(path.read_text(encoding="utf-8"))
    if not report["boxes"]:
        raise ValueError(f"RMUC 碰撞资产为空：{path}")
    assets,geometries=[],[]
    for cx,cy,cz,hx,hy,hz in report["boxes"]:
        polygon=[(cx-hx,cy-hy),(cx+hx,cy-hy),(cx+hx,cy+hy),(cx-hx,cy+hy)]
        pieces=[(polygon,cz-hz,cz+hz)]
        changed=False
        for x,y,surface in scene.hangars.values():
            next_pieces=[]
            for footprint,low,high in pieces:
                clipped,cut=_cut_platform(footprint,low,high,x,y,surface)
                next_pieces.extend(clipped);changed|=cut
            pieces=next_pieces
        if not changed:
            geometries.append(f'    <geom type="box" pos="{cx} {cy} {cz}" size="{hx} {hy} {hz}" rgba="0 0 0 0"/>')
            continue
        for footprint,low,high in pieces:
            name=f"hangar_cut_{len(assets)}"
            vertices=" ".join(f"{px:.9g} {py:.9g} {z:.9g}" for z in (low,high) for px,py in footprint)
            assets.append(f'<mesh name="{name}" vertex="{vertices}"/>')
            geometries.append(f'    <geom type="mesh" mesh="{name}" rgba="0 0 0 0"/>')
    return "\n".join(assets),"\n".join(geometries)


def build_scene(*, name: str | None = None, asset_dir: Path | None = None, team: str | None = None) -> str:
    """场地：Bevy 负责视觉（`field.glb`），MuJoCo 加载体素盒和甲板凸棱柱。"""
    scene = resolve(name)
    directory = asset_dir or scene.dir
    selected_team = team or load_team()
    collision_assets, collision = _collision_xml(directory, scene.id.collision, scene)
    return rf"""<mujoco model="firefly-{scene.id.season}">
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <asset>{collision_assets}</asset>

  <worldbody>
    <!-- 视觉：Bevy 载 {scene.id.dir / "field.glb"}；物理使用下面的透明碰撞体 -->
{collision}

    <!-- 无人机（freejoint 六自由度，停机坪上电） -->
{_drone_xml("%.9g %.9g %.9g" % drone_pad(scene.name, team=selected_team))}
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
    return ROOT / "configs" / "scene.toml"


def load_scene_name(path: Path | None = None) -> str:
    """读取可选场景配置并校验已注册；缺文件使用默认场景。"""
    values = load_toml(path or config_path())
    if unknown := values.keys() - {"scene"}:
        raise ValueError(f"未知场景配置项: {', '.join(sorted(unknown))}")
    name = values.get("scene", DEFAULT_SCENE)
    if not isinstance(name, str):
        raise ValueError("scene 必须是已注册的场景名称")
    resolve(name)
    return name
