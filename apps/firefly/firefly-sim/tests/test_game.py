"""队伍配置与机库核心支撑面的几何契约。"""
import json

import mujoco
import numpy as np
import pytest

from firefly_workspace.game import load_team
from firefly_mujoco.scene import resolve, build_scene, drone_pad, _cut_platform, _polygon_area


def test_team_config_is_strict(tmp_path):
    config = tmp_path / "game.toml"
    assert load_team(config) == "red"
    for team in ("red", "blue"):
        config.write_text(f'team = "{team}"\n')
        assert load_team(config) == team
    for content in ('team = "green"', 'team = 1', 'teams = "red"'):
        config.write_text(content)
        with pytest.raises(ValueError):
            load_team(config)


@pytest.mark.parametrize("team", ["red", "blue"])
@pytest.mark.parametrize("offset", [(0.,0.),(.07,.02),(.1,.04),(-.1,-.04)])
def test_spawn_is_over_selected_hangar_and_settles_on_its_surface(tmp_path, team, offset):
    scene = resolve()
    x, y, surface = scene.hangar(team)
    (tmp_path / scene.id.collision).write_text(json.dumps({"boxes": [[x, y, surface/2, .3, .3, surface/2+.125]]}))
    model = mujoco.MjModel.from_xml_string(build_scene(asset_dir=tmp_path, team=team))
    data = mujoco.MjData(model)
    np.testing.assert_allclose(data.qpos[:3], drone_pad(team=team))
    data.qpos[:2]+=offset
    mujoco.mj_forward(model, data)
    assert all(c.dist >= -.002 for c in data.contact)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    np.testing.assert_allclose(data.qpos[:3], [x+offset[0],y+offset[1],surface+.021], atol=.002)
    assert np.linalg.norm(data.qvel) < .01


def test_core_correction_preserves_volume_outside_the_cad_octagon():
    polygon=[(-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)]
    pieces,changed=_cut_platform(polygon,1.7,2.1,0.,0.,1.9)
    assert changed
    assert sum(_polygon_area(p)*(high-low) for p,low,high in pieces)==pytest.approx(.4-.1568*.2)
    outside=[(1.5,-.5),(2.5,-.5),(2.5,.5),(1.5,.5)]
    assert _cut_platform(outside,1.7,2.1,0.,0.,1.9)==([(outside,1.7,2.1)],False)
