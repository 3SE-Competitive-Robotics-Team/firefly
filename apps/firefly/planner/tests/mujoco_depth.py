"""Produce axial millimetre images for the planner's map-revisit regression."""

import mujoco
import numpy as np


def main():
    model = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody>
        <geom name="wall" type="box" pos="4.15 0 1" size=".1 18 2"/>
        <geom name="back" type="box" pos="-4.05 0 1" size=".1 18 2"/>
        <body mocap="true" pos="2.25 .05 1.05">
          <geom name="box" type="box" size=".2 .55 .55"/>
        </body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    v, u = np.indices((13, 17))
    rays = np.stack(((u - 8) / 16, (v - 6) / 16, np.ones_like(u)), axis=-1).reshape(-1, 3)
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    sensor_z = rays[:, 2]
    geom_ids = np.empty(len(rays), dtype=np.int32)
    distance = np.empty(len(rays))
    for i in range(4000):
        cycle, phase = divmod(i, 400)
        facing = -1 if 40 <= phase < 120 else 1
        # Sweep eight metres, then re-enter from the opposite side on alternate laps.
        excursion = max(0, min(phase - 160, 320 - phase)) * 0.1
        y = 0.05 + excursion * (1 if cycle % 2 == 0 else -1)
        data.mocap_pos[0, 1] = 0.05 if phase < 60 else 16.0
        mujoco.mj_forward(model, data)
        directions = np.column_stack((facing * sensor_z, rays[:, 1], -facing * rays[:, 0]))
        mujoco.mj_multiRay(model, data, np.array([0.05, y, 1.05]), directions.ravel(),
                           None, True, -1, geom_ids, distance, None, len(rays), 20.)
        pixels = np.rint(1000 * sensor_z * np.maximum(distance, 0)).astype(np.uint16)
        if 324 <= phase < 350:
            pixels.fill(0)
        print(i, y, facing, *pixels)


if __name__ == "__main__":
    main()
