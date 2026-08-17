"""firefly-mujoco：无人机 MuJoCo 物理环境库。

提供 [`DroneEnv`](firefly_mujoco.env.DroneEnv)（物理步进 + IMU/双目灰度/
深度提取 + PD 控制）与跨语言消息契约 [`messages`](firefly_mujoco.messages)。
"""

from .env import DroneEnv
from .messages import (
    DepthImageMessage,
    GrayImageMessage,
    IMAGE_HEIGHT,
    IMAGE_SIZE,
    IMAGE_WIDTH,
    ImuMessage,
    OdomMessage,
    ReferenceMessage,
    TraceContext,
)
from .scene import SCENE_XML

__all__ = [
    "DroneEnv",
    "SCENE_XML",
    "IMAGE_WIDTH",
    "IMAGE_HEIGHT",
    "IMAGE_SIZE",
    "TraceContext",
    "ImuMessage",
    "GrayImageMessage",
    "DepthImageMessage",
    "ReferenceMessage",
    "OdomMessage",
]
