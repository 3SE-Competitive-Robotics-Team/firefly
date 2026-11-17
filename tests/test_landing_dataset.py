"""CIF 数据集标签遵循可见 CAD 面积，缺测位姿保持显式。"""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec=importlib.util.spec_from_file_location("landing_dataset",Path(__file__).resolve().parents[1]/"scripts/collect_landing_dataset.py")
dataset=importlib.util.module_from_spec(spec)
spec.loader.exec_module(dataset)


def test_masks_require_h_paint_and_keep_pixel_edge_coordinates():
    mask=np.zeros((100,200,3))
    assert dataset.mask_labels(mask)==[]
    mask[20:60,40:120,0]=1
    assert dataset.mask_labels(mask) is None
    mask[30:40,60:70,2]=1
    assert dataset.mask_labels(mask)==[[0,.4,.4,.4,.4]]


def test_pose_interpolation_never_extrapolates_or_bridges_long_gaps():
    first=[0.,0.,1.,0.,0.,0.,1.]
    second=[.2,0.,1.,0.,0.,0.,-1.]
    poses={0:first,20_000_000:second}
    assert dataset.pose_at(poses,0)==(first,False)
    pose,interpolated=dataset.pose_at(poses,10_000_000)
    assert interpolated and pose[:3]==pytest.approx([.1,0.,1.])
    assert np.linalg.norm(pose[3:])==pytest.approx(1.)
    assert dataset.pose_at(poses,-1)==(None,False)
    assert dataset.pose_at({0:first,100_000_000:second},50_000_000)==(None,False)
