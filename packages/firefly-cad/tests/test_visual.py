"""CAD surface colors must not depend on the tessellation's triangle shapes."""

import importlib.util
import json

import numpy as np

from firefly_workspace import workspace_root


spec = importlib.util.spec_from_file_location(
    "blender_field",
    workspace_root() / "scripts/blender_field.py",
)
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


def test_uniform_cad_surface_keeps_one_material():
    # The affected platform's source color; both a sliver and its neighboring
    # broad triangle belong to the same uniformly colored CAD surface.
    colors = np.tile([1.0, 1.0, 0.8878815174102783], (2, 1))
    actual = exporter.remap_body(colors)
    np.testing.assert_allclose(actual[0], actual[1], rtol=0, atol=0)
    assert actual.max() < 0.3


def test_source_white_and_colored_markings_are_preserved():
    colors = np.array(
        [
            [1.0, 1.0, 1.0],
            [0.98, 0.98, 0.98],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    actual = exporter.remap_body(colors)
    np.testing.assert_allclose(actual[:2], np.full((2, 3), 0.9))
    np.testing.assert_array_equal(actual[2:], colors[2:])


def test_uncolored_and_neutral_surfaces_keep_body_materials():
    colors = np.array([[-1.0, -1.0, -1.0], [0.5, 0.5, 0.5]])
    actual = exporter.remap_body(colors)
    np.testing.assert_allclose(actual, [[0.060, 0.065, 0.085], [0.095, 0.105, 0.130]])


def test_surface_texture_does_not_invent_markings_and_is_deterministic():
    profile = json.loads(
        (workspace_root() / "scripts/field_surfaces/rmuc2027.json").read_text()
    )
    atlas = exporter.surface_atlas(profile, "ground", width=1024)
    assert atlas.min() > 0.15 and atlas.max() < 0.35
    assert np.ptp(atlas, axis=2).max() < 0.04
    assert (
        exporter.surface_atlas(profile, "platform", width=1024).mean()
        > atlas.mean() + 0.2
    )
    np.testing.assert_array_equal(
        atlas, exporter.surface_atlas(profile, "ground", width=1024)
    )


def test_paint_applies_only_to_reviewed_upward_neutral_parts():
    vertices = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    faces = np.array([[0, 1, 2], [0, 2, 1], [0, 1, 2], [0, 1, 2], [0, 1, 2]])
    colors = np.array(
        [
            [1.0, 1.0, 0.89],
            [1.0, 1.0, 0.89],
            [1.0, 1.0, 0.89],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
        ]
    )
    profile = {"ground_parts": [9], "platform_parts": [85]}
    masks = exporter.textured_faces(
        vertices, faces, np.array([9, 9, 999, 9, 9]), colors, profile
    )
    np.testing.assert_array_equal(masks["ground"], [True, False, False, False, False])
    assert not masks["platform"].any()
