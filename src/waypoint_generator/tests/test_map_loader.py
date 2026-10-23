import numpy as np
import pytest

from waypoint_generator.generator import load_map


@pytest.mark.parametrize("suffix", ["png", "pgm"])
def test_trinary_and_saved_slam_format(map_factory, suffix):
    path = map_factory(np.array([[255, 0, 205]], dtype=np.uint8), suffix=suffix)
    grid = load_map(path)
    assert grid.cells.tolist() == [[0, 100, -1]]
    assert grid.image_path == path.with_suffix("." + suffix)
    assert (grid.width, grid.height) == (3, 1)


def test_negation_and_threshold_equality(map_factory):
    grid = load_map(map_factory(np.array([[0, 255, 128]], dtype=np.uint8),
                               negate=1, free_thresh=128 / 255))
    assert grid.cells.tolist() == [[0, 100, -1]]


def test_alpha_is_unknown(map_factory):
    rgba = np.array([[[255, 255, 255, 0], [255, 255, 255, 255]]], dtype=np.uint8)
    assert load_map(map_factory(rgba)).cells.tolist() == [[-1, 0]]


def test_rotated_origin_row_flip_and_round_trip(map_factory):
    grid = load_map(map_factory(np.full((3, 4), 255, dtype=np.uint8),
                               resolution=0.2, origin=[10., -2., np.pi / 2]))
    assert np.allclose(grid.pixel_to_world([0, 0]), [9.5, -1.9])
    assert np.allclose(grid.pixel_to_world([0, 2]), [9.9, -1.9])
    pixels = np.array([[0, 0], [3, 2], [0.25, 1.75]])
    assert np.allclose(grid.world_to_pixel(grid.pixel_to_world(pixels)), pixels)
    assert grid.contains(grid.pixel_to_world(pixels)).all()
    assert not grid.contains([1000, 1000])
    assert not grid.contains([np.nan, 0])


@pytest.mark.parametrize("override", [
    {"resolution": 0}, {"resolution": float("inf")}, {"resolution": "0.05"},
    {"origin": [0, 0]}, {"origin": [0, 0, float("nan")]},
    {"free_thresh": 0.8}, {"occupied_thresh": 1.1}, {"negate": 2},
    {"negate": True}, {"mode": "raw"}, {"mode": "scale"}, {"image": "missing.png"},
])
def test_invalid_metadata(map_factory, override):
    with pytest.raises(ValueError):
        load_map(map_factory(**override))


def test_bad_or_missing_yaml(tmp_path):
    for content in ("[]", "image: [", "{}"):
        path = tmp_path / "bad.yaml"
        path.write_text(content)
        with pytest.raises(ValueError):
            load_map(path)
    with pytest.raises(ValueError, match="Cannot read"):
        load_map(tmp_path / "missing.yaml")
    with pytest.raises(ValueError, match="YAML"):
        load_map(tmp_path / "image.png")
