from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from waypoint_generator.generator import load_map


@pytest.fixture
def map_factory(tmp_path):
    def create(pixels=None, suffix="png", **overrides):
        if pixels is None:
            y, x = np.indices((140, 140))
            radius = np.hypot(x - 70, y - 70)
            pixels = np.where((radius >= 28) & (radius <= 48), 255, 0).astype(np.uint8)
        index = len(list(tmp_path.glob("*.yaml")))
        image = tmp_path / f"map{index}.{suffix}"
        Image.fromarray(pixels).save(image)
        metadata = {"image": image.name, "resolution": 0.05, "origin": [-3.5, -3.5, 0.0],
                    "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}
        metadata.update(overrides)
        path = image.with_suffix(".yaml")
        path.write_text(yaml.safe_dump(metadata))
        return path
    return create


@pytest.fixture
def ring(map_factory):
    return load_map(map_factory())


@pytest.fixture
def waypoint_dir(tmp_path, monkeypatch):
    from waypoint_generator import cli
    directory = tmp_path / "pure_pursuit" / "waypoints"
    monkeypatch.setattr(cli, "waypoint_directory", lambda: directory)
    return directory
