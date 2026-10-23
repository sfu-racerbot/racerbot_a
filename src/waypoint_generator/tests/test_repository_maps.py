"""Integration checks against the repository maps and existing reference CSVs."""

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree

from waypoint_generator.cli import main

SOURCE = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("map_name,reference", [("levine_blocked", "levine"),
                                               ("Spielberg", "spielberg")])
def test_repository_map_pipeline_and_alignment(waypoint_dir, map_name, reference):
    output = waypoint_dir / f"{map_name}_generated.csv"
    assert main(["--map", str(SOURCE / "maps" / f"{map_name}.yaml")]) == 0
    generated = np.loadtxt(output, delimiter=",")[:, :2]
    existing = np.loadtxt(SOURCE / "pure_pursuit/waypoints" / f"{reference}.csv", delimiter=",")[:, :2]
    # References establish metre-coordinate alignment, not exact ground truth.
    assert np.quantile(cKDTree(existing).query(generated)[0], 0.95) < 0.5


@pytest.mark.parametrize("name", ["levine", "levine_obs"])
def test_open_or_obstructed_repository_map_is_rejected(waypoint_dir, name, capsys):
    output = waypoint_dir / f"{name}_generated.csv"
    assert main(["--map", str(SOURCE / "maps" / f"{name}.yaml")]) == 1
    assert not output.exists()
    assert "candidate corridors" in capsys.readouterr().err
