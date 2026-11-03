"""Real solver and adapter regression tests; no ROS or mocked optimizer."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest
import yaml

spec = importlib.util.spec_from_file_location("raceline_adapter", Path(__file__).parents[1] / "optimize.py")
opt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(opt)
from waypoint_generator.generator import generate_waypoints


@pytest.fixture(autouse=True)
def isolated_waypoints(tmp_path, monkeypatch):
    directory = tmp_path / "waypoints"
    monkeypatch.setattr(opt, "WAYPOINTS", directory)
    return directory


@pytest.fixture
def ring(tmp_path):
    y, x = np.indices((180, 180))
    radius = np.hypot(x - 90, y - 90)
    pixels = np.where((radius >= 40) & (radius <= 75), 255, 0).astype(np.uint8)
    Image.fromarray(pixels).save(tmp_path / "ring.png")
    path = tmp_path / "ring.yaml"
    path.write_text(yaml.safe_dump(dict(image="ring.png", resolution=0.05,
        origin=[2.3, -1.7, 0.4], negate=0, occupied_thresh=0.65, free_thresh=0.196)))
    return path


def input_csv(path, direction="ccw"):
    data = generate_waypoints(path, direction=direction)
    csv = path.with_name(f"{direction}.csv")
    np.savetxt(csv, data, delimiter=",", header="x_m,y_m,w_tr_right_m,w_tr_left_m")
    return csv


@pytest.mark.parametrize("direction", ["ccw", "cw"])
def test_pipeline_csv_only_and_direction(ring, tmp_path, direction, monkeypatch):
    # Any accidental speed-profile execution must fail the test.
    def forbidden(*a, **kw):
        raise AssertionError("Speed calculation must not execute")
    monkeypatch.setattr(opt.tph.calc_vel_profile, "calc_vel_profile", forbidden)
    csv = input_csv(ring, direction)
    out = opt.WAYPOINTS / "ring_Optimized"
    csv_name = "ring_optimized.csv"
    original_centerline = csv.read_bytes()
    assert opt.main(["--map", str(ring), "--centerline", str(csv)]) == 0
    assert csv.read_bytes() == original_centerline
    assert not (opt.WAYPOINTS / csv_name).exists()
    points = np.loadtxt(out / csv_name, delimiter=",")
    assert not np.allclose(points[0], points[-1])
    assert points.shape[1] == 2 and np.isfinite(points).all()
    assert (opt.signed_area(points) > 0) == (direction == "ccw")
    before = (out / csv_name).read_bytes()
    # Replacing a legacy result also removes old diagnostics.
    (out / "report.json").write_text("{}")
    (out / "workflow.log").write_text("legacy log")
    assert opt.main(["--map", str(ring), "--centerline", str(csv)]) == 1
    assert opt.main(["--map", str(ring), "--centerline", str(csv),
                     "--overwrite"]) == 0
    assert {p.name for p in out.iterdir()} == {csv_name}
    csv.write_text("1,2,3,4\n")
    assert opt.main(["--map", str(ring), "--centerline", str(csv), "--overwrite"]) == 1
    assert (out / csv_name).read_bytes() == before


def test_publish_rolls_back_all_outputs(tmp_path, monkeypatch):
    staged = tmp_path / "staged"
    staged.mkdir()
    (staged / "track.csv").write_text("new")
    (staged / "report.json").write_text("new report")
    details = tmp_path / "details"
    details.mkdir()
    (details / "report.json").write_text("old report")
    rename = Path.rename
    def fail_publish_once(self, target):
        if self.name == "new" and Path(target) == details:
            raise OSError("Simulated CSV publication failure")
        return rename(self, target)
    monkeypatch.setattr(Path, "rename", fail_publish_once)
    with pytest.raises(OSError):
        opt.publish_results(staged, details)
    assert (details / "report.json").read_text() == "old report"
    assert not (details / "track.csv").exists()


@pytest.mark.parametrize("option", ["--config", "--corridor-map"])
def test_result_replacement_cannot_delete_input(ring, tmp_path, option, capsys):
    folder = opt.WAYPOINTS / "ring_Optimized"
    folder.mkdir(parents=True)
    (folder / "report.json").write_text("{}")
    protected = folder / "input.txt"
    protected.write_text("preserve this input")
    assert opt.main(["--map", str(ring), "--centerline", str(tmp_path / "unused.csv"),
                     option, str(protected), "--overwrite"]) == 1
    assert "Input files must be outside" in capsys.readouterr().err
    assert protected.read_text() == "preserve this input"


@pytest.mark.parametrize("data", [np.ones((8, 3)), np.ones((8, 4)),
    np.array([[i, i, -1, 1] for i in range(8)]), np.full((8, 4), np.nan)])
def test_invalid_csv(tmp_path, data):
    csv = tmp_path / "bad.csv"
    np.savetxt(csv, data, delimiter=",")
    with pytest.raises(ValueError):
        opt.load_track(csv)


def test_asymmetric_width_direction_reversal(ring):
    grid = opt.load_map(ring)
    theta = np.linspace(0, 2*np.pi, 90, endpoint=False)
    points = grid.pixel_to_world(np.column_stack((90 + 53*np.cos(theta), 90 + 53*np.sin(theta))))
    right, left = opt.estimate_widths(points, grid, grid.free)
    assert not np.allclose(right, left)
    rr, ll = opt.estimate_widths(points[::-1], grid, grid.free)
    np.testing.assert_allclose(rr, left[::-1])
    np.testing.assert_allclose(ll, right[::-1])


def test_solver_and_curvature_rejection(ring):
    track = opt.load_track(input_csv(ring))
    steps, smooth = opt.load_config(Path(opt.__file__).with_name("config.ini"))
    vehicle = opt.VehicleGeometry()
    geometry = opt.solve_geometry(track, steps, smooth, vehicle)
    grid = opt.load_map(ring)
    with pytest.raises(ValueError, match="curvature"):
        opt.validate_geometry(geometry, grid, grid.free, "ccw", replace(vehicle, max_steering_angle=0.001))
    narrow = track.copy()
    narrow[:, 2:] = 0.01
    with pytest.raises(RuntimeError, match="not solvable"):
        opt.solve_geometry(narrow, steps, {**smooth, "s_reg": 0}, vehicle)


def test_collision_rejected(ring):
    track = opt.load_track(input_csv(ring))
    grid = opt.load_map(ring)
    with pytest.raises(ValueError):
        opt.validate_path(track[:, :2] + [100, 0], grid, grid.free, "ccw", opt.VehicleGeometry())


def test_planning_corridor_on_open_map(ring, tmp_path):
    metadata = yaml.safe_load(ring.read_text())
    y, x = np.indices((180, 180))
    # Remove only the outer wall; the free region now reaches the image border.
    pixels = np.where(np.hypot(x - 90, y - 90) >= 40, 255, 0).astype(np.uint8)
    Image.fromarray(pixels).save(tmp_path / "open.png")
    opened = tmp_path / "open.yaml"
    metadata["image"] = "open.png"
    opened.write_text(yaml.safe_dump(metadata))
    grid = opt.load_map(opened)
    with pytest.raises(ValueError, match="candidate corridors"):
        generate_waypoints(opened)
    mask = opt.planning_mask(grid, corridor_map=ring)
    assert not (mask & ~grid.free).any()
    track = generate_waypoints(opened, corridor_map=ring)
    csv = tmp_path / "centerline.csv"
    np.savetxt(csv, track, delimiter=",")
    assert opt.main(["--map", str(opened), "--centerline", str(csv),
                     "--corridor-map", str(ring)]) == 0
    assert {p.name for p in (opt.WAYPOINTS / "open_Optimized").iterdir()} == {"open_optimized.csv"}
    # A planning mask cannot erase a wall cutting the real corridor in two.
    pixels[:90, 89:92] = 0
    Image.fromarray(pixels).save(tmp_path / "open.png")
    with pytest.raises(ValueError, match="candidate corridors"):
        opt.planning_mask(opt.load_map(opened), corridor_map=ring)
    metadata["origin"][0] += 1
    opened.write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError, match="must match"):
        opt.planning_mask(opt.load_map(opened), corridor_map=ring)


@pytest.mark.parametrize("direction", [-1, 1])
def test_crossing_bounds_only_shrink(direction):
    angle = direction * np.arange(30) * 2 * np.pi / 30
    points = np.column_stack((np.cos(angle), np.sin(angle)))
    normals = direction * points
    ref = np.column_stack((points, np.full((len(points), 2), 2.)))
    before = ref.copy()
    assert opt.tph.check_normals_crossing.check_normals_crossing(ref, normals)
    opt.trim_crossing_bounds(ref, normals, minimum_width=1.)
    np.testing.assert_array_equal(ref[:, :2], before[:, :2])
    assert (ref[:, 2:] <= before[:, 2:]).all()
    assert (ref[:, 2:].sum(axis=1) >= 1.).all()
    assert not opt.tph.check_normals_crossing.check_normals_crossing(ref, normals)


@pytest.mark.integration
def test_slam_reference_rejects_excessive_smoothing():
    grid = opt.load_map(opt.ROOT / "maps/slam_1026.yaml")
    track = generate_waypoints(grid.yaml_path)
    steps, smooth = opt.load_config(Path(opt.__file__).with_name("config.ini"))
    with pytest.raises(ValueError, match="Smoothed reference"):
        opt.prepare_reference(track, steps, {**smooth, "s_reg": 160}, grid, grid.free)


@pytest.mark.integration
@pytest.mark.parametrize("name,config", [("compTestTrack", "compTestTrack.ini"),
                                         ("slam_1026", "config.ini")])
def test_tight_turn_maps(name, config):
    grid = opt.load_map(opt.ROOT / "maps" / f"{name}.yaml")
    mask = opt.extract_track(grid)
    track = generate_waypoints(grid.yaml_path)
    steps, smooth = opt.load_config(Path(opt.__file__).with_name(config))
    geometry = opt.solve_geometry(track, steps, smooth, opt.VehicleGeometry(), grid, mask)
    assert geometry["trimmed_bounds"] > 0
    checks = opt.validate_geometry(geometry, grid, mask, "ccw", opt.VehicleGeometry())
    assert checks["max_abs_curvature"] <= checks["curvature_limit"]


@pytest.mark.integration
def test_repository_map(tmp_path):
    path = opt.ROOT / "maps/levine_blocked.yaml"
    csv = tmp_path / "centerline.csv"
    np.savetxt(csv, generate_waypoints(path), delimiter=",")
    assert opt.main(["--map", str(path), "--centerline", str(csv),
                     "--config",
                     str(Path(opt.__file__).with_name("levine_blocked.ini"))]) == 0
    assert (opt.WAYPOINTS / "levine_blocked_Optimized/levine_blocked_optimized.csv").is_file()
    assert len(list((opt.WAYPOINTS / "levine_blocked_Optimized").iterdir())) == 1
