import csv

import numpy as np
import pytest

from waypoint_generator.cli import main
from waypoint_generator import cli
from waypoint_generator.generator import generate_waypoints, load_map, signed_area


def test_full_pipeline_csv_and_overwrite_protection(map_factory, waypoint_dir, capsys):
    source = map_factory()
    output = waypoint_dir / f"{source.stem}_generated.csv"
    args = ["--map", str(source)]
    assert main(args) == 0
    assert f"track:={source.stem}_generated" in capsys.readouterr().out
    rows = list(csv.reader(line for line in output.read_text().splitlines() if not line.startswith("#")))
    assert len(rows) > 8
    assert all(len(row) == 4 and np.isfinite(list(map(float, row))).all() for row in rows)
    assert np.allclose(np.loadtxt(output, delimiter=","), generate_waypoints(source))
    assert list(output.parent.iterdir()) == [output]  # CSV only
    original = output.read_bytes()
    assert main(args) == 1
    assert output.read_bytes() == original
    assert main(args + ["--overwrite", "--width", "4"]) == 1
    assert output.read_bytes() == original
    assert main(args + ["--overwrite", "--direction", "cw"]) == 0
    assert signed_area(np.loadtxt(output, delimiter=",")[:, :2]) < 0


@pytest.mark.parametrize("bad_args", [["--spacing", "nan"], ["--spacing", "0"],
                                      ["--seed", "nan", "1"], ["--width", "-1"]])
def test_failure_never_exports_csv(map_factory, waypoint_dir, bad_args, capsys):
    source = map_factory()
    output = waypoint_dir / f"{source.stem}_generated.csv"
    assert main(["--map", str(source)] + bad_args) == 1
    assert not output.exists()
    assert "waypoint_generator:" in capsys.readouterr().err


def test_missing_map_reports_error(tmp_path, waypoint_dir, capsys):
    output = waypoint_dir / "missing_generated.csv"
    assert main(["--map", str(tmp_path / "missing.yaml")]) == 1
    assert not output.exists()
    assert "Cannot read map YAML" in capsys.readouterr().err


def test_saved_slam_format_with_rotated_origin(map_factory, waypoint_dir):
    source = map_factory(suffix="pgm", origin=[2.0, -4.0, 0.61], mode="trinary")
    output = waypoint_dir / f"{source.stem}_generated.csv"
    assert main(["--map", str(source), "--direction", "cw"]) == 0
    points = np.loadtxt(output, delimiter=",")[:, :2]
    assert load_map(source).contains(points).all()
    assert signed_area(points) < 0


@pytest.mark.parametrize("suffix", ["yaml", "png"])
def test_output_symlink_cannot_overwrite_map(map_factory, waypoint_dir, suffix):
    source = map_factory()
    target = source.with_suffix("." + suffix)
    waypoint_dir.mkdir(parents=True)
    output = waypoint_dir / f"{source.stem}_generated.csv"
    output.symlink_to(target)
    original = target.read_bytes()
    assert main(["--map", str(source), "--overwrite"]) == 1
    assert target.read_bytes() == original


def test_output_argument_is_removed(map_factory):
    with pytest.raises(SystemExit) as exc:
        main(["--map", str(map_factory()), "--output", "unused.csv"])
    assert exc.value.code == 2


@pytest.mark.parametrize("installed", [False, True])
def test_find_standalone_repository(tmp_path, monkeypatch, installed):
    # Deliberately use a checkout name unrelated to racerbot_a or any workspace.
    repo = tmp_path / "custom_checkout"
    for name in ("pure_pursuit", "waypoint_generator"):
        package = repo / "src" / name
        package.mkdir(parents=True)
        (package / "package.xml").write_text("<package/>")
    (repo / ".git").mkdir()
    if installed:
        module = repo / "install/waypoint_generator/lib/python3.10/site-packages/waypoint_generator/cli.py"
        share = repo / "install/pure_pursuit/share/pure_pursuit"
        share.mkdir(parents=True)
        (share / "package.xml").write_text("<package/>")
        monkeypatch.chdir(share)
    else:
        module = repo / "src/waypoint_generator/waypoint_generator/cli.py"
        monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(cli, "__file__", str(module))
    assert cli.waypoint_directory() == repo / "src/pure_pursuit/waypoints"


def test_current_repository_takes_precedence(tmp_path, monkeypatch):
    for name in ("pure_pursuit", "waypoint_generator"):
        package = tmp_path / "src" / name
        package.mkdir(parents=True)
        (package / "package.xml").write_text("<package/>")
    monkeypatch.chdir(tmp_path / "src/pure_pursuit")
    monkeypatch.setattr(cli, "__file__", str(tmp_path / "unrelated_install/cli.py"))
    assert cli.waypoint_directory() == tmp_path / "src/pure_pursuit/waypoints"


def test_does_not_search_outside_repository(tmp_path, monkeypatch):
    for name in ("pure_pursuit", "waypoint_generator"):
        package = tmp_path / "src" / name
        package.mkdir(parents=True)
        (package / "package.xml").write_text("<package/>")
    child = tmp_path / "separate_repo"
    child.mkdir()
    (child / ".git").write_text("gitdir: elsewhere")
    monkeypatch.chdir(child)
    monkeypatch.setattr(cli, "__file__", str(child / "installed/cli.py"))
    with pytest.raises(ValueError, match="source repository"):
        cli.waypoint_directory()


def test_missing_source_package_reports_error(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(cli, "__file__", str(tmp_path / "installed/cli.py"))
    assert main(["--map", "track.yaml"]) == 1
    assert "run from the racerbot_a directory" in capsys.readouterr().err
    assert not list(tmp_path.rglob("*.csv"))
