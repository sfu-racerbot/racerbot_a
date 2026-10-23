import argparse
from dataclasses import fields
import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np

from .generator import VehicleGeometry, generate_waypoints


def waypoint_directory():
    """Find this repository by its source packages, independent of its location."""
    # look from where we ran it, then from where this script lives
    for start in (Path.cwd(), Path(__file__).resolve().parent):
        for root in (start, *start.parents):
            packages = root / "src"
            if all((packages / name / "package.xml").is_file()
                   for name in ("pure_pursuit", "waypoint_generator")):
                return packages / "pure_pursuit" / "waypoints"

            # stop here so we do not pick up a different repo
            if (root / ".git").exists():
                break
    raise ValueError("Cannot find the racerbot_a source repository; run from the racerbot_a directory")


def main(argv=None):
    cli = argparse.ArgumentParser(description="Generate a geometry-checked centerline CSV")
    cli.add_argument("--map", required=True, help="Completed ROS map YAML")
    cli.add_argument("--spacing", type=float, default=0.15, help="Spacing in metres")
    cli.add_argument("--seed", type=float, nargs=2, metavar=("X", "Y"))
    cli.add_argument("--direction", choices=("ccw", "cw"), default="ccw")
    cli.add_argument("--overwrite", action="store_true")

    # use the vehicle defaults as CLI options too
    for field in fields(VehicleGeometry):
        cli.add_argument("--" + field.name.replace("_", "-"), type=float, default=field.default,
                         help=f"{field.name}, default {field.default} (metres/radians)")

    args = cli.parse_args(argv)

    try:
        # name the CSV after the map, without changing the original track files
        name = Path(args.map).stem + "_generated"
        output = waypoint_directory() / (name + ".csv")

        # do not write through a shortcut to another file
        if output.is_symlink():
            raise ValueError("Output must not be a symbolic link")
        if output.exists() and not args.overwrite:
            raise FileExistsError("Output already exists; use --overwrite to replace it")

        vehicle = VehicleGeometry(**{f.name: getattr(args, f.name) for f in fields(VehicleGeometry)})
        data = generate_waypoints(args.map, args.spacing, args.seed, args.direction, vehicle)
        output.parent.mkdir(parents=True, exist_ok=True)

        # write a temporary file first, so a failed run keeps the old CSV
        with TemporaryDirectory(dir=output.parent) as directory:
            temporary = Path(directory) / "waypoints.csv"
            np.savetxt(temporary, data, delimiter=",", fmt="%.9f",
                       header="x_m,y_m,w_tr_right_m,w_tr_left_m\nclosure=implicit; driving_verified=false")

            if args.overwrite:
                temporary.replace(output)
            else:
                os.link(temporary, output)

    except (ValueError, OSError) as exc:
        print(f"waypoint_generator: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote {len(data)} waypoints to {output}; geometry checked, driving unverified")
    print(f"From racerbot_a, rebuild pure_pursuit and source install/setup.bash; select track:={name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
