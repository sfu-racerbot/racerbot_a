"""Offline racerbot adapter for Waterloo's unchanged minimum-curvature pipeline."""
import argparse
import configparser
from dataclasses import asdict, fields
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import warnings

ROOT = Path(__file__).resolve().parents[2]
WAYPOINTS = ROOT / "src/pure_pursuit_a/waypoints"
sys.path.insert(0, str(ROOT / "third_party/raceline_optimization"))
sys.path.insert(0, str(ROOT / "src/waypoint_generator"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import numpy as np
import trajectory_planning_helpers as tph
from helper_funcs_glob.src.result_plots import result_plots
from waypoint_generator.generator import (
    VehicleGeometry, WaypointPath, estimate_widths, extract_track, load_map, planning_mask,
    signed_area, validate_waypoints,
)


def load_track(path):
    track = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
    if (track.ndim != 2 or track.shape[1] != 4 or len(track) < 8
            or not np.isfinite(track).all() or (track[:, 2:] <= 0).any()):
        raise ValueError("Centerline needs at least eight finite x,y,right-width,left-width rows; widths must be positive")
    if len(np.unique(np.round(track[:, :2], 8), axis=0)) != len(track):
        raise ValueError("Duplicate centerline points; use implicit closure without a repeated endpoint")
    return track


def load_config(path):
    parser = configparser.ConfigParser()
    if not parser.read(path):
        raise ValueError(f"Cannot read config: {path}")
    section = parser["GENERAL_OPTIONS"]
    steps = json.loads(section["stepsize_opts"])
    smooth = json.loads(section["reg_smooth_opts"])
    for key in ("stepsize_prep", "stepsize_reg", "stepsize_interp_after_opt"):
        if not isinstance(steps[key], (float, int)) or not math.isfinite(steps[key]) or steps[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    if smooth["k_reg"] != 3 or not math.isfinite(smooth["s_reg"]) or smooth["s_reg"] < 0:
        raise ValueError("Use cubic k_reg=3 and a finite nonnegative s_reg")
    return steps, smooth


def trim_crossing_bounds(ref, normals, minimum_width=0., horizon=10):
    """Shrink lateral bounds before intersecting neighbouring normal segments.

    This restricts the search corridor; it never opens occupied space. Keep a
    2% gap from each intersection and retain Waterloo's final crossing check.
    Trimming either intersecting ray suffices; prefer the least restrictive
    choice that still accommodates the vehicle's conservative width.
    """
    widths = ref[:, 2:]
    for i in range(len(ref)):
        for offset in range(1, min(horizon, (len(ref) - 1) // 2) + 1):
            j = (i + offset) % len(ref)
            if np.isclose(np.cross(normals[i], normals[j]), 0.):
                continue
            distances = np.linalg.solve(np.column_stack((normals[i], -normals[j])),
                                        ref[j, :2] - ref[i, :2])
            if (-ref[i, 3] <= distances[0] <= ref[i, 2]
                    and -ref[j, 3] <= distances[1] <= ref[j, 2]):
                candidates = []
                for index, distance in zip((i, j), distances):
                    side = 0 if distance >= 0 else 1
                    bound = 0.98 * abs(distance)
                    remaining = widths[index, 1 - side] + bound
                    reduction = widths[index, side] - bound
                    candidates.append((remaining >= minimum_width, -reduction, index, side, bound))
                _, _, index, side, bound = max(candidates)
                widths[index, side] = bound


def prepare_reference(track, steps, smooth, grid=None, mask=None, minimum_width=0.):
    # Waterloo prep_track's numerical functions, with occupancy correction
    # BEFORE its normal-crossing check. Upstream source remains unchanged.
    ref = tph.spline_approximation.spline_approximation(
        track=track, k_reg=smooth["k_reg"], s_reg=smooth["s_reg"],
        stepsize_prep=steps["stepsize_prep"], stepsize_reg=steps["stepsize_reg"], debug=False)
    _, _, matrix, normals = tph.calc_splines.calc_splines(
        path=np.vstack((ref[:, :2], ref[0, :2])))
    if not np.isfinite(ref).all() or (ref[:, 2:] <= 0).any():
        raise ValueError("Smoothed reference has invalid lateral widths; reduce smoothing")
    if grid is not None:
        outside = np.flatnonzero(~grid.contains(ref[:, :2], mask))
        if len(outside):
            raise ValueError(f"Smoothed reference leaves the free corridor at {len(outside)} points "
                             f"(first: {ref[outside[0], :2].tolist()}); reduce smoothing")
        # Smoothing can change reference positions and normals. Never let its
        # interpolated widths claim space outside the original occupancy map.
        distances = np.arange(grid.resolution / 4,
                              np.hypot(grid.width, grid.height) * grid.resolution,
                              grid.resolution / 4)
        for i, (point, normal) in enumerate(zip(ref[:, :2], normals)):
            for column, sign in ((2, 1), (3, -1)):
                available = grid.contains(point + sign * distances[:, None] * normal, mask)
                width = np.flatnonzero(~available)[0] * grid.resolution / 4
                ref[i, column] = min(ref[i, column], width)
    before = ref[:, 2:].copy()
    trim_crossing_bounds(ref, normals, minimum_width)
    horizon = min(10, (len(ref) - 1) // 2)
    if tph.check_normals_crossing.check_normals_crossing(ref, normals, horizon=horizon):
        raise ValueError("Reference normal bounds still intersect after corridor restriction")
    return ref, normals, matrix, int(np.count_nonzero(ref[:, 2:] < before))


def solve_geometry(track, steps, smooth, vehicle, grid=None, mask=None):
    # Adapted from Waterloo main_globaltraj_f110.py's mincurv branch. No speed stage.
    # A circumscribed disk covers the whole rectangle at any orientation,
    # including the reference offset and safety margin. Add raster padding.
    radius = math.hypot(vehicle.length / 2 + abs(vehicle.reference_offset_x) + vehicle.safety_margin,
                        vehicle.width / 2 + vehicle.safety_margin)
    optimization_width = 2 * radius + (grid.resolution if grid is not None else 0)
    ref, normals, matrix, trimmed = prepare_reference(
        track, steps, smooth, grid, mask, optimization_width)
    narrow = np.flatnonzero(ref[:, 2:].sum(axis=1) < optimization_width)
    if len(narrow):
        i = narrow[0]
        raise RuntimeError(f"Reference corridor not solvable at {ref[i, :2].tolist()}: "
                           f"{ref[i, 2:].sum():.3f} m available, {optimization_width:.3f} m required; "
                           "refine the reference/smoothing without reducing vehicle clearance")
    alpha, linearization_error = tph.opt_min_curv.opt_min_curv(
        reftrack=ref, normvectors=normals, A=matrix,
        kappa_bound=math.tan(vehicle.max_steering_angle) / vehicle.wheelbase,
        w_veh=optimization_width,
        print_debug=False, plot_debug=False)
    result = tph.create_raceline.create_raceline(
        refline=ref[:, :2], normvectors=normals, alpha=alpha,
        stepsize_interp=steps["stepsize_interp_after_opt"])
    points, _, cx, cy, indices, t, s, lengths, ds = result
    heading, curvature = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=cx, coeffs_y=cy, ind_spls=indices, t_spls=t)
    return dict(points=points, heading=heading, curvature=curvature, s=s,
                segment_lengths=ds, spline_lengths=lengths, coeffs_x=cx, coeffs_y=cy,
                reference=ref, alpha=alpha, linearization_error=np.asarray(linearization_error),
                trimmed_bounds=np.asarray(trimmed),
                optimization_width=np.asarray(optimization_width))


def validate_path(points, grid, mask, direction, vehicle):
    spacing = float(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1).mean())
    right, left = estimate_widths(points, grid, mask)
    validate_waypoints(WaypointPath(points, right, left, spacing, direction), grid, mask, vehicle)


def validate_geometry(geometry, grid, mask, direction, vehicle):
    validate_path(geometry["points"], grid, mask, direction, vehicle)
    # Sample the spline at resolution/4 approximate arc intervals. Existing
    # validator then checks connecting segments and swept rectangles with padding.
    lengths = geometry["spline_lengths"]
    dense, indices, t, _ = tph.interp_splines.interp_splines(
        coeffs_x=geometry["coeffs_x"], coeffs_y=geometry["coeffs_y"],
        spline_lengths=lengths, incl_last_point=False, stepsize_approx=grid.resolution / 4)
    _, curvature = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=geometry["coeffs_x"], coeffs_y=geometry["coeffs_y"], ind_spls=indices, t_spls=t)
    limit = math.tan(vehicle.max_steering_angle) / vehicle.wheelbase
    if not np.isfinite(curvature).all() or np.max(np.abs(curvature)) > limit + 1e-6:
        raise ValueError("Optimized spline exceeds the configured steering curvature limit")
    validate_path(dense, grid, mask, direction, vehicle)
    return {"max_abs_curvature": float(np.max(np.abs(curvature))), "curvature_limit": limit,
            "dense_validation_points": len(dense)}


def save_waterloo_plots(directory, track, geometry, vehicle, grid):
    """Save upstream's raceline and curvature figures without a speed stage."""
    reference = geometry["reference"]
    _, _, _, normals = tph.calc_splines.calc_splines(
        path=np.vstack((reference[:, :2], reference[0, :2])))
    _, _, _, imported_normals = tph.calc_splines.calc_splines(
        path=np.vstack((track[:, :2], track[0, :2])))
    # Upstream accepts seven trajectory columns. The two unused speed columns
    # are placeholders; both velocity plotting branches remain disabled.
    trajectory = np.column_stack((geometry["s"], geometry["points"],
                                  geometry["heading"], geometry["curvature"],
                                  np.zeros((len(geometry["points"]), 2))))
    existing_figures = set(plt.get_fignums())
    try:
        with warnings.catch_warnings():
            # Upstream calls show(); the Agg backend saves figures instead.
            warnings.filterwarnings("ignore", message="FigureCanvasAgg is non-interactive.*",
                                    category=UserWarning)
            result_plots(
                plot_opts=dict(raceline=True, raceline_curv=True, imported_bounds=False,
                               racetraj_vel_3d=False, spline_normals=False),
                width_veh_opt=float(geometry["optimization_width"]), width_veh_real=vehicle.width,
                refline=reference[:, :2],
                bound1_imp=track[:, :2] + imported_normals * track[:, 2, None],
                bound2_imp=track[:, :2] - imported_normals * track[:, 3, None],
                bound1_interp=reference[:, :2] + normals * reference[:, 2, None],
                bound2_interp=reference[:, :2] - normals * reference[:, 3, None],
                trajectory=trajectory)
        figures = sorted(set(plt.get_fignums()) - existing_figures)
        if len(figures) != 2:
            raise RuntimeError("Expected Waterloo raceline and curvature figures")
        # Keep upstream's plotting function, then present its two path lines on
        # the real occupancy map instead of its vehicle/boundary debug layers.
        overlay = plt.figure(figures[0])
        ax = overlay.axes[0]
        centerline, raceline = ax.lines[0], ax.lines[-1]
        for line in ax.lines:
            line.set_visible(line in (centerline, raceline))
        centerline.set_data(*np.vstack((track[:, :2], track[0, :2])).T)
        centerline.set(color="#2563eb", linestyle="--", linewidth=1.3,
                       label="Generated centerline", zorder=3)
        raceline.set_data(*np.vstack((geometry["points"], geometry["points"][0])).T)
        raceline.set(color="#dc2626", linewidth=1.7, label="Optimized raceline", zorder=4)
        for patch in tuple(ax.patches):
            patch.remove()  # Upstream's fixed two-metre arrow dominates small tracks.
        transform = (Affine2D().rotate(grid.origin[2]).translate(*grid.origin[:2])
                     + ax.transData)
        ax.imshow(np.where(grid.cells < 0, 50, grid.cells), cmap="gray_r", vmin=0, vmax=100,
                  origin="upper", extent=(0, grid.width * grid.resolution,
                                          0, grid.height * grid.resolution),
                  transform=transform, zorder=0, interpolation="nearest")
        bounds = np.vstack((track[:, :2] + imported_normals * track[:, 2, None],
                            track[:, :2] - imported_normals * track[:, 3, None],
                            geometry["points"]))
        ax.set_xlim(bounds[:, 0].min() - .5, bounds[:, 0].max() + .5)
        ax.set_ylim(bounds[:, 1].min() - .5, bounds[:, 1].max() + .5)
        start = geometry["points"][0]
        ax.plot(*start, "o", color="#111827", markersize=4, zorder=5)
        ax.annotate("Start", start, xytext=(6, 6), textcoords="offset points", fontsize=8)
        ax.set(title="Centerline and optimized raceline", xlabel="x [m]", ylabel="y [m]")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(False)
        ax.legend(loc="best")
        overlay.set_size_inches(10, 6)
        curvature_ax = plt.figure(figures[1]).axes[0]
        curvature_ax.lines[0].set_color("#dc2626")
        curvature_ax.set_title("Optimized raceline curvature")
        for number, name in zip(figures, ("overlay.png", "curvature.png")):
            plt.figure(number).savefig(directory / name, dpi=160, bbox_inches="tight")
    finally:
        for number in set(plt.get_fignums()) - existing_figures:
            plt.close(number)


def output_locations(args):
    name = Path(args.map).stem + "_optimized"
    details = WAYPOINTS / (Path(args.map).stem + "_Optimized")
    controller_csv = details / (name + ".csv")
    if details.is_symlink():
        raise ValueError(f"Output must not be a symlink: {details}")
    if details.exists() and not args.overwrite:
        raise ValueError(f"Output already exists: {details}; use --overwrite to replace optimized results")
    if details.exists() and (not details.is_dir() or not (
            controller_csv.is_file() or (details / "report.json").is_file())):
        raise ValueError(f"Refusing to overwrite a directory without an optimized CSV or legacy raceline report: {details}")
    sources = [args.centerline, args.map, args.config]
    if getattr(args, "corridor_map", None):
        sources.append(args.corridor_map)
    for source in sources:
        if Path(source).resolve().is_relative_to(details.resolve()):
            raise ValueError("Input files must be outside the generated result folder")
    return controller_csv.name, controller_csv, details


def publish_results(staged, destination):
    """Replace one complete result folder, restoring it if publication fails."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".raceline-", dir=destination.parent) as temporary:
        new = Path(temporary) / "new"
        backup = Path(temporary) / "previous"
        shutil.copytree(staged, new)
        existed = destination.exists()
        if existed:
            destination.rename(backup)
        try:
            new.rename(destination)
        except OSError:
            if existed:
                backup.rename(destination)
            raise


def run(args):
    csv_name, controller_csv, details = output_locations(args)
    vehicle = VehicleGeometry(**{f.name: getattr(args, f.name) for f in fields(VehicleGeometry)})
    steps, smooth = load_config(args.config)
    track = load_track(args.centerline)
    grid = load_map(args.map)
    corridor_map = getattr(args, "corridor_map", None)
    mask = planning_mask(grid, args.seed, corridor_map)
    direction = "ccw" if signed_area(track[:, :2]) > 0 else "cw"
    validate_path(track[:, :2], grid, mask, direction, vehicle)
    geometry = solve_geometry(track, steps, smooth, vehicle, grid, mask)
    checks = validate_geometry(geometry, grid, mask, direction, vehicle)
    def fingerprint(path):
        return {"path": str(Path(path).resolve()), "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest()}
    report = dict(schema_version=1, geometry_validated=True, driving_verified=False,
                  velocity_profile=False, direction=direction, coordinate_frame=args.frame,
                  vehicle=asdict(vehicle), stepsize_opts=steps, reg_smooth_opts=smooth,
                  map=fingerprint(args.map), image=fingerprint(grid.image_path),
                  centerline=fingerprint(args.centerline), raceline_file=csv_name,
                  controller_csv=str(controller_csv), controller_track=controller_csv.relative_to(WAYPOINTS).with_suffix("").as_posix(),
                  details_directory=str(details), validation=checks,
                  point_count=len(geometry["points"]), lap_length_m=float(geometry["spline_lengths"].sum()),
                  linearization_error=float(geometry["linearization_error"]),
                  optimization_width_m=float(geometry["optimization_width"]),
                  trimmed_normal_bounds=int(geometry["trimmed_bounds"]),
                  upstream=json.loads((ROOT / "third_party/UPSTREAM.json").read_text()))
    if corridor_map:
        report["planning_corridor"] = dict(map=fingerprint(corridor_map),
                                          image=fingerprint(load_map(corridor_map).image_path))
    with tempfile.TemporaryDirectory(prefix="raceline-build-") as temporary:
        staged = Path(temporary) / "result"
        staged.mkdir()
        np.savetxt(staged / csv_name, geometry["points"], delimiter=",", fmt="%.9f",
                   header="x_m,y_m\nclosure=implicit; driving_verified=false")
        save_waterloo_plots(staged, track, geometry, vehicle, grid)
        publish_results(staged, details)
    print(f"Validated waypoints written to {controller_csv}; driving unverified")
    print(f"Waterloo raceline and curvature plots: {details}")
    print(f"Rebuild pure_pursuit_a and select track:={report['controller_track']}")
    return report


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--centerline", required=True)
    cli.add_argument("--map", required=True)
    cli.add_argument("--corridor-map", help="Aligned closed planning corridor, also used when generating the centerline")
    cli.add_argument("--config", default=str(Path(__file__).with_name("config.ini")))
    cli.add_argument("--frame", default="map", help="Coordinate frame label; does not transform coordinates")
    cli.add_argument("--seed", type=float, nargs=2)
    cli.add_argument("--overwrite", action="store_true")
    for field in fields(VehicleGeometry):
        cli.add_argument("--" + field.name.replace("_", "-"), type=float, default=field.default)
    args = cli.parse_args(argv)
    try:
        run(args)
    except (ValueError, OSError, RuntimeError, KeyError, configparser.Error, np.linalg.LinAlgError) as exc:
        print(f"raceline_optimizer: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
