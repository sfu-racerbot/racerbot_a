from dataclasses import replace

import numpy as np
import pytest

from waypoint_generator.generator import (WaypointPath, estimate_widths, extract_track,
    generate_centerline, process_waypoints, resample_loop, signed_area, skeleton_loop)
from waypoint_generator.generator import load_map
from waypoint_generator.generator import VehicleGeometry, validate_waypoints


def test_track_selection_and_unknown_exclusion(ring, map_factory):
    mask = extract_track(ring)
    assert np.array_equal(mask, ring.free)
    assert np.array_equal(extract_track(ring, ring.pixel_to_world([108, 70])), mask)
    with pytest.raises(ValueError, match="Seed"):
        extract_track(ring, ring.pixel_to_world([70, 70]))
    # Gray unknown space must not bridge gaps or become a second free region.
    pixels = np.where(mask, 255, 205).astype(np.uint8)
    grid = load_map(map_factory(pixels))
    assert np.array_equal(extract_track(grid), mask)


def test_ambiguous_regions_need_seed(map_factory, ring):
    pixels = np.zeros((140, 300), dtype=np.uint8)
    pixels[:, :140] = ring.free * 255
    pixels[:, 160:] = ring.free * 255
    grid = load_map(map_factory(pixels))
    with pytest.raises(ValueError, match="2 candidate"):
        extract_track(grid)
    selected = extract_track(grid, grid.pixel_to_world([108, 70]))
    assert selected[:, :140].any() and not selected[:, 160:].any()


def test_open_boundary_obstacle_and_image_edge_rejected(map_factory, ring):
    for kind in ("cut", "obstacle", "border"):
        free = ring.free.copy()
        if kind == "cut":
            free[68:73, 70:] = False
        elif kind == "obstacle":
            free[68:72, 106:110] = False
        else:
            free[68:73, 108:] = True
        with pytest.raises(ValueError):
            extract_track(load_map(map_factory((free * 255).astype(np.uint8))))


def test_loop_prunes_dead_ends_and_is_deterministic(ring):
    free = ring.free.copy()
    free[68:73, 108:132] = True
    loop = generate_centerline(free)
    assert len(np.unique(loop, axis=0)) == len(loop)
    assert np.linalg.norm(loop[0] - loop[-1]) <= np.sqrt(2)
    assert loop[:, 0].max() < 120
    assert np.array_equal(loop, generate_centerline(free))


def test_disconnected_or_empty_skeleton_rejected():
    with pytest.raises(ValueError):
        skeleton_loop(np.zeros((10, 10), dtype=bool))
    skeleton = np.zeros((30, 60), dtype=bool)
    for dx in (0, 30):
        skeleton[5, dx + 5:dx + 20] = True
        skeleton[19, dx + 5:dx + 20] = True
        skeleton[5:20, dx + 5] = True
        skeleton[5:20, dx + 19] = True
    with pytest.raises(ValueError, match="Multiple"):
        skeleton_loop(skeleton)


@pytest.mark.parametrize("direction", ["cw", "ccw"])
def test_processing_and_safe_footprint(ring, direction):
    mask = extract_track(ring)
    path = process_waypoints(generate_centerline(mask), ring, mask, direction=direction)
    distances = np.linalg.norm(np.roll(path.points, -1, axis=0) - path.points, axis=1)
    assert distances.max() <= 0.15 * 1.01
    assert distances.min() > 0.15 * 0.95
    assert (signed_area(path.points) > 0) == (direction == "ccw")
    assert np.min(path.widths_left) > 0.35
    validate_waypoints(path, ring, mask)


def test_widths_swap_when_direction_reverses(ring):
    theta = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    points = np.column_stack((0.025 + 1.8 * np.cos(theta), -0.025 + 1.8 * np.sin(theta)))
    right, left = estimate_widths(points, ring, ring.free)
    reversed_right, reversed_left = estimate_widths(points[::-1], ring, ring.free)
    assert np.allclose(right, reversed_left[::-1])
    assert np.allclose(left, reversed_right[::-1])


def test_duplicate_cleanup_and_invalid_spacing():
    points = np.array([[0, 0], [1, 0], [1, 0], [1, 1], [0, 1], [0, 0]])
    result = resample_loop(points, 0.1)
    assert len(result) == 40
    assert len(np.unique(result, axis=0)) == len(result)
    for spacing in (0, -1, np.nan, np.inf):
        with pytest.raises(ValueError):
            resample_loop(points, spacing)


def test_validator_rejects_bad_paths_and_vehicle(ring):
    path = process_waypoints(generate_centerline(ring.free), ring, ring.free)
    cases = [replace(path, points=path.points * 10),
             replace(path, points=np.full_like(path.points, np.nan)),
             replace(path, points=np.vstack((path.points[0], path.points[0], path.points[2:]))),
             replace(path, points=path.points[::4]),
             replace(path, widths_left=np.zeros(len(path.points))),
             replace(path, widths_right=np.ones(2))]
    for candidate in cases:
        with pytest.raises(ValueError):
            validate_waypoints(candidate, ring, ring.free)
    oversized = VehicleGeometry(width=2.0)
    with pytest.raises(ValueError, match="footprint"):
        validate_waypoints(path, ring, ring.free, oversized)
    with pytest.raises(ValueError):
        VehicleGeometry(safety_margin=-0.1)


def test_segment_obstacle_and_corner_clipping_are_detected(ring):
    path = process_waypoints(generate_centerline(ring.free), ring, ring.free)
    # Introduce an obstacle under a front corner without moving a waypoint out
    # of free space. This tests body coverage rather than point containment.
    track = ring.free.copy()
    p = path.points[10]
    tangent = path.points[11] - path.points[9]
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    corner = p + 0.20 * tangent + 0.15 * normal
    x, y = np.floor(ring.world_to_pixel(corner) + 0.5).astype(int)
    track[y, x] = False
    assert ring.contains(path.points, track).all()
    with pytest.raises(ValueError, match="footprint"):
        validate_waypoints(path, ring, track)


def test_self_intersection_is_a_mandatory_failure(ring):
    # A figure eight has nonadjacent segments meeting at its centre.
    t = np.linspace(0, 2 * np.pi, 80, endpoint=False)
    points = np.column_stack((np.sin(t), np.sin(2 * t)))
    path = WaypointPath(points, np.ones(80), np.ones(80), 0.15, "ccw")
    with pytest.raises(ValueError, match="self-intersections"):
        validate_waypoints(path, ring, ring.free)


def test_variable_width_curved_corridor(map_factory):
    y, x = np.indices((180, 180))
    outer = ((x - 90) / 70) ** 2 + ((y - 90) / 58) ** 2 <= 1
    inner = ((x - 90) / 42) ** 2 + ((y - 90) / 32) ** 2 < 1
    grid = load_map(map_factory(((outer & ~inner) * 255).astype(np.uint8)))
    mask = extract_track(grid)
    path = process_waypoints(generate_centerline(mask), grid, mask)
    validate_waypoints(path, grid, mask)
    assert np.ptp(path.widths_left) > 0.04


def test_obstacle_between_waypoints_is_checked(ring):
    path = process_waypoints(generate_centerline(ring.free), ring, ring.free)
    midpoint = (path.points[20] + path.points[21]) / 2
    x, y = np.floor(ring.world_to_pixel(midpoint) + 0.5).astype(int)
    mask = ring.free.copy()
    mask[y, x] = False
    assert ring.contains(path.points, mask).all()
    with pytest.raises(ValueError, match="Connecting segments"):
        validate_waypoints(path, ring, mask)
