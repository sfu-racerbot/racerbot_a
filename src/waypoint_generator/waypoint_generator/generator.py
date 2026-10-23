from dataclasses import asdict, dataclass
from pathlib import Path
import math
import warnings

import numpy as np
from PIL import Image
import yaml
from scipy import ndimage
from skimage.morphology import medial_axis, skeletonize
from shapely.geometry import LineString, Polygon, box


def finite_number(value, name):
    """Reject strings, booleans, NaN and infinity in numeric metadata."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


@dataclass(frozen=True)
class OccupancyMap:
    """Image-order cells: -1 unknown, 0 free, 100 occupied; origin is ROS x,y,yaw."""

    cells: np.ndarray
    resolution: float
    origin: tuple
    yaml_path: Path
    image_path: Path
    occupied_thresh: float
    free_thresh: float
    negate: int

    @property
    def free(self):
        return self.cells == 0

    @property
    def height(self):
        return self.cells.shape[0]

    @property
    def width(self):
        return self.cells.shape[1]

    def pixel_to_world(self, points):
        """Convert (..., 2) image (column,row) pixel centres to world metres."""
        p = np.asarray(points, dtype=float)
        # use pixel centres and flip image rows into map coordinates
        local = np.stack(((p[..., 0] + 0.5) * self.resolution,
                          (self.height - p[..., 1] - 0.5) * self.resolution), axis=-1)

        # rotate and shift using the map origin
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return local @ np.array([[c, s], [-s, c]]) + self.origin[:2]

    def world_to_pixel(self, points):
        """Inverse conversion; returned indices are continuous, not rounded."""
        p = np.asarray(points, dtype=float) - self.origin[:2]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        local = p @ np.array([[c, -s], [s, c]]) / self.resolution
        return np.stack((local[..., 0] - 0.5,
                         self.height - local[..., 1] - 0.5), axis=-1)

    def contains(self, points, mask=None):
        """Test containing cells; outside the raster is always unavailable."""
        p = self.world_to_pixel(points)
        valid = np.isfinite(p).all(axis=-1) & (p >= -0.5).all(axis=-1)
        valid &= (p < [self.width - 0.5, self.height - 0.5]).all(axis=-1)

        # clip only for safe indexing; points outside still return false
        clipped = np.clip(np.where(np.isfinite(p), p, 0), [0, 0], [self.width - 1, self.height - 1])
        indices = np.floor(clipped + 0.5).astype(int)
        grid = self.free if mask is None else mask
        return valid & grid[indices[..., 1], indices[..., 0]]


def load_map(path):
    """Read YAML + 8-bit PNG/PGM. Unsupported scale/raw modes fail explicitly."""
    path = Path(path).expanduser().resolve()
    if path.suffix.lower() not in (".yaml", ".yml"):
        raise ValueError("Provide a map YAML file referencing a PNG or PGM image")
    try:
        metadata = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot read map YAML {path}: {exc}") from exc
    if not isinstance(metadata, dict):
        raise ValueError("Map YAML must contain a metadata mapping")

    # check the map settings before trying to use the image
    required = {"image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"}
    missing = required - metadata.keys()
    if missing:
        raise ValueError(f"Missing map metadata: {', '.join(sorted(missing))}")
    if metadata.get("mode", "trinary") != "trinary":
        raise ValueError("Only ROS trinary map mode is supported (not scale/raw)")

    resolution = finite_number(metadata["resolution"], "resolution")
    if resolution <= 0:
        raise ValueError("resolution must be positive")

    origin = metadata["origin"]
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError("origin must contain [x, y, yaw]")
    origin = tuple(finite_number(v, "origin") for v in origin)

    occupied = finite_number(metadata["occupied_thresh"], "occupied_thresh")
    free = finite_number(metadata["free_thresh"], "free_thresh")
    if not 0 <= free < occupied <= 1:
        raise ValueError("Thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1")
    negate = metadata["negate"]
    if type(negate) is not int or negate not in (0, 1):
        raise ValueError("negate must be 0 or 1")
    if not isinstance(metadata["image"], str) or not metadata["image"].strip():
        raise ValueError("image must be a nonempty file path")

    image_path = (path.parent / metadata["image"]).expanduser().resolve()
    if image_path.suffix.lower() not in (".png", ".pgm"):
        raise ValueError("Only PNG and PGM occupancy images are supported")
    try:
        with Image.open(image_path) as image:
            if image.mode not in ("1", "L", "LA", "P", "RGB", "RGBA"):
                raise ValueError(f"Unsupported image mode {image.mode}; use an 8-bit map")
            rgba = np.asarray(image.convert("RGBA"), dtype=np.uint8)
    except OSError as exc:
        raise ValueError(f"Cannot read map image {image_path}: {exc}") from exc

    # brightness becomes occupancy; values between thresholds stay unknown
    shade = rgba[..., :3].mean(axis=-1) / 255.0
    probability = shade if negate else 1.0 - shade
    cells = np.full(probability.shape, -1, dtype=np.int8)
    cells[probability < free] = 0
    cells[probability > occupied] = 100

    # transparent pixels are unknown too
    cells[rgba[..., 3] != 255] = -1
    return OccupancyMap(cells, resolution, origin, path, image_path, occupied, free, negate)


@dataclass(frozen=True)
class WaypointPath:
    points: np.ndarray
    widths_right: np.ndarray
    widths_left: np.ndarray
    spacing: float
    direction: str


def extract_track(grid, seed_world=None):
    # label each separate free region, then use the seed if one was given
    labels, count = ndimage.label(grid.free)
    if seed_world is not None:
        seed = np.asarray(seed_world, dtype=float)
        if seed.shape != (2,) or not np.isfinite(seed).all() or not grid.contains(seed):
            raise ValueError("Seed must be a finite world-coordinate point in known free space")
        x, y = np.floor(grid.world_to_pixel(seed) + 0.5).astype(int)
        choices = [labels[y, x]]
    else:
        choices = range(1, count + 1)

    candidates = []
    for label in choices:
        mask = labels == label
        border = mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any()
        holes = ndimage.binary_fill_holes(mask) & ~mask
        # a closed track should surround one island and avoid the image edge
        if not border and ndimage.label(holes)[1] == 1:
            candidates.append(mask)

    if len(candidates) != 1:
        raise ValueError(f"Found {len(candidates)} candidate corridors; use --seed for ambiguity, "
                         "or check for incomplete boundaries, obstacles and multiple routes")
    return candidates[0]


def skeleton_loop(skeleton):
    # treat skeleton pixels as a graph so we can walk around the track
    vertices = set(map(tuple, np.argwhere(skeleton)))
    graph = {p: set() for p in vertices}
    for y, x in vertices:
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            other = (y + dy, x + dx)
            # skip diagonals that would add tiny triangles
            if other not in vertices or (dx and dy and (
                    (y + dy, x) in vertices or (y, x + dx) in vertices)):
                continue
            graph[y, x].add(other)
            graph[other].add((y, x))

    # trim dead ends until only the loop should be left
    leaves = [p for p in graph if len(graph[p]) < 2]
    while leaves:
        for p in leaves:
            for neighbor in graph.pop(p):
                graph[neighbor].discard(p)
        leaves = [p for p in graph if len(graph[p]) < 2]

    if len(graph) < 8 or any(len(edges) != 2 for edges in graph.values()):
        raise ValueError("Centerline has no unique closed loop, or remains branching")

    # each remaining pixel needs two neighbours to form one simple loop
    start = min(graph)
    ordered, previous, current = [start], start, min(graph[start])
    while current != start:
        ordered.append(current)
        previous, current = current, next(p for p in graph[current] if p != previous)
    if len(ordered) != len(graph):
        raise ValueError("Multiple disconnected centerline loops remain")
    return np.asarray(ordered, dtype=float)[:, ::-1]  # column,row


def generate_centerline(track_mask):
    mask = np.asarray(track_mask, dtype=bool)
    if mask.ndim != 2 or not mask.any():
        raise ValueError("Track mask must be a nonempty two-dimensional image")

    # medial axis gives a thin path roughly halfway between the boundaries
    skeleton = medial_axis(mask, rng=0)
    # clean up tiny skeleton holes, without changing the actual map
    holes, _ = ndimage.label(ndimage.binary_fill_holes(skeleton) & ~skeleton)
    skeleton |= (holes > 0) & (np.bincount(holes.ravel())[holes] <= 3)
    return skeleton_loop(skeletonize(skeleton))


def signed_area(points):
    following = np.roll(points, -1, axis=0)
    return float(np.sum(points[:, 0] * following[:, 1] - following[:, 0] * points[:, 1]) / 2)


def headings(points):
    tangent = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    return np.arctan2(tangent[:, 1], tangent[:, 0])


def resample_loop(points, spacing):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError("Centerline must contain at least three finite 2D points")
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("Waypoint spacing must be finite and positive")

    # remove repeated neighbours and an extra copy of the starting point
    points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-8]]
    if np.linalg.norm(points[0] - points[-1]) <= 1e-8:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("Centerline degenerates after removing duplicates")

    # sample by distance around the loop, rather than by pixel index
    closed = np.vstack((points, points[0]))
    cumulative = np.r_[0., np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    count = max(8, int(np.ceil(cumulative[-1] / spacing)))
    if count > 100000:
        raise ValueError("Requested spacing would exceed 100000 waypoints")
    distances = np.arange(count) * cumulative[-1] / count
    return np.column_stack([np.interp(distances, cumulative, closed[:, axis]) for axis in (0, 1)])


def estimate_widths(points, grid, mask):
    """Sample each normal ray; widths stop before the first unavailable cell."""
    yaw = headings(points)
    normals = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    step = grid.resolution / 4
    distances = np.arange(step, np.hypot(grid.width, grid.height) * grid.resolution + step, step)
    widths = []

    # step out sideways until we hit a cell outside the track
    for sign in (-1, 1):  # right, left
        side = []
        for point, normal in zip(points, normals):
            free = grid.contains(point + sign * distances[:, None] * normal, mask)
            side.append(np.flatnonzero(~free)[0] * step)
        widths.append(np.asarray(side))
    return widths


def _segments_inside(points, grid, mask):
    for a, b in zip(points, np.roll(points, -1, axis=0)):
        count = max(2, int(np.ceil(np.linalg.norm(b - a) / (grid.resolution / 4))) + 1)
        if not grid.contains(np.linspace(a, b, count), mask).all():
            return False
    return True


def process_waypoints(centerline_pixels, grid, mask, spacing=0.15, direction="ccw"):
    if direction not in ("cw", "ccw"):
        raise ValueError("Direction must be cw or ccw")

    world = grid.pixel_to_world(centerline_pixels)
    if (signed_area(world) > 0) != (direction == "ccw"):
        world = world[::-1]

    dense = resample_loop(world, min(grid.resolution / 2, spacing / 4))
    step = np.linalg.norm(np.roll(dense, -1, axis=0) - dense, axis=1).mean()

    # try less smoothing if it cuts across a wall; wrap joins the loop ends
    for sigma in (0.20, 0.10, 0.05, 0.):
        smooth = ndimage.gaussian_filter1d(dense, sigma / step, axis=0, mode="wrap") if sigma else dense
        points = resample_loop(smooth, spacing)
        if _segments_inside(points, grid, mask):
            points = np.roll(points, -int(np.lexsort((points[:, 1], points[:, 0]))[0]), axis=0)
            right, left = estimate_widths(points, grid, mask)
            return WaypointPath(points, right, left, spacing, direction)
    raise ValueError("Smoothing/sampling cannot produce a path inside the track")


@dataclass(frozen=True)
class VehicleGeometry:
    length: float = 0.58
    width: float = 0.31
    reference_offset_x: float = 0.0  # forward from path reference to body centre
    safety_margin: float = 0.10
    wheelbase: float = 0.3302
    max_steering_angle: float = 0.4189

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in asdict(self).values())
                or min(self.length, self.width, self.wheelbase) <= 0
                or self.safety_margin < 0 or abs(self.reference_offset_x) >= self.length / 2
                or not 0 < self.max_steering_angle < math.pi / 2):
            raise ValueError("Vehicle dimensions/limits must be finite and positive; "
                             "margin nonnegative, reference inside body, steering below pi/2")


def _footprint_clear(point, yaw, grid, mask, vehicle, padding):
    # check the whole car rectangle, not just the waypoint at its centre
    half = np.array([vehicle.length, vehicle.width]) / 2 + vehicle.safety_margin + padding
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half
    corners[:, 0] += vehicle.reference_offset_x
    c, s = np.cos(yaw), np.sin(yaw)
    body = Polygon(grid.world_to_pixel(corners @ [[c, s], [-s, c]] + point))
    if not box(-0.5, -0.5, grid.width - 0.5, grid.height - 0.5).covers(body):
        return False

    # only check nearby cells that the body could overlap
    x0, y0, x1, y1 = body.bounds
    for y in range(max(0, math.ceil(y0 - 0.5)), min(grid.height, math.floor(y1 + 0.5) + 1)):
        for x in range(max(0, math.ceil(x0 - 0.5)), min(grid.width, math.floor(x1 + 0.5) + 1)):
            if not mask[y, x] and body.intersects(box(x - 0.5, y - 0.5, x + 0.5, y + 0.5)):
                return False
    return True


def validate_waypoints(path, grid, mask, vehicle=None):
    """Raise on unsafe geometry; steering feasibility remains a warning."""
    vehicle = vehicle or VehicleGeometry()
    points = np.asarray(path.points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (2,) or len(points) < 8 or not np.isfinite(points).all():
        raise ValueError("Path needs at least eight finite 2D points")
    widths = (np.asarray(path.widths_right), np.asarray(path.widths_left))
    if (not math.isfinite(path.spacing) or path.spacing <= 0 or any(
            w.shape != (len(points),) or not np.isfinite(w).all() or (w <= 0).any() for w in widths)):
        raise ValueError("Spacing and widths must be finite, positive and match the point count")

    # roll also includes the segment from the last point back to the first
    following = np.roll(points, -1, axis=0)
    ds = np.linalg.norm(following - points, axis=1)
    area = signed_area(points)
    checks = [
        (LineString(np.vstack((points, points[0]))).is_simple, "Path has self-intersections"),
        (len(np.unique(np.round(points, 8), axis=0)) == len(points), "Duplicate waypoints"),
        (ds.min() >= path.spacing * 0.5 and ds.max() <= path.spacing * 1.25, "Invalid spacing or closing segment"),
        (abs(area) >= grid.resolution ** 2, "Path encloses no meaningful area"),
        (path.direction in ("cw", "ccw") and (area > 0) == (path.direction == "ccw"), "Incorrect direction"),
        (grid.contains(points, mask).all(), "Waypoints leave the corridor"),
        (_segments_inside(points, grid, mask), "Connecting segments leave the corridor"),
    ]

    failures = [message for passed, message in checks if not passed]
    if failures:
        raise ValueError("; ".join(failures))

    yaw = headings(points)
    delta = np.arctan2(np.sin(np.roll(yaw, -1) - yaw), np.cos(np.roll(yaw, -1) - yaw))
    radius = math.hypot(vehicle.length / 2 + abs(vehicle.reference_offset_x) + vehicle.safety_margin,
                        vehicle.width / 2 + vehicle.safety_margin)

    for i, (a, b) in enumerate(zip(points, following)):
        # more samples for larger moves or turns; padding covers the gaps
        count = max(1, math.ceil((ds[i] + radius * abs(delta[i])) / (grid.resolution / 2)))
        for t in np.linspace(0, 1, count + 1):
            if not _footprint_clear(a + t * (b - a), yaw[i] + t * delta[i],
                                    grid, mask, vehicle, grid.resolution / 4):
                raise ValueError(f"Swept footprint plus margin collides on segment {i}")

    # this only warns about steering limits, it does not prove the car can drive it
    if (np.abs(delta) / ds).max() > math.tan(vehicle.max_steering_angle) / vehicle.wheelbase:
        warnings.warn("Curvature exceeds the configured steering limit; driving is unverified", stacklevel=2)


def generate_waypoints(map_path, spacing=0.15, seed=None, direction="ccw", vehicle=None):
    """Return validated N×4 x,y,right-width,left-width rows in world metres."""
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("Spacing must be finite and positive")

    # same pipeline for preloaded maps and maps saved after SLAM
    grid = load_map(map_path)
    mask = extract_track(grid, seed)
    path = process_waypoints(generate_centerline(mask), grid, mask, spacing, direction)

    # only return rows once the geometry checks pass
    validate_waypoints(path, grid, mask, vehicle)
    return np.column_stack((path.points, path.widths_right, path.widths_left))
