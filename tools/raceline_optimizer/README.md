# Raceline optimizer

Uses Waterloo's minimum-curvature optimizer to turn a saved SLAM map and
centerline into an optimized raceline CSV for Pure Pursuit. Waterloo's code is
not committed here; clone it in Step 1. Run commands from **`racerbot_a`**.

## 1. Setup (first time only)

Clone Waterloo's two repositories into `third_party/` (they are gitignored) at
the commits pinned in `third_party/UPSTREAM.json`, and apply the one-line SciPy
patch to the helper:

```bash
git clone https://github.com/CL2-UWaterloo/Raceline-Optimization third_party/raceline_optimization
git -C third_party/raceline_optimization checkout 9290c5d503462e46f7e3e9033002e7ddf165ba7b

git clone https://github.com/CL2-UWaterloo/trajectory_planning_helpers third_party/trajectory_planning_helpers
git -C third_party/trajectory_planning_helpers checkout fde6cee2b7bf6dd7d0f8f3d32f6a1be3cfe35b56
git -C third_party/trajectory_planning_helpers apply ../patches/scipy-flat-vector.patch
```

Use Python 3.10. On Ubuntu, install prerequisites if missing:

```bash
sudo apt-get install python3.10-venv python3.10-dev build-essential
```

```bash
python3.10 -m venv tools/raceline_optimizer/.venv
source tools/raceline_optimizer/.venv/bin/activate
python -m pip install 'setuptools==80.9.0' 'wheel==0.45.1'
python -m pip install -r tools/raceline_optimizer/requirements.lock.txt
python -m pip install --no-deps --no-build-isolation -e third_party/trajectory_planning_helpers
python -m pip check
```

**Already installed? Start at Step 2. Already have a centerline? Start at Step 3.**
Activate the environment in each new terminal:

```bash
source tools/raceline_optimizer/.venv/bin/activate
```

## 2. Generate a centerline

Save the completed map's YAML and referenced PNG/PGM in `maps/`. Keep the SLAM
map's origin and resolution. Choose the map name without `.yaml`:

```bash
MAP=levine_blocked
PYTHONPATH=src/waypoint_generator python -m waypoint_generator.cli \
  --map "maps/${MAP}.yaml" --spacing 0.15 --direction ccw
```

This creates `src/pure_pursuit_a/waypoints/${MAP}_generated.csv`.
The generator expects a closed corridor with one central island.

## 3. Optimize

Set `MAP` to the saved map's name (the same value as Step 2):

```bash
MAP=levine_blocked
python tools/raceline_optimizer/optimize.py \
  --map "maps/${MAP}.yaml" \
  --centerline "src/pure_pursuit_a/waypoints/${MAP}_generated.csv" \
  --config tools/raceline_optimizer/levine_blocked.ini
```

For a **new SLAM map**, start with `config.ini`. Existing presets are
`levine_blocked.ini` for both Levine maps, `Spielberg.ini` for Spielberg, and
`compTestTrack.ini` for compTestTrack; `slam_1026` uses `config.ini`.
If tuning is needed, copy a preset and adjust `s_reg` (smoothing); excessive
smoothing can cut across walls. Add `--overwrite` when replacing existing outputs.

## Parameters you may change

| Option                | Purpose                                                  |
| --------------------- | -------------------------------------------------------- |
| `--map PATH`          | Saved map YAML, required in both commands.               |
| `--centerline PATH`   | Generated four-column centerline CSV. (optimizer only).  |
| `--config PATH`       | Optimizer preset. Defaults to `config.ini` when omitted. |
| `--overwrite`         | Replace existing outputs after successful validation.    |
| `--spacing METRES`    | Centerline spacing. Default = `0.15`.                    |
| `--direction ccw\|cw` | Centerline travel direction. Default is`ccw`.            |

For the original `levine.yaml`, add `--corridor-map maps/levine_blocked.yaml`
to both commands. Corridor maps must match the input's image size, resolution
and origin. Use `--help` for all options.

## 4. Use the output

Each optimized folder contains the controller CSV and two plots from Waterloo's
`result_plots.py`, saved as PNGs rather than displayed interactively:

```text
src/pure_pursuit_a/waypoints/<MAP>_Optimized/
  <MAP>_optimized.csv
  overlay.png          # map, blue dashed centerline, red raceline and start marker
  curvature.png        # optimized curvature versus distance
```

Speed plots are deferred with speed profiling. Simulator diagnostics, logs and
geometry archives are not written into these folders.
The adapter styles Waterloo's figures with a map background and legend, hiding
vehicle-width debug lines and replacing its oversized direction arrow.

Rebuild Pure Pursuit and select `track:=<MAP>_Optimized/<MAP>_optimized`
without `.csv`. For example: `track:=levine_blocked_Optimized/levine_blocked_optimized`.
Follow the [Pure Pursuit README](../../src/pure_pursuit_a/README.md) for build,
simulator map selection and launch commands. Test tracking at low speed after
regeneration; successful export validates geometry, not driving performance.
