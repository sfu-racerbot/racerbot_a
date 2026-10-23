# Waypoint generator

Generate centreline waypoints from a completed map (YAML + PNG/PGM).
All commands below run from the root of the `racerbot_a` repository.

## Setup (once)

From `racerbot_a`, install the Python package and dependencies:

```bash
python3 -m pip install -e ./src/waypoint_generator
```

Build the ROS package from the same repository root:

```bash
colcon build --packages-select waypoint_generator
source install/setup.bash
```

## Generate waypoints

Run the generator once for your chosen map. Replace the placeholders with your
YAML file path and waypoint spacing in metres:

```text
ros2 run waypoint_generator generate_waypoints \
  --map "/path/to/your/map.yaml" --spacing <spacing_metres>
```

The YAML references the map image. Relative map paths are resolved from your
current directory. Output is saved automatically relative to the repository root:

```text
src/pure_pursuit/waypoints/<map_name>_generated.csv
```

`<map_name>` is the YAML filename without `.yaml` or `.yml`, preserving capitalization.

## Load the CSV in Pure Pursuit (optional)

This loads the generated CSV; it does not run the generator again. From
`racerbot_a`, rebuild Pure Pursuit to install the new CSV:

```bash
colcon build --packages-select pure_pursuit
source install/setup.bash
```

Then replace `<map_name>` below with the name described above:

```text
ros2 run pure_pursuit pure_pursuit_node --ros-args -p track:=<map_name>_generated
```

The controller's driving logic remains unfinished; loading a CSV does not verify driving.

## Parameters

- `--map`: input the corresponding path to saved map's YAML file.

- `--spacing`: interval in metres between each waypoint, default is `0.15`.

- `--length`, `--width`, `--reference-offset-x`, `--safety-margin`: vehicle geometry
  in metres. Defaults are `0.58`, `0.31`, `0`, `0.10`.

- `--wheelbase`, `--max-steering-angle`: steering warning limits. Defaults are
  `0.3302 m`, `0.4189 rad`.

- `--overwrite`: use this to replace an existing generated CSV. Failed generation
  preserves the existing file.

- `--direction`: `ccw` (default) or `cw`, selecting travel direction around the loop.

To do:

- Update code in `pure_pursuit` to include parameter for CSV file.
