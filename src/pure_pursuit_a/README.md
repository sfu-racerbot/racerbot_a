# Pure Pursuit

Description TODO

## Building
```bash
cd /racerbot_ws
colcon build --packages-select pure_pursuit_a
```

## Launching in real life

```bash
ros2 launch pure_pursuit_a pure_pursuit_a_launch.py track:=mapname
```

## Launching in sim

```bash
ros2 launch pure_pursuit_a pure_pursuit_a_launch.py track:=mapname enable_safety:=false odom_topic:=/ego_racecar/odom use_pf:=false
```

## Safety Node
Make sure to build and run the safety node before running this node if `enable_safety` is enabled, otherwise it will not work. See [Safety Node Readme](../safety_node/README.md) for more details.

If you are testing in the simulator, disable safety with `enable_safety:=false`

## Using an optimized raceline

The [raceline optimizer](../../tools/raceline_optimizer/README.md) writes each
map's optimized CSV into `waypoints/<map_name>_Optimized/`. Select the
CSV with a relative path **without `.csv`**, including that one folder:

```bash
cd /racerbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pure_pursuit_a
source install/setup.bash
ros2 launch pure_pursuit_a pure_pursuit_a_launch.py \
  track:=levine_blocked_Optimized/levine_blocked_optimized \
  use_pf:=false odom_topic:=/ego_racecar/odom \
  min_speed:=0.5 max_speed:=0.5 enable_safety:=false
```

This example disables the safety gate for simulator-only testing. On the real
car, use safety enabled with its safety node and joystick/deadman input, and
use PF when localizing against the saved map.

`track` selects only the waypoint file. Set the simulator map separately in
`/racerbot_ws/src/f1tenth_gym_ros/config/sim.yaml`, under
`bridge: ros__parameters:`. For this example:

```yaml
map_path: '/racerbot_ws/src/racerbot_a/maps/levine_blocked.yaml'
map_img_ext: '.png'
scale: 1.0
sx: -9.89566336
sy: -0.46237203
stheta: 0.01002020
```

After changing the map/start pose, rebuild `f1tenth_gym_ros` and restart its
launch. Stop the movement node before restarting the simulator. Foxglove
shows the simulator's `/map` in fixed frame `map`.

Useful launch parameters: `track` is the CSV path above; `use_pf` selects PF
pose (`true`) or `odom_topic` pose (`false`); `min_speed`/`max_speed` are m/s;
`lookahead_base`, `lookahead_min`, `lookahead_max` are metres, and
`lookahead_speed_gain` increases lookahead with speed. Use
`ros2 launch pure_pursuit_a pure_pursuit_a_launch.py --show-args` for all options.

### Tight-turn simulator example: compTestTrack

Set the simulator map to `/racerbot_ws/src/racerbot_a/maps/compTestTrack.yaml` and place the car on its
raceline facing forward. At `0.5 m/s`, a `0.5 m` lookahead followed these tight
turns more closely than the default `1.0 m` base lookahead. After building and
sourcing the workspace, run:

```bash
ros2 run pure_pursuit_a pure_pursuit_a_node --ros-args \
  -p track:=compTestTrack_Optimized/compTestTrack_optimized \
  -p use_pf:=false -p odom_topic:=/ego_racecar/odom \
  -p min_speed:=0.5 -p max_speed:=0.5 \
  -p lookahead_base:=0.5 -p lookahead_min:=0.5 \
  -p lookahead_speed_gain:=0.0 -p enable_safety:=false
```

These settings were tested for one simulated lap with odometry localization;
they do not establish higher-speed or physical-robot performance. The optimizer checks planned geometry before exporting the CSV; test
controller tracking separately in the simulator after regeneration.
