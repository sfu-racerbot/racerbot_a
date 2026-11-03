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
