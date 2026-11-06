# Disparity Extender

The Disparity Extender Node processes LiDAR disparities to safely expand obstacles and select the furthest reachable point, allowing the vehicle to drive faster while reducing the risk of clipping obstacles.

## Building
```bash
cd /racerbot_ws
colcon build --packages-select disparity_extender
```

## Launching in real life

```bash
ros2 launch disparity_extender disparity_extender_launch.py
```

## Launching in sim

```bash
ros2 launch disparity_extender disparity_extender_launch.py enable_safety:=false
```

## Safety Node
Make sure to build and run the safety node before running this node if `enable_safety` is enabled, otherwise it will not work. See [Safety Node Readme](../safety_node/README.md) for more details.

If you are testing in the simulator, disable the deadman switch with `enable_safety:=false`
