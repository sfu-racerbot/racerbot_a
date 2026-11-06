# Reactive

A reactive Follow-The-Gap (FTG) driving stack for the F1TENTH car, using LIDAR-based gap detection with a least-squares steering solver (with a furthest-point fallback method), plus a safety node for deadman-gated driving and time-to-collision emergency braking.

## Building
```bash
cd /racerbot_ws
colcon build --packages-select reactive
```

## Launching in real life

**With the least-squares method (default):**
```bash
ros2 launch reactive reactive_launch.py
```

**With the furthest-point fallback method:**
```bash
ros2 launch reactive reactive_launch.py use_fallback_follow_method:=true
```

## Launching in sim

**With the least-squares method:**
```bash
ros2 launch reactive reactive_launch.py enable_safety:=false
```

**With the furthest-point fallback method:**
```bash
ros2 launch reactive reactive_launch.py enable_safety:=false use_fallback_follow_method:=true
```

## Safety Node
Make sure to build and run the safety node before running this node if `enable_safety` is enabled, otherwise it will not work. See [Safety Node Readme](../safety_node/README.md) for more details.

If you are testing in the simulator, disable safety with `enable_safety:=false`
