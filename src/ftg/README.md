# Follow The Gap

Description TODO

## Building
```bash
cd /racerbot_ws
colcon build --packages-select ftg
```

## Launching in real life

```bash
ros2 launch ftg ftg_launch.py
```

## Launching in sim

```bash
ros2 launch ftg ftg_launch.py enable_safety:=false
```

## Safety Node
Make sure to build and run the safety node before running this node if `enable_safety` is enabled, otherwise it will not work. See [Safety Node Readme](../safety_node/README.md) for more details.

If you are testing in the simulator, disable safety with `enable_safety:=false`
