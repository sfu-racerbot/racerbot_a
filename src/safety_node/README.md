# Safety Node
A ROS2 node that adds a deadman switch (based on input from the `/joy` topic) to ensure code only runs when operators want it to.

## Usage
```bash
cd /racerbot-ws
colcon build --packages-select safety_node
ros2 launch safety_node launch_safety.py
```

### Without TTC enabled
```bash
ros2 launch safety_node launch_safety.py enable_ttc:=false
```

### Testing Deadman in the simulator
In one terminal run:
```bash
ros2 launch safety_node launch_safety.py message_timeout_duration:=false
```

In another run your algorithim node (FTG, DE, etc.)

Then to use the deadman switch, run:
```bash
ros2 topic pub -r 10 /joy sensor_msgs/msg/Joy "{header: {frame_id: ''}, axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]}"
```
To kill the switch (i.e. release the deadman), hit Ctrl+C

## Topics Published
- `/safety/brake` (`std_msgs::msg::Bool`) Whether or not the car should brake due to the deadman switch not being held/connected