## Launching the System

Before running any commands, source the ROS 2 and workspace setup files in every terminal.

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
```

### Base Launch

Starts the gamepad, LiDAR, and VESC drivers required for the car to operate.

```
ros2 launch f1tenth_stack bringup_launch.py
```

Starts the RealSense D435i camera and publishes its sensor data.

```
ros2 launch racerbot_launch realsense_camera_launch.py
```

### Safety Launch

Starts the safety node, with Time-to-Collision (TTC) disabled.

```
ros2 launch safety_node launch_safety.py enable_ttc:=false
```

### Reactive Methods Launch

Uses Follow the Gap to identify the largest open space and steer the car toward it.

```
ros2 launch ftg ftg_launch.py
```

Extends obstacle disparities in LiDAR data to account for the car's physical width and improve obstacle avoidance.

```
ros2 launch disparity_extender disparity_extender_launch.py
```

Uses a least-squares method to calculate the steering command based on the available reactive navigation data. This is the default method.

```
ros2 launch reactive reactive_launch.py
```

Enables obstacle detection and switches to a fallback following method when necessary.

```
ros2 launch reactive reactive_launch.py use_fallback_follow_method:=true
```

### Mapping Methods Launch

#### Map Creation Launch

Runs SLAM Toolbox to build a map of the environment using LiDAR data.

```
ros2 launch slam_toolbox online_async_launch.py params_file:=$HOME/racerbot-ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```

Saves the generated SLAM map to /racerbot_ws/racerbot_a/maps/track_slam.

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /slam_map \
  -f /racerbot_ws/racerbot_a/maps/track_slam \
  --ros-args -p map_subscribe_transient_local:=true
```

Alternatively, launch the Racerbot SLAM configuration:

```
ros2 launch racerbot_launch slam_launch.py
```

#### Map Methods Launch

Uses a particle filter to estimate the car's position within the map.

```
ros2 launch particle_filter localize_launch.py
```

Generates waypoints for the `track_slam` map, defining the racing line for Pure Pursuit to follow.

```
TBD
```

Uses the Pure Pursuit algorithm to follow the racing line on the track_slam map using the generated waypoints.

```
ros2 launch pure_pursuit_a pure_pursuit_a_launch.py track:=track_slam
```

### Dashboard

The dashboard already runs automatically at boot, so it does not need to be launched manually. Check its status with:

```
systemctl status racerbot-dashboard
```
