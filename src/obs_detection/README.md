# obs_detection

ROS 2 node that detects an obstacle in the car's lane from LiDAR and, while one is present, drives the car with a follow-the-gap (FTG) fallback.

## How it works

1. **Detection:** finds the closest point inside the car's lane (`car_width_extended`), then looks left and right for persistent range jumps (`obstacle_edge_jump`). If both edges exist and the lateral width is at most `obstacle_max_width`, an obstacle is detected.
2. **Hysteresis:** the obstacle state is held for `obstacle_hold_frames` scans after the last detection, so it doesn't flicker.
3. **Fallback:** while the state is held, the node runs follow-the-gap on the same scan and publishes a drive command. When the state clears, it stops publishing.

If an obstacle is detected but FTG finds no valid gap, the node publishes zero speed.

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `scan` | `sensor_msgs/LaserScan` |
| Pub | `obstacle_ahead` | `std_msgs/Bool` (hysteresis-held state) |
| Pub | `drive` | `ackermann_msgs/AckermannDriveStamped` (only while obstacle held) |

**Note:** if your normal driver also publishes to `drive`, both will publish while an obstacle is held. Use a mux keyed on `obstacle_ahead`, or remap this node's `drive` to a separate topic.

## Parameters

**Detection**

| Name | Default | Description |
|------|---------|-------------|
| `max_obstacle_distance` | 10.0 | Range cap for the detector (m) |
| `obstacle_edge_jump` | 0.5 | Min range jump to count as an edge (m) |
| `car_width_extended` | 0.55 | Lane width searched for obstacles (m) |
| `fov_half_angle_deg` | 135.0 | Detector half field of view (deg) |
| `obstacle_min_target_angle_deg` | 0.0 | Declared, currently unused |
| `obstacle_max_width` | 0.8 | Max lateral obstacle width (m) |
| `obstacle_hold_frames` | 8 | Scans to hold the state after last detection |

**Follow-the-gap fallback**

| Name | Default | Description |
|------|---------|-------------|
| `max_speed` / `min_speed` | 4.5 / 0.5 | Speed limits (m/s) |
| `max_lidar_range` | 10.0 | Range cap for FTG (m) |
| `ftg_fov_half_angle_deg` | 70.0 | FTG half field of view (deg) |
| `car_width` | 0.4 | Car width for bubble and gap checks (m) |
| `minimum_gap_threshold` | 1.0 | Min range to count as free space (m) |
| `safety_bubble_scale` | 0.5 | Safety bubble size scale |
| `prev_angle_adjuster` | 0.6 | Weight of previous steering angle (smoothing) |
| `bubble_lookahead` / `bubble_half_width` | 3.0 / 0.45 | Box ahead of the car where obstacles get a bubble (m) |
| `wheelbase` | 0.33 | Vehicle wheelbase (m) |
| `lookahead_max` | 1.8 | Max steering lookahead (m) |
| `steering_deadzone` | 0.7 | Steering below this is zeroed (deg) |

## Run

```bash
ros2 launch obs_detection obs_detection.launch.py
# override any parameter, e.g.
ros2 launch obs_detection obs_detection.launch.py obstacle_hold_frames:=12 max_speed:=3.0
```