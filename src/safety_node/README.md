# Safety Node
A ROS2 node that adds a deadman switch (based on input from the `/joy` topic) to ensure code only runs when operators want it to.

## Usage
### Integrating with other nodes
In whatever node you want to use the safety node you need to add a few snippets of code

Add a parameter to control whether or not the safety node is used:
```cpp
this->declare_parameter("enable_safety", true);

// make sure to declare this in the node header (bool enable_safety_)
enable_safety_ = this->get_parameter("enable_safety").as_bool();
```

Also update the launch file
```python
enable_safety_arg = DeclareLaunchArgument(
    "enable_safety",
    default_value="true",
    description="Whether the safety brake from the safety node is enabled",
)
...
return LaunchDescription(
        [
            enable_safety_arg,
            ...
        ]
)
```

Subscribe to the safety node's topic
```cpp
// again, don't forget to declare the subscriber in the node's header (rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_brake_sub_)
safety_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/safety/brake", 10,
        std::bind(&GapFollowNode::safety_brake_callback, this, std::placeholders::_1));
```

Declare this member variable in your node's header
```cpp
bool safety_brake_engaged_ = false;
```

This should be placed in whatever function publishes drive commands (e.g. `gap_callback`)
```cpp
int safety_publisher_count = this->safety_brake_sub_->get_publisher_count();
if (enable_safety_ && (safety_brake_engaged_ || safety_publisher_count == 0))
{
    // for example: publish a drive message with speed at 0.0
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = commanded_steering_angle_;
        drive_msg.drive.speed = 0.0;
        drive_pub_->publish(drive_msg);

    // ALWAYS RETURN SO NOTHING ELSE RUNS!!!!!
    return;
}
```

Add std_msgs to `package.xml` and update `CMakeLists.txt` for each node declared in `CMakeLists.txt`
```xml
<depend>std_msgs</depend>
```

```cmake
find_package(std_msgs REQUIRED)
...
ament_target_dependencies(... std_msgs)
```

### Running the safety node
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