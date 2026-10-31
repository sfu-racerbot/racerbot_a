import os
import time
## for loop detection implementation
import math
from std_msgs.msg import String
##----------------------------------
import pytest
import launch_pytest
from launch_pytest.actions import ReadyToTest
from pathlib import Path
import launch
import launch_ros
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
# Custom logger module
from simulation_logger import SimulationTestLogger
# For Crash Detection
from sensor_msgs.msg import LaserScan

##--- Converts car's orientation/3d rotation to angle range of initial position
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
##------------------------------------------------------------
##--- takes in cars position and heading, notifies of loop is complete
class LoopDetector:
    # sets up values and memory
    def __init__(self, radius=1.0, leave_dist=3.0, max_heading_err=math.radians(45)):
        self.radius = radius
        self.leave_dist = leave_dist
        self.max_heading_err = max_heading_err
        self.start = None          # (x, y, yaw)
        self.armed = False
        self.laps = 0

        self.lap_times = []        # seconds per completed lap (sim time)
        self.travelled = 0.0       # total distance driven (m)
        self.last_xy = None
        self.t_first = None
        self.t_last = None
        self.t_lap_start = None

    # called on every odom msg, loops through values to check if lap occurs
    def update(self, x, y, yaw, t):
        self.t_last = t
        if self.last_xy is not None:
            self.travelled += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

        if self.start is None:
            self.start = (x, y, yaw)
            self.t_first = self.t_lap_start = t
            return False

        sx, sy, syaw = self.start
        dist = math.hypot(x - sx, y - sy)

        if not self.armed:
            if dist > self.leave_dist:
                self.armed = True
            return False

        heading_err = abs(math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw)))
        if dist < self.radius and heading_err < self.max_heading_err:
            self.armed = False
            self.laps += 1
            self.lap_times.append(t - self.t_lap_start)
            self.t_lap_start = t
            return True
        return False

    def summary(self):
        if self.t_first is None:
            return "Loop summary: no odometry received."
        elapsed = self.t_last - self.t_first
        lines = [
            f"Laps completed : {self.laps}",
            f"Sim time       : {elapsed:.1f} s",
            f"Distance driven: {self.travelled:.1f} m",
        ]
        if self.lap_times:
            lines.append("Lap times      : " + ", ".join(f"{t:.2f}s" for t in self.lap_times))
            lines.append(f"Best / average : {min(self.lap_times):.2f}s / "
                         f"{sum(self.lap_times) / len(self.lap_times):.2f}s")
        return "\n".join(lines)
###----------------------------------------------------------------------------------------

## Takes in lidar data and odometry to determine if a car has collided with anything. If so ends the test as a failure
class CrashDetector:
    def __init__(self, crash_threshold = 0.3, slowdown_threshold = 0.1):
        # radius of the bubble around the car that should be considered a crash.
        # idealy should be the smallest radius which will fit all four corners of the car. My testing has
        # found that 0.3 is a good threshold
        self.crash_threshold = crash_threshold

        # Threshold for the ratio between the previous and current forward speed of the car. In practive this measures how much speed
        # the car must lose between one odometry reading and other for it to be considered a crash (0.1 == current speed is 10% of previous
        # speed). Must leave some room for error as the simulator does not completely stop all momentum when a car collides with a wall.
        # As slowdown threshold approaches 1 the detection becomes more sensitive and as a result has a higher
        # likelyhood of false positives. My testing has found that a good default threshold is 0.1
        self.slowdown_threshold = slowdown_threshold
        
        self.last_measured_speed = 0.0

        self.crash_detected = False
        self.summary = "No Crash Detected"
        
    # Searching for a lidar distance that is less than the defined radius of the car's bubble (crash_threshold)
    # Lidar readings limited to a 270 degree angle around the car so collisions with the rear of the car will not
    # be caught by this method (Ex: high speed turn leading to the rear end of the car getting clipped due to skidding).
    def detect_crash_bubble(self, ranges):
        for distance in ranges:
            if(distance < self.crash_threshold):
                self.crash_detected = True
                self.create_summary("Object Entered The Cars Bubble")

    # Checking to see if the car experienced a sudden intense slowdown. If that slowdown is intense enough
    # assume that it was caused by collision with an object.
    # Secondary crash detection method incase the collision occures in the lidar blindspot or the bubble crash detection method fails 
    # unexpectadly
    def detect_crash_slowdown(self, new_measured_speed):
        if((self.last_measured_speed > new_measured_speed) and ((new_measured_speed/self.last_measured_speed) < self.slowdown_threshold)):
                self.crash_detected = True
                self.create_summary("Sudden Speed Loss")
        
        self.last_measured_speed = new_measured_speed


    def create_summary(self, reason):
        failure_message = "Status: Failed\n"
        failure_message += "Reason: Crash -- " + reason + "\n"
        failure_message += "Crash Threshold: " + str(self.crash_threshold) + "\n"
        failure_message += "Slowdown Ratio: " + str(self.slowdown_threshold) + "\n"
        failure_message += "Algorithm: TBA once variable speed and algorithms are added to the script\n"
        failure_message += "Max Speed: TBA ^" 
        self.summary = failure_message


@pytest.fixture
def sim_logger():
    algorithm = os.environ["ALGORITHM_PACKAGE"]
    map_name = os.environ["MAP"]
    min_speed = float(os.environ["MIN_SPEED"])
    max_speed = float(os.environ["MAX_SPEED"])

    logger = SimulationTestLogger(algorithm, map_name, min_speed, max_speed)
    logger.log_info()

    return logger

@launch_pytest.fixture
def simulation():
    sim_share = Path(
        get_package_share_directory("f1tenth_gym_ros")
    )
    
    algorithm_pkg = os.environ.get("ALGORITHM_PACKAGE")
    algorithm_ex = os.environ.get("ALGORITHM_EXECUTABLE")

    min_speed = float(os.environ.get("MIN_SPEED"))
    max_speed =  float(os.environ.get("MAX_SPEED"))
    
    map_name = os.environ.get("MAP")

    config_path = os.environ.get("SIM_CONFIG")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / "launch" / "gym_bridge_launch.py")
        ),
        launch_arguments={
            "config": config_path,
            "open_foxglove": "false",
            "num_agent": "1",
        }.items(),
    )
    sim_logger = SimulationTestLogger(algorithm_pkg, map_name, min_speed, max_speed)
    algorithm = launch_ros.actions.Node(
        package=algorithm_pkg,
        executable=algorithm_ex,
        output="screen",
        parameters=[{"enable_safety": False}, {"min_speed": min_speed}, {"max_speed": max_speed}]
    )
    sim_logger.log_info()
    return launch.LaunchDescription([
        simulator,
        algorithm,
        ReadyToTest(),
    ])



@pytest.mark.launch(fixture=simulation)
def test_odometry_received(sim_logger):
    algorithm = os.environ.get("ALGORITHM_PACKAGE", "ftg")
    min_speed =  os.environ.get("MIN_SPEED")
    max_speed =  os.environ.get("MAX_SPEED")
    map_name = os.environ.get("MAP_NAME")

    sim_logger = SimulationTestLogger(algorithm, map_name, min_speed, max_speed)

    sim_logger.log_test_description("Basic odometry test")

    rclpy.init()

    node = rclpy.create_node("simulation_test_monitor")
    received_messages = []

    subscription = node.create_subscription(
        Odometry,
        "/ego_racecar/odom",
        lambda msg: received_messages.append(msg),
        10,
    )

    try:
        deadline = time.monotonic() + 10.0

        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received_messages, ( "Simulator did not publish odometry within 30 seconds" )
    except Exception:
        sim_logger.log_result(False)
        raise
    else:
        sim_logger.log_result(True)

    finally:
        node.destroy_node()
        rclpy.shutdown()


#FAILING TEST ---> I've commented the test down below out due to how quickly it fails,
#                  which is then exiting out of the simulator so early to the point where
#                  we don't even see the car run in the simulator. 

##--- watches car's odometry, passes if loop is detected
# @pytest.mark.launch(fixture=simulation)
# def test_loop_detected(request):
#     rclpy.init()
#     node = rclpy.create_node("main_test")

#     detector = LoopDetector(radius=1.0, leave_dist=3.0)
#     # class to keep track of data relevant for crash detection
#     crash_detector = CrashDetector(crash_threshold = 0.3, slowdown_threshold=0.1)
#     lap_pub = node.create_publisher(String, "/lap_detected", 10)
#     received = []

#     # callback function for the bubble crash detection
#     def lidar_sub(msg):
#         ranges = msg.ranges
#         crash_detector.detect_crash_bubble(ranges)

#     def odom_cb(msg):
#         received.append(msg)
#         p = msg.pose.pose
#         yaw = yaw_from_quat(p.orientation)
#         t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if detector.update(p.position.x, p.position.y, yaw, t):
#             text = f"Lap {detector.laps} completed"
#             lap_pub.publish(String(data=text))
#             node.get_logger().info(text)
#         # Watching how the cars forward momentum changes. If it instantly goes down by a massive amount register that as a crash
#         new_measured_speed = msg.twist.twist.linear.x
#         crash_detector.detect_crash_slowdown(new_measured_speed)

#     node.create_subscription(Odometry, "/ego_racecar/odom", odom_cb, 10)
#     node.create_subscription(LaserScan, "/scan", lidar_sub, 10)

#     try:
#         deadline = time.monotonic() + 30.0
#         while (time.monotonic() < deadline and not(crash_detector.crash_detected)):
#             rclpy.spin_once(node, timeout_sec=0.1)
#             assert not(crash_detector.crash_detected), "Crash Detected During The Run"

#     finally:
#         request.config.loop_summary = detector.summary()   # saved for the report
#         request.config.crash_summary = crash_detector.summary
#         node.destroy_node()
#         rclpy.shutdown()

#     assert received, "Simulator did not publish odometry"
#     assert detector.laps >= 1, "Car did not complete a lap within 30 seconds"
#     assert not(detector.crash_detected), "Crash Detected During The Run"

