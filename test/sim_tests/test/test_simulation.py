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
import yaml
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
    def __init__(self, crash_threshold = 0.0, slowdown_threshold = 0.1):
        # The extra % distance youd like to give around the car to add a small bubble around the physical diameter of the car. Default is
        # 0.0 to create the tightest wrap around the car possible.
        self.crash_threshold = crash_threshold
        self.car_half_width = 0.16

        # Threshold for the ratio between the previous and current forward speed of the car. In practive this measures how much speed
        # the car must lose between one odometry reading and other for it to be considered a crash (0.1 == current speed is 10% of previous
        # speed). Must leave some room for error as the simulator does not completely stop all momentum when a car collides with a wall.
        # As slowdown threshold approaches 1 the detection becomes more sensitive and as a result has a higher
        # likelyhood of false positives. My testing has found that a good default threshold is 0.1
        self.slowdown_threshold = slowdown_threshold
        
        self.last_measured_speed = 0.0

        self.crash_detected = False
        self.create_summary("N/A")
        
    # Searching for a lidar distance that is less than the defined radius of the car's bubble (crash_threshold)
    # Lidar readings limited to a 270 degree angle around the car so collisions with the rear of the car will not
    # be caught by this method (Ex: high speed turn leading to the rear end of the car getting clipped due to skidding).
    def detect_crash_bubble(self, lid_msg):
        ranges = lid_msg.ranges

        #keeping track of the current angle of the lidar scan. So i can adjust the appropriate 
        angle = lid_msg.angle_max

        for distance in ranges:
            if(abs(angle) >= math.pi/2):
                # If the car is behind the front plate but within the 270 degree vision of the car then calculate the hypoteneus of
                # the triangle formed by the center of the front of the car (where the lidar is measured from), the front corner of the car,
                # and the point where our current scan meets our car (we know the distance from the lidar to the corner so
                # h = car_width/2/(cos(angle of the hypoteneus line relative to the front bumper))) 
                limit = (self.car_half_width/math.cos(abs(angle)- math.pi/2))
            else:
                # For the angles infront of our car im setting the limit to 0.1. If we slam straight on into a wall atleast one
                # of the ranges should be withing this range. If not the slowdown crash detection will handle this no problem
                limit = 0.1

            #Adding crash_threshold worth of extra room to the bubble just to add some space
            if(distance < (self.crash_threshold * (limit) + limit)):
                self.crash_detected = True
                self.create_summary("Object Entered The Cars Bubble")
            angle -= 0.005760866682976484

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

        algorithm_name = os.environ.get("ALGORITHM")
        map_name = os.environ.get("MAP")
        
        config_algo_path = os.environ.get("ALGO_CONFIG")
        with open(config_algo_path) as file:
            config = yaml.safe_load(file)

        algorithms = config["algorithms"]
        selected_algo = algorithms[algorithm_name]
        params = dict(selected_algo.get("parameters", {}))
        min_speed = float(params.get("min_speed", 0.0))
        max_speed = float(params.get("max_speed", 5.0))

        if(self.crash_detected):
            failure_message = "Status: Crashed\n"
            failure_message += "Reason: Crash --- " + reason + "\n"
        else:
            failure_message = "Status: No Crash\n"
        failure_message += "Algorithm: " + algorithm_name + "\n"
        failure_message += "Map: " + map_name + "\n"
        failure_message += "Min Speed: " + str(min_speed) + "\n"
        failure_message += "Max Speed: " + str(max_speed) 
        self.summary = failure_message


def check_nodes_life(node, request, last_odom, start_time):

    current_time = time.monotonic()

    # Giving the drivers and the simulator some time to startup so we dont treat a non started node as a crashed node
    start_grace_period = 5.0
    if current_time - start_time < start_grace_period:
        return


    # Check the autonomous driving topic to make sure it' still running.
    # Assuming we arent trying to runs the tests while also running other driving nodes then this should detect a failed
    # driving node. This could be done using timestamps like the odom test but that would require more resources being 
    # put towards keeping a constant watch on the drive topic which felt unessesary and counterproductive especially
    # if we plan on running this on the limited hardware assigned to github runners
    if node.count_publishers("/drive") == 0:
        request.config.program_summary = "The driving algorithm crashed or never started"
        assert False, ("The driving algorithm crashed or never started")

    # Due to the fact that the sim topics seem to launch quite a bit slower than the driving nodes and that we're already reading all the 
    # odometry messages so i elected to use timestamps here instead of just checking count_publishers. The simulator is considered crashed
    # if there has not been a message from it in the last two seconds or if it never recieves a message
    # Checking to make sure atleast one odom message has been sent during the start grace period
    if last_odom[0] == -1:
        request.config.program_summary = "The never started or crashed early"
        assert False ("The never started or crashed early")
    
    time_since_odom = current_time - last_odom[0]
    if time_since_odom > 2.0:
        request.config.program_summary = "The simulator crashed mid test"
        assert False, "The simulator crashed mid test"

@pytest.fixture
def sim_logger():

    algorithm = os.environ.get("ALGORITHM")
    map_name = os.environ.get("MAP")
    config_algo_path = os.environ.get("ALGO_CONFIG")
    logger = SimulationTestLogger(algorithm, map_name, config_algo_path)
    return logger


@launch_pytest.fixture
def simulation():
    sim_share = Path(
        get_package_share_directory("f1tenth_gym_ros")
    )
    algorithm_name = os.environ.get("ALGORITHM")
    map_name = os.environ.get("MAP")
    config_algo_path = os.environ.get("ALGO_CONFIG")
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

    #open config/algorithms.yaml file 
    with open(config_algo_path) as file:
        config = yaml.safe_load(file)

    #specify algorithm 
    algorithms = config["algorithms"]
    selected_algo = algorithms[algorithm_name]
    
    #use the data from the yaml to configure the algorithms parameters
    params = dict(selected_algo.get("parameters", {}))

    #for pure pursuit specifically we make sure to run the proper waypoints depending on the map being used
    if(selected_algo["package"] == "pure_pursuit_a"): 
        params["track"] = selected_algo["tracks"][map_name]
    
    
    algorithm = launch_ros.actions.Node(
        package=selected_algo["package"],
        executable=selected_algo["executable"],
        parameters=[params],
        output="screen",
    )

    return launch.LaunchDescription([
        simulator,
        algorithm,
        ReadyToTest(),
    ])



@pytest.mark.launch(fixture=simulation)
def test_odometry_received(sim_logger):
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
        deadline = time.monotonic() + 7.5

        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received_messages, ( "Simulator did not publish odometry within 15 seconds" )
    except Exception:
        sim_logger.log_result(False)
        raise
    else:
        sim_logger.log_result(True)

    finally:
        node.destroy_node()
        rclpy.shutdown()


##--- watches car's odometry, passes if loop is detected
@pytest.mark.launch(fixture=simulation)
def test_loop_detected(request):
    rclpy.init()
    # Reseting the config summaries so a previous summary doesnt bleed through into a new test
    request.config.crash_summary = ""
    request.config.loop_summary=""
    request.config.program_summary=""
    node = rclpy.create_node("main_test")

    detector = LoopDetector(radius=1.0, leave_dist=3.0)
    # class to keep track of data relevant for crash detection
    crash_detector = CrashDetector(crash_threshold = 0.0, slowdown_threshold=0.1)
    lap_pub = node.create_publisher(String, "/lap_detected", 10)
    received = []

    # counter variable to keep track of how long its been since the simulator sent a scan message. 
    # a list instead of a variable becuase variables inside of the odom_cb definition were being treated as local and not global
    last_odom_message = [-1]
    # callback function for the bubble crash detection
    def lidar_sub(msg):
        #ranges = msg.ranges
        crash_detector.detect_crash_bubble(lid_msg=msg)

    def odom_cb(msg):
        received.append(msg)
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if detector.update(p.position.x, p.position.y, yaw, t):
            text = f"Lap {detector.laps} completed"
            lap_pub.publish(String(data=text))
            node.get_logger().info(text)
        # Watching how the cars forward momentum changes. If it instantly goes down by a massive amount register that as a crash
        new_measured_speed = msg.twist.twist.linear.x
        crash_detector.detect_crash_slowdown(new_measured_speed)

        last_odom_message[0] = time.monotonic()

    node.create_subscription(Odometry, "/ego_racecar/odom", odom_cb, 10)
    node.create_subscription(LaserScan, "/scan", lidar_sub, 10)

    start_time = time.monotonic()
    
    try:
        time_offset = 100
        deadline = time.monotonic() + time_offset
        # While loop ends if we hit the timeout, finish a lap, or crash. 
        while (time.monotonic() < deadline and not(crash_detector.crash_detected) and (detector.laps <= 2)):
            rclpy.spin_once(node, timeout_sec=0.1)
            check_nodes_life(node, request, last_odom_message, start_time)

    finally:
        request.config.loop_summary = detector.summary()   # saved for the report
        request.config.crash_summary = crash_detector.summary
        node.destroy_node()
        rclpy.shutdown()

    assert received, "Simulator did not publish odometry"
    assert detector.laps >= 1, "Car did not complete a lap within 30 seconds"
    assert not(crash_detector.crash_detected), "Crash Detected During The Run"

