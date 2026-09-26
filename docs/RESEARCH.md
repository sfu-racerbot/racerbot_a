# Research

Contains notes and learnings from testing in simulation, on the car, and from lectures, so we can track what's worked, what hasn't, and why

## Simulation

- If our turning in simulation is not consistent with our turning in real life testing, the likely culprit to consider would be the simulator's vehicle dynamics model.
    - If nonlinear: We may be cornering too fast, causing saturation/sliding that wouldn't be picked up with a kinematic or linear model.

## Safety
- TTC needs reliable odometry.
- Safety should be built into the driving controller, not a separate process, otherwise they compete with each other.

## Reactive Driving

### Follow the Gap (FTG)
- Preprocess lidar scan
    - Restrict scan to 180 degrees: Good since it removes lidar points behind and to the sides of the car that aren't relevant to forward driving.
    - Smooth by taking mean every 3 scans: Don't know if it makes a difference.
    - Remove outlier scans: To be tested.

- Safety bubble
    - Using exact value instead of linear approximation is good.
    - Overall, good to avoid crashing into corners.

- Finding Gap
    - We choose widest gap: Backfires when a wide but shallow gap wins over a narrower, deeper one, steering us toward a dead end.
    - We choose width*depth gap: Solid.
    - We account for how much we must turn: To be tested.
    - We choose center of widest gap: Solid.

- Steering
    - Go to best point: Car oversteers, as it chooses a new point each time.

- Speed
    - Slower speeds generally crash less and are closer to matching real life vs. simulation.
    - No solid speed function yet.

### Disparity Extender (DE)

- Decreasing the speed caused the car to crash into more corners. Added saftey bubble to try and counteract this. Good


### Least Squares

(No notes yet.)

## Mapping Driving


### TF Systems

**Issue:** the car tracks its own motion by counting wheel turns and steering angle (dead reckoning). This is smooth and fast, but drifts over time from wheel slip, tire wear, and hardware imperfections. (this live estimate is the `odom` frame.)

**Frames and the tree**
- Every coordinate frame (`map`, `odom`, `base_link`, sensor frames) sits in one tree, and each frame has exactly one parent. (Set by [REP 105](https://www.ros.org/reps/rep-0105.html), the official ROS spec for robot frames.)
- Fixed parts on the car (sensor mounts) publish a static transform once. Moving things publish a live one continuously.
- Any node can ask "where is X relative to Y" and tf2 chains the pieces together, using [tf2 tutorials, ROS 2 docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html) as the reference.

**Order**
`map` -> `odom` -> `base_link` -> sensors
- `base_link`: bolted to the chassis, doesn't move relative to the car (rear axle center).
- `odom`: dead reckoning. Drifts over time, but continuous ([REP 105](https://www.ros.org/reps/rep-0105.html)).
- `map`: tied to the saved track map. Accurate long term, but can jump when localization corrects it.
- Map and odom can't both attach straight to base_link. A frame can only have one parent, so they have to be stacked.

![image](https://github.com/user-attachments/assets/8ea4d426-5d16-4cf7-b944-fdc7f856afce)

**Odometry (`odom → base_link`)**
- Published by whatever reads the wheels/VESC (plus IMU if we fuse it), at a high steady rate.
- This is what our reactive nodes actually use (wall follow, follow the gap, pure pursuit) since they only need recent motion, not a global position.
- Reliable odometry is also what our TTC safety logic needs (see Safety above).


**Splitting up `map` and `odom`**
- Speed: localization updates slower (10 to 40Hz) than control needs (50 to 100+Hz). Splitting allows fast nodes work asynchronously with slow nodes.
- Safety: jumps only ever happen in `map` to `odom`. `map` to `base_link` never jumps, so PID and follow the gap never get destabilized mid turn.
- Simplicity: anything that wants the car's map position (costmap, raceline follower, RViz) just asks for it, and tf2 chains map, odom, and base_link together automatically.


**Debugging:** use `ros2 run tf2_ros tf2_echo  ` to show a live transform, and use `ros2 run rqt_tf_tree rqt_tf_tree` to see the whole tree for debugging purposes.



## AI Driving

(No notes yet.)
