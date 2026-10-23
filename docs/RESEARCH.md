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

    - Can cause the car to snake much more when driving down wider corridors.
        - Current Solution: Decrease safety bubble size on obstacles the car is unlikely to hit.
        - Current Implementation:
            - Decrease the size of the bubble if the obstacle is away from the car's facing direction and not very close to the car.
            - Parameters have not been tested thoroughly yet.
            - Snaking still occurs without the safety bubble

- Finding Gap
  - We choose widest gap: Backfires when a wide but shallow gap wins over a narrower, deeper one, steering us toward a dead end.
  - We choose width\*depth gap: Solid.
  - We account for how much we must turn: To be tested.
  - We choose center of widest gap: Solid.

- Steering
    - Go to best point: Car oversteers, as it chooses a new point each time.
    - Smoothing the steering with a low-pass filter seems to have very little effect.

- Speed
  - Slower speeds generally crash less and are closer to matching real life vs. simulation.
  - No solid speed function yet.

### Disparity Extender (DE)

- Speed
  - Decreasing the speed caused the car to crash into more corners. Added saftey bubble to try and counteract this. Good
  - should be the lower of two limits: slow down for sharp turns, and slow down when the target point is close. Okay but sometimes can be too slow, might or might not be a tuning problem.
- Drunk (snaking left and right)
  - The car is drunk because it steers toward the furthest point, which constantly shifts each time
  - Aim for middle gap: Basically FTG if we do that, somewhat defeats the idea of DE. To be looked into
  - Steering is smoothed with a low-pass filter: each new angle is blended with the previous one (`alpha`), so a single jumpy scan only nudges the car instead of jerking it. Lower `alpha` = smoother but slower to react.
- Obstacle avoidance
  - Too much disparity extending and we slowly go to walls, too little and we crash corners
  - Tried using safety bubble to avoid crashing into corners but this isn't good because it doesn't work well and is taking the role of disparity extender.

### Least Squares

- Car seemed to drive in "S-Curves" on straightaways
- Hypothesis was that the car would come out of turns slightly turned, drive towards a wall and then compensate, causing it to drive to the other wall. This process compounded over time and resulted in the "S-Curves"
- Deviation Penalty
  - Apply a penalty that influences the car to drive straight based on a penalty factor
  - No matter what values were tried for the penalty factor, the car either drove straight into a wall or the penalty had 0 effect
- Limiting the angle returned from the function that determined steering angle to the car's turning range also had no effect
- Slew rate limiter
  - Capped how fast the turning angle could change per callback tick, rather than how far obstacles could be.
  - This allowed for more smoother turns and helped the car stay straight coming out of turns
  - This fixed the "S-Curves" issue

## AI Driving

(No notes yet.)

# Waypoint Generation:

The offline CLI saves `<map_filename>_generated.csv` automatically in
`src/pure_pursuit/waypoints`; no `--output` argument is needed. Rebuild
`pure_pursuit` to install the CSV, then select its filename stem with the `track`
parameter. `--direction` is optional (default `ccw`): a closed loop still has an
ordered travel direction. See the [package README](../src/waypoint_generator/README.md).

### Optimized racing line (Future research)

#### What, and Why?

We want to generate waypoints that help the car complete laps faster. An optimzed racing line can use the available track information gathered via map upload or SLAM mapping to reduce sharp turns and improve driving fluidity.

#### Current approach

Currently, we generate a smoothed centreline of waypoints from completed maps and export it as a waypoint CSV. The generator estimates distances to the track boundaries and checks vehicle clearance. (No optimizastions yet)

#### 3. Current limitations/ challenges

- The centreline is not necessarily the fastest route. A shorter or smoother path also does not automatically mean a faster lap.
- The car has steering, grip, acceleration and braking limits. Optimization should consider physical contrainsts of the vehicle (this must be tested and tweaked in real life.)
- The entire car must clear the walls, including its corners and overhangs. Additional space is needed for localization and other mapping errors.
- Pure Pursuit may cut corners depending on lookahead. Optimized routes must have a dynamic lookahead function.
- Competition track will have other cars in the track, which may ruin optimized line tracking. Optimization should have a contingency plan incase of contact or space conflict with other cars.

#### 4. Implementation ideas

- Start with minimum-curvature optimization. Adjust centreline within the track boundaries to make turns less sharp. Possibly reearch minimum-lap-time optimization once vehicle dynamics and speed control are understood more via tests.
- [TUM’s global racing-trajectory optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization) implements minimum-curvature and minimum-time for waypoint generation optimization. (Could look into this)
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack) also uses the [TUM’s global racing-trajectory optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization), could be used to investigate the implementation.

### Simultaneous SLAM and waypoint generation (Future research)

#### 1. What and why?

We want waypoint generation to run while SLAM builds and improves the map. This removes the intermediary step of saving a map and running the generator separately, allowing the car to prepare its route while remaining on track.

#### 2. Current approach

For now, we assume the car can complete several mapping laps using SLAM first. After the track has been mapped, we save the completed map as YAML + PNG/PGM and provide it to the offline waypoint generator.

Preloaded maps and saved SLAM maps use the same algorithm without the SLAM mapping. The complete SLAM-to-driving workflow has not yet been demonstrated.

#### 3. current limitations/challenges

- Early maps may contain unknown/ incorrect areas, missing walls or disconnected sections. (The current algorithm requires a complete closed track.)
- Completing several laps does not guarantee a usable map. We need to determine when enough of the track has been mapped reliably.
- SLAM may regenerate waypoints depending on subsequent loop closures (can cause overlap in waypoints).
- Switching from mapping to waypoint following requires a consistent map, route and vehicle pose.
- Other cars can create temporary mapping issues, which skews waypoint generation.

#### 4. Implementation ideas

- Provide selected snapshots of the live SLAM map into the existing generator instead of repeatedly saving and loading map files.
- Wait until 1 full closed loop and generate a probability funciton for waypoints. (Can be confirmed or rejected with subsequent loop closures.)
- ForzaETH Race Stack may provide ideas for coordinating mapping, planning and driving. (requires more investigation.)

### References

- [Medial-axis transform](https://scikit-image.org/docs/stable/api/skimage.morphology.html#skimage.morphology.medial_axis)
- [ROS Humble map-loading semantics](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)
- [SLAM Toolbox Humble](https://github.com/SteveMacenski/slam_toolbox/tree/humble)
- [TUM global racing-trajectory optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [TUM graph-based local trajectory planning](https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner)
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
