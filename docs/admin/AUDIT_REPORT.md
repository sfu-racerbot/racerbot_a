# Racerbot A Driving-Code Audit

**Audit date:** 2026-07-25  
**Audited commit:** `66020354af7f6c7924a647025fd2eab47456796c`  
**Scope:** `src/reactive`, `src/ftg_node`, and `src/disparity_extender`, compared with the ROS 2 Jazzy workspace's checked-in hardware, topic, mux, and safety configuration.

## Executive verdict

**Not ready for physical driving.** The packages build on ROS 2 Jazzy and the
default `reactive_launch.py` route does place `safety_node` between the
controller and `/drive`. However, there are release-blocking safety and vehicle
integration problems:

1. the controller can publish motion directly to `/drive` without a live LB
   deadman signal when it is run outside the composition launch;
2. the checked-in TTC source uses the wrong odometry topic, while the installed
   `/odom` build is stale and still releases its stop at zero speed with an
   obstacle present;
3. the default straight-line command is about **4.326 m/s**, with no configured
   maximum-speed clamp; and
4. the steering range substantially exceeds this car's calibrated actuator
   range.

Do not floor-test this implementation in its current state. The findings below
should be resolved and covered by automated tests before following the
workspace's wheels-off-ground and low-speed physical test ladder.

## Emergency-collision follow-up: `/odom` did not fix the wall strikes

This follow-up was performed after wall contact continued with the TTC input
changed to `/odom`. It supersedes the topic-only diagnosis in A-02 and the
severity originally assigned to A-09.

### Root conclusion

Changing the subscription fixes only one wiring error. The collision is
explained by a **fail-open, stateless TTC stop**, no hard footprint-clearance
check, unsafe gap targeting, and deployed software that does not match the
checked-in source.

Every LaserScan callback first clears `estop_active_`. TTC is evaluated only
while measured closing speed is positive. Once a stop slows odometry to zero,
all beams are skipped, the stop clears, and the next positive `/drive_raw`
command is forwarded even if the wall remains directly ahead. This produces a
drive/stop/drive pulse or creep into the wall. Raising the TTC threshold cannot
correct that state-machine error.

### Deployed code did not match checked-in code

Installed `reactive` artifacts were dated July 22 while relevant source and
launch files were changed on July 25. Runtime inspection showed:

| Item | Checked-in source | Installed executable |
|---|---|---|
| Odometry topic | `/ego_racecar/odom` | `/odom` |
| TTC threshold | `0.3 s` | `30.0 s` |
| Gap controller | least-squares, up to 4.326 m/s | older direct-angle, up to 2.0 m/s |
| Gap target | fallback uses `gap_start` | older finder retains first furthest beam |

The `/odom` experiment reached an installed safety executable, but did not test
the source currently visible in this repository. C++ changes require a rebuild,
sourcing that exact install, and confirming live parameters and subscriptions
before physical conclusions are reproducible.

### Reproduced failure

The installed executable was tested in an isolated ROS domain with synthetic
messages only; no hardware or motor node was started.

1. With LB held, `/odom.linear.x = 0`, a forward range of `0.1 m`, and a
   positive `/drive_raw` request, the positive command was forwarded.
2. With `/odom.linear.x = 2 m/s` and a forward range of `1 m`, TTC triggered
   and zero speed was published.
3. Returning odometry to zero re-created step 1 while the obstacle remained.

This is why `/odom`—even with the installed `30 s` threshold—did not prevent
contact.

### TTC is not a footprint-clearance check

The node divides raw LiDAR range by projected speed. It neither subtracts the
body boundary nor checks static minimum clearance. With the workspace geometry
(`0.58 m` length, `0.31 m` width, LiDAR `0.33 m` ahead of the rear-axle
reference), the body extends about `0.12 m` ahead of the LiDAR and `0.155 m`
sideways. A bumper or corner can contact a wall while raw range remains
positive. At zero speed TTC cannot guard clearance at all.

At the checked-in straight speed of `4.326 m/s` and threshold of `0.3 s`, the
raw trigger is about `1.30 m`. Body clearance is about `1.18 m`; even ignoring
latency, that requires roughly `8.0 m/s²` constant deceleration.

The node has no odometry timestamp and does not require odometry before motion.
Missing odom behaves as zero speed and permits launch; stale odom is used
forever. If physical forward odometry is negative, the
`max(speed * cos(angle), 0)` calculation disables forward TTC. The sign must be
verified wheels-off-ground.

### Why the workspace “vibecoded” code behaves better

The comparison implementation does not rely on TTC alone. Its current code:

- refuses motion when odometry is missing or stale;
- computes rectangular body-boundary distance and hard footprint clearance;
- computes TTC from clearance rather than raw range;
- uses a `0.5 s` threshold with a `2.0 m/s` maximum;
- inflates obstacles by half-width plus a `0.10 m` margin (`0.255 m` total,
  versus Racerbot A's approximately `0.175 m`);
- checks gap depth and physical width and targets the midpoint;
- limits corner speed and fails safe on invalid scans; and
- passes all 29 current ROS-free gap-logic tests.

Racerbot A can instead target a gap edge, fits LiDAR obstacle-hit points rather
than a proven collision-free centerline, and does not check wheelbase, LiDAR
pose, swept body corners, or physical turn clearance. The comparison code's
planner avoidance, static-clearance stop, and TTC form three layers; Racerbot A
does not currently have those layers working together.

## Severity definitions

- **Critical:** can permit unintended motion or defeat a required safety layer;
  blocks physical testing.
- **High:** can produce unsafe or materially incorrect driving; should block
  physical testing until resolved.
- **Medium:** reliability, maintainability, or integration weakness that should
  be addressed before wider use.
- **Low:** cleanup or documentation issue with limited immediate runtime impact.

## Findings

### A-01 — Critical — The drive-producing node has no mandatory deadman gate

Evidence:

- `src/reactive/src/gap_follow_node.cpp:7` creates a `drive` publisher.
- `src/reactive/src/gap_follow_node.cpp:94-98` publishes non-zero Ackermann
  commands without subscribing to `/joy` or checking LB.
- `src/reactive/launch/reactive_launch.py:53-70` remaps this publisher to
  `drive_raw`, and `safety_node` normally gates that stream. The protection
  therefore depends on using this exact launch composition.
- `ros2 run reactive gap_follow_node` bypasses the wrapper and publishes
  directly to `/drive`. An isolated synthetic `Gap` test, with no `/joy`
  publisher at all, produced:

  ```text
  steering_angle: ~0.0 rad
  speed: 4.326356 m/s
  ```

Impact: running, reusing, remapping, or composing the executable directly can
move the physical car without LB. This violates the workspace policy that every
node capable of moving the car independently enforce a fresh LB hold.

Required direction: put the live `/joy`/LB check in every executable that can
publish a motion command. A safety-wrapper composition can remain as
defence-in-depth, but it must not be the only deadman gate.

### A-02 — Critical — Odometry integration is inconsistent and fails open

Evidence:

- The checked-in source subscribes to `/ego_racecar/odom`, although the car
  publishes `/odom`.
- The installed executable inspected in this follow-up did subscribe to
  `/odom`, proving that the reported topic change exists only in stale build
  artifacts and not in the current source tree.
- Neither version records odometry time or blocks motion before fresh odometry.
- Missing odometry leaves `speed_` at zero, skips every TTC calculation, and
  allows a positive raw command through.
- The code considers only positive projected `linear.x`; a reversed physical
  odometry sign would also skip forward TTC.

Impact: source inspection, launch behavior, and physical-test behavior are not
reproducible from one version, while missing, stale, or wrong-sign speed can
silently disable the collision layer.

Required direction: parameterize `/odom` as the physical default, require fresh
odometry before any positive command, validate its sign wheels-off-ground,
reject implausible data, and test the exact installed build.

### A-03 — Critical — Default speed reaches 4.326 m/s with no speed clamp

Evidence:

- `src/reactive/src/gap_follow_node.cpp:147-150` uses
  `-1.6 * log(angle + 0.3) + 2.4`.
- At zero steering this is 4.326356 m/s, confirmed by the isolated runtime
  test.
- The result is not clamped and there is no `max_speed` parameter.
- The checked-in VESC limit corresponds to about 5.039 m/s
  (`23250 ERPM / 4614 ERPM-per-m/s`), so the downstream driver does not reduce
  a 4.326 m/s request.
- The fallback controller also uses fixed speeds as high as 3.0 m/s
  (`src/reactive/src/gap_follow_node.cpp:40-50`).

Impact: the first valid straight gap can request near-racing speed, including
on an unvalidated track and before the controller's geometry is proven on the
car.

Required direction: add conservative, validated min/max speed parameters and
clamp every output. Initial physical validation should use the workspace's
low-speed values, not the present formula maximum.

### A-04 — High — FOV masking creates division by zero in obstacle extension

Evidence:

- `preprocess_lidar()` sets beams outside ±90 degrees to exactly zero
  (`src/reactive/src/gap_finder_node.cpp:63-70`).
- At an FOV boundary, an open in-FOV beam commonly differs from that artificial
  zero by more than `disparity_threshold_`.
- `extend_obstacles()` then chooses `closer_range == 0` and divides by it at
  lines 91-94 before converting the non-finite result to `size_t`.
- A uniform open full-circle synthetic scan produced "No valid gap found,"
  demonstrating that artificial boundary values can eliminate otherwise open
  space. The exact amount of corruption is scan-resolution dependent.

Impact: a clear scene can yield no command, and the floating-to-integer
conversion has undefined/implementation-dependent behavior.

Required direction: crop the processing window without presenting mask zeros
as physical disparities, and reject non-positive/non-finite denominators before
angle-to-index conversion.

### A-05 — High — NaN LaserScan values are not sanitized and can stall gap search

Evidence:

- `preprocess_lidar()` only applies `std::min`; it never checks
  `std::isfinite` or `range_min` (`src/reactive/src/gap_finder_node.cpp:59-73`).
- A NaN survives the `std::min` call.
- In `find_furthest_gap()`, both `NaN <= threshold` and `NaN > threshold` are
  false (`src/reactive/src/gap_finder_node.cpp:115-131`). The index is then not
  advanced, while width arithmetic also consumes the NaN.

Impact: a malformed or degraded scan can hang the callback, consume a CPU core,
and stop controller updates. The mux should eventually time out the old
command, but the controller itself is no longer healthy or diagnosable.

Required direction: sanitize every beam using `range_min`, `range_max`, and
finite checks; ensure every loop path advances; add NaN, infinity, zero, empty,
and malformed-scan tests.

### A-06 — High — Steering commands exceed this car's calibrated range

Evidence:

- `src/reactive/include/gap_follow_node.hpp:27` allows ±90 degrees
  (±1.571 rad).
- This car maps steering to servo position as
  `servo = -1.2135 * angle + 0.5304`, with servo limits 0.15 to 0.85.
- Those checked-in values reach the configured servo stops at approximately
  +0.313 rad and -0.263 rad.
- The VESC driver clips the resulting servo value, so much of the controller's
  requested range collapses to a saturated physical command.

Impact: the path model assumes steering changes that the actuator cannot
produce. Speed selection is then based on the requested angle rather than
known achievable steering, reducing predictability and creating discontinuous
control at the stops.

Required direction: use a car-specific steering limit at or inside the
calibrated range and validate the sign and asymmetric limits wheels-off-ground.

### A-07 — High — The fallback target is the first beam of the gap, not its best point

Evidence:

- Gap selection searches for the gap containing the furthest range
  (`src/reactive/src/gap_finder_node.cpp:144-153`).
- The published `target_angle` and `target_range` instead use `gap_start`
  (`src/reactive/src/gap_finder_node.cpp:53-54`).
- The fallback controller steers directly to that angle
  (`src/reactive/src/gap_follow_node.cpp:38-57`).
- The least-squares controller also invokes this fallback automatically when
  there are too few points (`src/reactive/src/gap_follow_node.cpp:63-67`).

Impact: fallback behavior aims at a gap edge—typically adjacent to the inflated
obstacle—rather than the gap center or furthest safe point.

Required direction: explicitly publish and test a center/best-point target that
maintains vehicle-footprint clearance.

### A-08 — High — Control outputs and parameters are not validated

Evidence:

- `degree`, `steering_gain`, and `lookahead_distance` accept arbitrary values
  without bounds (`src/reactive/src/gap_follow_node.cpp:13-23`).
- The code assumes `angles.size() == ranges.size()` but does not check it
  (`src/reactive/src/gap_follow_node.cpp:74-79`).
- Polynomial coefficients, target output, steering, and speed are not checked
  for finite values before publication.
- A negative degree, malformed `Gap`, ill-conditioned fit, or non-finite input
  can cause invalid Eigen dimensions, out-of-bounds access, or NaN actuator
  commands.

Impact: parameter mistakes or bad intermediate data can turn into crashes or
invalid commands instead of a fail-safe zero-speed command.

Required direction: validate parameters at startup, validate every received
message, and refuse to publish motion unless the final speed and steering are
finite and within explicit limits.

### A-09 — Critical — TTC stop releases at zero speed while the wall remains

Evidence:

- Each scan clears `estop_active_` before TTC evaluation.
- Projected closing speeds at or below `1e-3` are skipped.
- Once a TTC stop reduces odometry to zero, the next scan clears the stop
  without obstacle-clearance hysteresis or operator reset.
- The next positive `/drive_raw` command can enter the same obstacle. This was
  reproduced against the installed `/odom` build with a `0.1 m` forward range.
- Raw range is not converted to footprint clearance, and there is no static
  minimum-clearance condition.
- The checked-in TTC threshold is only 0.3 seconds
  (`src/reactive/src/safety_node.cpp:11`); the stale installed executable used
  30.0 seconds and still exhibited the zero-speed release.
- The node publishes a zero Ackermann speed
  (`src/reactive/src/safety_node.cpp:83-88`); it does not command or verify a
  braking-current path.
- There is no odometry freshness check; missing odometry leaves TTC disabled.

Impact: correcting `/odom` can make TTC trigger during motion but cannot keep
the car stopped at the wall. Repeated re-acceleration directly explains the
reported collision despite the topic change.

Required direction: latch the stop until every footprint-relevant beam exceeds
a conservative release distance for a defined interval (or require operator
reset), add a hard static body-clearance stop independent of speed, and verify
the complete stopping envelope at every permitted speed.

### A-10 — Medium — The repository has no functional regression tests

Evidence:

- No `test/` source files exist in this repository.
- The gap geometry, polynomial path, speed curve, deadman gate, TTC, topic
  wiring, malformed input, and actuator clamps have no automated behavioral
  coverage.
- The configured lint suite does not fully pass:
  - `reactive`: 3 of 6 lint tests failed (`flake8`, `lint_cmake`,
    `uncrustify`);
  - `ftg_node`: 2 of 6 failed (`flake8`, `uncrustify`);
  - `disparity_extender`: no tests were configured.
- The Jazzy build emitted signed/unsigned comparison warnings in
  `gap_finder_node.cpp:47` and `gap_follow_node.cpp:74`.

Impact: safety regressions and integration mistakes can merge without a
behavioral signal.

Required direction: extract ROS-free control math for unit tests and add ROS
integration tests for topic/QoS/deadman/TTC behavior before physical use.

### A-11 — Medium — Two installed packages are skeletons, not usable controllers

Evidence:

- `ftg_node` installs and launches an executable, but its LaserScan callback is
  empty (`src/ftg_node/src/ftg_node.cpp:14-16`). It advertises `drive` but never
  commands it.
- `disparity_extender` contains only build metadata and installs no executable.
- `ftg_node/package.xml` still contains TODO description/license fields.

Impact: operators can successfully build or launch packages that do no driving,
which complicates race-day diagnosis and makes package readiness unclear.

Required direction: clearly label skeleton packages as non-operational or
remove them from operator-facing workflows until implemented and tested.

## Positive observations

- All four audited packages (`reactive`, `ftg_node`,
  `disparity_extender`, and Racerbot B's separately located
  `gap_follow_node`) compiled together on the installed ROS 2 Jazzy toolchain.
- `rosdep check` reports all Racerbot A system dependencies satisfied.
- The default `reactive_launch.py` composition remaps controller output to
  `/drive_raw`, enables the safety node's deadman gate by default, checks LB at
  button index 4, and expires Joy input after 0.5 seconds.
- The final command path uses `AckermannDriveStamped` and `/drive`, so it enters
  `ackermann_mux` rather than bypassing the mux or publishing low-level VESC
  commands.
- Published commands are timestamped.
- When the gap finder emits no new message, the callback-driven control stream
  goes quiet and the workspace mux's 0.2-second navigation timeout provides a
  downstream stale-command stop.

These strengths are useful foundations, but they do not offset the critical
findings above.

## Verification performed

All runtime checks used isolated ROS domain IDs and only synthetic messages.
The hardware bringup, VESC, and motors were not started.

| Check | Result |
|---|---|
| ROS 2 Jazzy isolated build | Pass; all audited packages built |
| `rosdep check --from-paths src --ignore-src` | Pass |
| Configured lint tests | Fail; details in A-10 |
| Functional/unit tests | None present |
| Direct controller with no `/joy` | Published 4.326356 m/s |
| Checked-in safety source topic | `/ego_racecar/odom`; incorrect for car |
| Installed safety executable topic | `/odom`; differs from source |
| Installed effective TTC threshold | `30.0 s`; differs from source `0.3 s` |
| Zero speed, 0.1 m obstacle, positive raw command | Positive command forwarded |
| 2.0 m/s, 1.0 m obstacle | TTC stop triggered and zero published |
| Comparison gap-logic tests | Pass; 29 tests |
| Uniform open full-circle scan to gap finder | No valid gap produced |
| Physical-car test | Not performed; unsafe at this audit state |

## Minimum exit criteria before physical testing

1. Every motion-producing executable independently enforces a fresh LB hold.
2. The reviewed source is rebuilt and shown to match the installed executable,
   topic graph, and effective parameters.
3. TTC consumes fresh `/odom`, validates direction/sign, and fails closed when
   odometry is missing, stale, or implausible.
4. A footprint-aware static-clearance stop remains latched at zero speed until
   a conservative release condition or operator reset.
5. Stop, hold, and release tests reproduce the drive/stop/drive sequence found
   in this follow-up.
6. Speed and steering are explicitly clamped to conservative car-specific
   values.
7. LaserScan and `Gap` data are validated, including NaN/Inf/empty/malformed
   cases.
8. Gap selection and swept-path checks maintain body clearance using the car's
   measured geometry; gap-edge fallback is removed.
9. Unit and ROS integration tests cover the findings above and the configured
   test suite passes.
10. Only then: static topic inspection, wheels off the ground, and low-speed
   floor testing in an open area with an operator at the power disconnect.

