# Publishing drive intent from racerbot_a

This guide is for adding `/drive_intent` to **this** repo's nodes. It is
written against the code as it stands at commit `a7345d7` — file and
function names below are real, not placeholders.

## What this gets you

The car's [web dashboard](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace)
(`web_dashboard` in the shared workspace) draws a curved arrow ahead of
the car showing where the driving algorithm **intends** to go, plus a
panel explaining **why** it is making the current decision.

- **Length** = distance the plan covers over the prediction horizon
- **Width** = planned speed at that point along the arrow
- **Curve** = the actual shape of the plan, not a straight tangent
- **Panel** = decision state, the reason sentence, and which speed limit
  is *actually* holding the car back right now

It is a display, not a control path. Nothing subscribes to
`/drive_intent` except the dashboard.

The full schema reference lives in the shared workspace at
[`docs/drive-intent.md`](https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace/blob/main/docs/drive-intent.md).
This file is the racerbot_a-specific part.

## The header is already here

`src/reactive/include/drive_intent/drive_intent.hpp` is vendored into
this repo. It is a self-contained C++17 header — standard library plus
`std_msgs`, nothing else. No dependency on the shared workspace, no
`rosidl` interface package, no `nlohmann/json`.

It is vendored rather than shared on purpose: this repo builds in its own
workspace as well as inside the car's, and a shared message package would
have coupled all three repos' rebuilds together for a diagnostics
feature. The cost is that if the schema ever changes, the header gets
re-copied. `v` in the payload is the version handshake — the dashboard
refuses a version it does not know rather than half-drawing it.

`reactive/CMakeLists.txt` already does `include_directories(include)`, so
`#include "drive_intent/drive_intent.hpp"` compiles with **no CMake
change at all**. The header is clean under this repo's
`-Wall -Wextra -Wpedantic`.

## Safety rules — these are not optional

`/drive_intent` is diagnostics running inside a node that steers a
physical car. Three rules:

1. **Publish intent only *after* `drive_pub_->publish(drive_msg)`.**
   Nothing in the intent path may sit in front of a drive command, and
   especially not in front of a stop.
2. **Wrap it all in one `try`/`catch`, and never rethrow.** An exception
   from a drawing must not propagate out of a subscription callback and
   take down the node holding the car's steering. `FailureLatch` in the
   header turns repeated failures into "intent switches itself off",
   logged once, with the node still driving.
3. **Read only what the control path already computed.** If the control
   path ever starts reading something the intent code produced, it is no
   longer diagnostics.

Plus: throttle it (~20 Hz, via `IntentThrottle`), and never emit a
non-finite number — `encode()` throws on NaN/inf rather than writing a
bare `NaN`, which is not valid JSON and would break the dashboard's whole
message stream, not just one arrow.

Even though this adds no arithmetic to the drive path, it edits files
that can move the car. Test in the usual order: static topic check with
no driver stack running → wheels off the ground → floor, low speed, open
space.

## Step 1: add `std_msgs` to the `reactive` package

`reactive` does not currently depend on `std_msgs`. Two small edits.

`src/reactive/package.xml`:

```diff
   <depend>rclcpp</depend>
   <depend>ackermann_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>nav_msgs</depend>
+  <depend>std_msgs</depend>
   <depend>eigen</depend>
```

`src/reactive/CMakeLists.txt`:

```diff
 find_package(nav_msgs REQUIRED)
+find_package(std_msgs REQUIRED)
 find_package(rosidl_default_generators REQUIRED)
```
```diff
 add_executable(gap_follow_node src/gap_follow_node.cpp)
-ament_target_dependencies(gap_follow_node rclcpp ackermann_msgs sensor_msgs nav_msgs Eigen3)
+ament_target_dependencies(gap_follow_node rclcpp ackermann_msgs sensor_msgs nav_msgs std_msgs Eigen3)
 target_link_libraries(gap_follow_node "${cpp_typesupport_target}" Eigen3::Eigen)
```

Do the same for `safety_node` if you add intent there too.

## Step 2: `gap_follow_node` — the members

In `src/reactive/include/gap_follow_node.hpp`:

```cpp
#include "drive_intent/drive_intent.hpp"
#include <std_msgs/msg/string.hpp>
```

and in the class body:

```cpp
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intent_pub_;
    // 20Hz publish, 1s between repeated reason strings. Both independent
    // of the gap callback rate -- no browser needs more, and the Jetson
    // is also driving the car.
    drive_intent::IntentThrottle intent_throttle_{20.0, 1.0};
    drive_intent::FailureLatch intent_latch_;
    double wheelbase_{0.33};   // this car; see the workspace hardware reference
```

In the constructor, next to `drive_pub_`:

```cpp
    intent_pub_ = this->create_publisher<std_msgs::msg::String>("/drive_intent", 10);
```

## Step 3: `drive_best_point()` — the simple case

This method already picks a speed from `target_range` in four bands and
steers straight at `gap_msg->target_angle`. That is a *heading* choice,
not a path, so one constant-curvature arc is the honest prediction —
exactly like the shared workspace's own `gap_follow`.

Append this **after** the existing `drive_pub_->publish(drive_msg);`:

```cpp
    publish_intent(
        "gap_follow",
        gap_msg->target_angle,          // desired steering
        velocity,                       // desired speed
        gap_msg->target_angle,           // commanded (no shaping here)
        velocity,
        // The four range bands are the whole speed law, so naming which
        // band fired *is* the explanation. Only one is ever active, so it
        // is marked directly rather than by bind_min().
        {{"range band", velocity, "m/s", true},
         {"target range", gap_msg->target_range, "m", false}},
        gap_msg);
```

## Step 4: `least_squares_pathfinding()` — the good one

This is the most interesting intent arrow of any node across the three
codebases, and it costs almost no new maths.

`least_squares_pathfinding()` already **fits a polynomial through the gap
points**. That polynomial *is* the intended path — so instead of
approximating it with an arc, sample it directly. The dashboard will draw
the actual curve the fit produced, which means a bad fit becomes visible
as a bad arrow *before* the car drives it.

Append after the existing `drive_pub_->publish(drive_msg);`:

```cpp
    // Sample the fitted polynomial itself as the intended path. y = f(x)
    // in the car's body frame, which is exactly the frame the schema
    // wants, so no transform is needed.
    std::vector<drive_intent::PathPoint> fitted;
    const int kSamples = 16;
    for (int i = 0; i < kSamples; ++i) {
        const double px = lookahead * (static_cast<double>(i) / (kSamples - 1));
        double py = 0.0;
        for (int c = 0; c < coefficients.size(); ++c) {
            py += coefficients(c) * std::pow(px, c);
        }
        fitted.push_back({px, py, velocity});
    }

    publish_intent_path(
        "least_squares",
        steering_angle, velocity, steering_angle, velocity,
        fitted,
        {{"angle taper", velocity, "m/s", true}},
        gap_msg);
```

(Match the polynomial evaluation to whatever convention `fit_polynomial`
returns — the loop above assumes `coefficients(c)` multiplies `x^c`.)

Note the state string differs from `drive_best_point`'s. That is
deliberate: the dashboard's transition log will then show you exactly
when the node fell back from the least-squares path to the best-point
heuristic, which is currently invisible unless you are watching the
terminal for the `RCLCPP_WARN`.

## Step 5: the shared helper

Add this to `gap_follow_node.cpp`. Both call sites above go through it,
so the safety rules live in exactly one place:

```cpp
void GapFollowNode::publish_intent(
    const std::string & state,
    double desired_steering, double desired_speed,
    double commanded_steering, double commanded_speed,
    std::vector<drive_intent::Factor> factors,
    const reactive::msg::Gap::ConstSharedPtr & gap_msg)
{
    // A heading choice, not a path: one arc is the honest prediction.
    auto path = drive_intent::constant_arc(
        desired_steering, desired_speed, wheelbase_, 1.5, 16, 8.0);
    publish_intent_path(state, desired_steering, desired_speed,
                        commanded_steering, commanded_speed,
                        path, std::move(factors), gap_msg);
}

void GapFollowNode::publish_intent_path(
    const std::string & state,
    double desired_steering, double desired_speed,
    double commanded_steering, double commanded_speed,
    const std::vector<drive_intent::PathPoint> & path,
    std::vector<drive_intent::Factor> factors,
    const reactive::msg::Gap::ConstSharedPtr & gap_msg)
{
    // Rule 2: nothing below may escape into gap_callback.
    if (intent_latch_.disabled()) {return;}
    const double now = this->now().seconds();
    if (!intent_throttle_.should_publish(now)) {return;}

    try {
        drive_intent::Intent intent;
        intent.node = "racerbot_a_gap_follow";
        intent.state = state;
        intent.stamp = now;
        intent.horizon_s = 1.5;
        intent.desired_steering = desired_steering;
        intent.commanded_steering = commanded_steering;
        intent.desired_speed = desired_speed;
        intent.commanded_speed = commanded_speed;
        intent.severity = drive_intent::classify_severity(state, commanded_speed);
        intent.path = path;
        // No slew/acceleration shaping in this node yet, so the ghost is
        // the same as the plan. When shaping is added, build this from the
        // *shaped* command and the dashed line starts telling you
        // something the solid one cannot.
        intent.commanded_path = drive_intent::constant_arc(
            commanded_steering, commanded_speed, wheelbase_, 1.5, 16, 8.0);
        intent.factors = std::move(factors);

        // Gap.msg carries everything needed to show *which* gap was
        // picked, not just where the car ended up aiming.
        const auto target = drive_intent::polar_to_body(
            gap_msg->target_angle, gap_msg->target_range, 0.33, 0.0);
        intent.targets.push_back({"gap_target", target.x, target.y});
        if (!gap_msg->angles.empty()) {
            intent.set_wedge(0.33, 0.0,
                             gap_msg->angles.front(), gap_msg->angles.back(),
                             gap_msg->target_range);
        }

        // Only attach the reason on transitions and once a second: it is
        // the expensive field, and the browser holds the last one it saw.
        if (intent_throttle_.wants_reason(now, state)) {
            char reason[400];
            std::snprintf(reason, sizeof(reason),
                          "gap target %.2fm at %+.1fdeg over %zu gap points; "
                          "commanding %.2fm/s at %+.1fdeg",
                          gap_msg->target_range,
                          gap_msg->target_angle * 180.0 / M_PI,
                          gap_msg->ranges.size(),
                          commanded_speed,
                          commanded_steering * 180.0 / M_PI);
            intent.set_reason(reason);
        }

        std_msgs::msg::String msg;
        msg.data = drive_intent::encode(intent);
        intent_pub_->publish(msg);
        intent_latch_.record_success();
    } catch (const std::exception & e) {
        if (intent_latch_.record_failure()) {
            RCLCPP_ERROR(this->get_logger(),
                         "drive intent failed repeatedly (%s); switching it off "
                         "for this run. Driving is unaffected.", e.what());
        }
    }
}
```

Declare both in `gap_follow_node.hpp`.

## Step 6: `safety_node`

If `safety_node` can zero the command, publish a `stop` intent from it
naming *why* — `"emergency_brake"`, `"ttc_brake"`, whatever it actually
is. Set `commanded_speed = 0.0` and leave `path` empty;
`classify_severity` will return `stop`, and the dashboard draws a stop
marker instead of an arrow.

This is the single highest-value place to add intent in this repo,
because a stop you did not expect is the thing you most want explained,
and right now it is entirely silent from outside the node.

## `ftg_node` — nothing to do yet

`FollowTheGapNode::lidar_callback` in `src/ftg_node/src/ftg_node.cpp` is
currently an empty stub — it computes nothing and publishes nothing, so
there is no intent to report. Add intent when the callback is
implemented, following the `drive_best_point` pattern above.

## Verifying it

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /drive_intent --once
```

You should see one line of JSON with `"node":"racerbot_a_gap_follow"`.
Then open the dashboard (`ros2 launch web_dashboard web_dashboard_launch.py`,
then `http://<car-ip>:8080/`) and the arrow should appear ahead of the
car with your node's name in the intent panel.

If the dashboard logs `ignoring malformed message on '/drive_intent'`, it
tells you exactly which field it rejected.

### Checklist before merging

- [ ] `intent_pub_->publish(...)` is strictly below `drive_pub_->publish(...)`
- [ ] Everything is inside `try`/`catch`, and the catch never rethrows
- [ ] `FailureLatch` disables intent after repeated failures
- [ ] Publishing is throttled, not once per callback
- [ ] `severity` comes from `classify_severity(state, commanded_speed)`
- [ ] Nothing in the control path reads anything the intent code wrote
- [ ] Static topic check → wheels off the ground → floor
