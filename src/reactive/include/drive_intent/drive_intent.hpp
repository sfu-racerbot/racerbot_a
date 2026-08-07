// drive_intent.hpp -- single-header C++ port of the /drive_intent schema.
//
// Drop this file into a C++ driving node (racerbot_a, racerbot_b, or
// anything else on this car's bus) and it can publish the same intent
// messages this workspace's Python nodes do, so the web dashboard draws
// the same arrow and the same decision panel for it. See
// docs/drive-intent.md for the porting guide and the field reference.
//
// Dependencies: the C++17 standard library and std_msgs/msg/string.hpp.
// Deliberately nothing else -- not nlohmann/json, not a rosidl interface
// package. racerbot_a and racerbot_b are separate repositories that also
// build in their own workspaces, and a shared message package would make
// all three rebuild together for a diagnostics feature.
//
// This is a line-for-line translation of drive_intent/predict.py and
// drive_intent/schema.py. When you change one, change the other; the
// Python side has the unit tests (src/drive_intent/test/).
//
// SAFETY CONTRACT -- this runs inside a node that steers a physical car:
//
//   1. Call publish() only AFTER the drive command for this tick has
//      already gone out. Nothing here may sit in front of a stop command.
//   2. Wrap the call in try/catch. A diagnostic drawing must never take
//      down the node holding the car's steering. FailureLatch below turns
//      a run of failures into "intent switches itself off", not "the node
//      dies".
//   3. Read only values the control path already computed. Do not let the
//      control path read anything back out of here.
//
// Minimal usage:
//
//   #include "drive_intent/drive_intent.hpp"
//   ...
//   intent_pub_ = create_publisher<std_msgs::msg::String>("/drive_intent", 10);
//   drive_intent::IntentThrottle throttle(20.0, 1.0);
//   drive_intent::FailureLatch latch;
//   ...
//   drive_pub_->publish(drive_msg);            // rule 1: command first
//   const double now = this->now().seconds();
//   if (!latch.disabled() && throttle.should_publish(now)) {
//     try {
//       drive_intent::Intent intent;
//       intent.node = "gap_follow_node";
//       intent.state = "gap_follow";
//       intent.commanded_speed = speed;
//       intent.desired_speed = desired_speed;
//       intent.commanded_steering = steering;
//       intent.desired_steering = desired_steering;
//       intent.horizon_s = 1.5;
//       intent.path = drive_intent::constant_arc(
//           desired_steering, desired_speed, wheelbase_, 1.5, 16, 8.0);
//       intent.commanded_path = drive_intent::constant_arc(
//           steering, speed, wheelbase_, 1.5, 16, 8.0);
//       intent.factors = drive_intent::bind_min({
//           {"curve cap", curve_cap}, {"clearance cap", clearance_cap}});
//       intent.severity = drive_intent::classify_severity(intent.state, speed);
//       if (throttle.wants_reason(now, intent.state)) {
//         intent.set_reason(reason_string);      // only when it changed
//       }
//       std_msgs::msg::String msg;
//       msg.data = drive_intent::encode(intent);
//       intent_pub_->publish(msg);
//       latch.record_success();
//     } catch (const std::exception & e) {
//       if (latch.record_failure()) {
//         RCLCPP_ERROR(get_logger(), "drive intent off: %s", e.what());
//       }
//     }
//   }

#ifndef DRIVE_INTENT__DRIVE_INTENT_HPP_
#define DRIVE_INTENT__DRIVE_INTENT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace drive_intent
{

// Bump only for a breaking change, and in lockstep with
// drive_intent/schema.py. Consumers reject versions they do not know.
constexpr int kSchemaVersion = 1;

constexpr double kStraightCurvatureEps = 1e-9;
constexpr std::size_t kMaxPathPoints = 512;
constexpr std::size_t kMaxReasonChars = 2000;

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------

struct PathPoint
{
  double x{0.0};   // meters forward of base_link
  double y{0.0};   // meters left of base_link
  double v{0.0};   // planned speed at this point, m/s
};

struct Factor
{
  std::string name;
  double value{0.0};
  std::string unit{"m/s"};
  bool binding{false};
};

struct Target
{
  std::string kind;
  double x{0.0};
  double y{0.0};
};

struct Wedge
{
  double x{0.0};
  double y{0.0};
  double a0{0.0};
  double a1{0.0};
  double r{0.0};
};

struct Intent
{
  std::string node;
  std::string state;
  std::string severity{"caution"};
  std::string frame{"base_link"};
  double stamp{0.0};             // seconds; 0 means "fill in at encode time"
  double horizon_s{0.0};
  double desired_steering{0.0};
  double commanded_steering{0.0};
  double desired_speed{0.0};
  double commanded_speed{0.0};
  std::vector<PathPoint> path;             // what the algorithm wants
  std::vector<PathPoint> commanded_path;   // what the command will produce
  std::vector<Factor> factors;
  std::vector<Target> targets;

  bool has_reason{false};
  std::string reason;

  bool has_wedge{false};
  Wedge wedge;

  // Attach the human-readable explanation. Call this only on the ticks
  // where IntentThrottle::wants_reason() says so: on a state change, and
  // on the slow period. An explanation that is expensive to compute
  // should not be computed at all on the ticks in between.
  void set_reason(const std::string & text)
  {
    has_reason = true;
    reason = text.size() > kMaxReasonChars ? text.substr(0, kMaxReasonChars) : text;
  }

  void set_wedge(double wx, double wy, double a0, double a1, double r)
  {
    has_wedge = true;
    wedge = Wedge{wx, wy, a0, a1, r};
  }
};

// ---------------------------------------------------------------------------
// Prediction -- mirrors drive_intent/predict.py
// ---------------------------------------------------------------------------

inline double curvature_from_steering(double steering, double wheelbase)
{
  if (!std::isfinite(wheelbase) || wheelbase <= 0.0) {
    throw std::invalid_argument("wheelbase must be finite and positive");
  }
  if (!std::isfinite(steering)) {
    throw std::invalid_argument("steering must be finite");
  }
  return std::tan(steering) / wheelbase;
}

// One exact constant-curvature step. Not an Euler step: over a 1.5s
// horizon at full lock the two disagree by several centimeters, which is
// exactly the scale at which someone is squinting at the arrow to decide
// whether the car will clip a cone.
inline void arc_step(
  double & x, double & y, double & yaw,
  double speed, double steering, double wheelbase, double dt)
{
  const double kappa = curvature_from_steering(steering, wheelbase);
  const double ds = speed * dt;
  if (std::fabs(kappa) < kStraightCurvatureEps) {
    x += ds * std::cos(yaw);
    y += ds * std::sin(yaw);
    return;
  }
  const double dyaw = kappa * ds;
  const double radius = 1.0 / kappa;
  x += radius * (std::sin(yaw + dyaw) - std::sin(yaw));
  y -= radius * (std::cos(yaw + dyaw) - std::cos(yaw));
  yaw += dyaw;
}

// Hold this steering and this speed for the whole horizon. The honest
// model for a reactive controller, which chooses a heading rather than a
// path. If your controller follows a stored line, re-evaluate its
// steering law per step instead (see _intent_path in
// pure_pursuit_node.py) so the arrow bends through the corner ahead.
//
// `max_length_m <= 0` disables the length clamp.
inline std::vector<PathPoint> constant_arc(
  double steering, double speed, double wheelbase,
  double horizon_s, int samples, double max_length_m = 0.0)
{
  if (samples < 2) {
    throw std::invalid_argument("samples must be at least 2");
  }
  if (!std::isfinite(horizon_s) || horizon_s <= 0.0) {
    throw std::invalid_argument("horizon_s must be finite and positive");
  }
  if (!std::isfinite(speed)) {
    throw std::invalid_argument("speed must be finite");
  }

  const double dt = horizon_s / (samples - 1);
  double x = 0.0, y = 0.0, yaw = 0.0, travelled = 0.0;
  std::vector<PathPoint> points;
  points.reserve(static_cast<std::size_t>(samples));

  for (int i = 0; i < samples; ++i) {
    points.push_back(PathPoint{x, y, speed});
    if (i == samples - 1) {break;}
    if (max_length_m > 0.0 && travelled >= max_length_m) {break;}
    arc_step(x, y, yaw, speed, steering, wheelbase, dt);
    travelled += std::fabs(speed) * dt;
  }
  return points;
}

// A LIDAR-frame polar return as a base_link point. The LIDAR on this car
// sits 0.33m ahead of base_link, so a gap target plotted without the
// offset lands most of a car length behind where the controller is aiming.
inline PathPoint polar_to_body(
  double bearing, double distance,
  double sensor_offset_x = 0.0, double sensor_offset_y = 0.0)
{
  return PathPoint{
    sensor_offset_x + distance * std::cos(bearing),
    sensor_offset_y + distance * std::sin(bearing),
    0.0};
}

// ---------------------------------------------------------------------------
// Factors and severity -- mirrors drive_intent/schema.py
// ---------------------------------------------------------------------------

// Mark the smallest-valued factor(s) as binding. Correct for speed
// ceilings specifically -- the command is the min() of them, so the
// smallest is the one in charge, and that is the single most useful thing
// this whole feature reports. Ties are all marked: claiming one of two
// equal limits is "the" reason would be a lie of precision.
inline std::vector<Factor> bind_min(std::vector<Factor> factors, double tol = 1e-6)
{
  if (factors.empty()) {return factors;}
  double lowest = factors.front().value;
  for (const auto & f : factors) {lowest = std::min(lowest, f.value);}
  for (auto & f : factors) {f.binding = std::fabs(f.value - lowest) <= tol;}
  return factors;
}

// Keep this list in sync with NOMINAL_STATES in schema.py: the states in
// which the car is doing the ordinary thing its controller exists to do.
// Everything else -- fallbacks, overtakes, overrides, stops -- is worth an
// operator's attention and is drawn in amber or red.
inline bool is_nominal_state(const std::string & state)
{
  return state == "gap_follow" || state == "pure_pursuit";
}

inline std::string classify_severity(const std::string & state, double speed)
{
  if (speed <= 0.0) {return "stop";}
  return is_nominal_state(state) ? "drive" : "caution";
}

// ---------------------------------------------------------------------------
// Throttling and failure containment -- mirrors drive_intent/throttle.py
// ---------------------------------------------------------------------------

class IntentThrottle
{
public:
  IntentThrottle(double rate_hz, double reason_period_sec)
  : rate_hz_(rate_hz), reason_period_sec_(reason_period_sec) {}

  double min_period() const {return rate_hz_ <= 0.0 ? 0.0 : 1.0 / rate_hz_;}

  // True at most rate_hz times per second. A non-positive rate means
  // "every tick", not "never" -- switching intent off is a parameter, and
  // a rate of 0 silently publishing nothing would be a trap.
  bool should_publish(double now)
  {
    if (have_published_ && now - last_publish_ < min_period()) {return false;}
    have_published_ = true;
    last_publish_ = now;
    return true;
  }

  // True on every state transition, plus every reason_period_sec. The
  // transition is the diagnostic event -- it is the moment someone is
  // asking "why did it just do that?" -- so it always carries its reason.
  bool wants_reason(double now, const std::string & state)
  {
    const bool changed = !have_state_ || state != last_state_;
    const bool elapsed = reason_period_sec_ > 0.0 &&
      (!have_reason_time_ || now - last_reason_time_ >= reason_period_sec_);
    if (!changed && !elapsed) {return false;}
    have_state_ = true;
    last_state_ = state;
    have_reason_time_ = true;
    last_reason_time_ = now;
    return true;
  }

  // Forget all history, so the next tick publishes and explains. Worth
  // calling when a browser connects: whoever just opened the page has no
  // context, and making them wait a second for the first reason is the
  // wrong default.
  void reset()
  {
    have_published_ = false;
    have_reason_time_ = false;
    have_state_ = false;
  }

private:
  double rate_hz_;
  double reason_period_sec_;
  double last_publish_{0.0};
  double last_reason_time_{0.0};
  std::string last_state_;
  bool have_published_{false};
  bool have_reason_time_{false};
  bool have_state_{false};
};

// Disable a non-essential subsystem after it fails repeatedly. The
// alternative -- letting an exception out of intent generation -- would
// take down a node holding a moving car's steering, to protect a drawing.
class FailureLatch
{
public:
  explicit FailureLatch(int max_failures = 5)
  : max_failures_(max_failures)
  {
    if (max_failures < 1) {
      throw std::invalid_argument("max_failures must be at least 1");
    }
  }

  bool disabled() const {return disabled_;}

  // Returns true only on the call that trips the latch, so the "intent is
  // now off" message is logged exactly once.
  bool record_failure()
  {
    if (disabled_) {return false;}
    if (++consecutive_ >= max_failures_) {
      disabled_ = true;
      return true;
    }
    return false;
  }

  // A working call clears the count. The latch is for sustained breakage,
  // not one bad scan in a thousand. It does not re-enable a tripped latch:
  // diagnostics that flap back on under load are worse than silent ones.
  void record_success() {consecutive_ = 0;}

private:
  int max_failures_;
  int consecutive_{0};
  bool disabled_{false};
};

// ---------------------------------------------------------------------------
// JSON encoding
// ---------------------------------------------------------------------------

namespace detail
{

// snprintf honours LC_NUMERIC, and a process that has called
// setlocale(LC_ALL, "") in a de_DE/fr_FR environment will happily write
// "1,234" -- which is not a number in JSON, and which would break the
// browser's parse of the whole message rather than one field. rclcpp does
// not touch the locale, but a teammate's main() might, so the decimal
// comma is corrected rather than assumed away.
inline std::string num(double value, int places)
{
  if (!std::isfinite(value)) {
    // Emitting a bare NaN here would produce invalid JSON and break the
    // dashboard's socket, not just this one arrow. Fail on the car.
    throw std::invalid_argument("intent contains a non-finite number");
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", places, value);
  std::string out(buffer);
  for (char & c : out) {
    if (c == ',') {c = '.';}
  }
  return out;
}

inline std::string quote(const std::string & text)
{
  std::string out;
  out.reserve(text.size() + 2);
  out.push_back('"');
  for (const unsigned char c : text) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char esc[8];
          std::snprintf(esc, sizeof(esc), "\\u%04x", c);
          out += esc;
        } else {
          out.push_back(static_cast<char>(c));
        }
    }
  }
  out.push_back('"');
  return out;
}

inline std::string points(const std::vector<PathPoint> & pts)
{
  std::string out = "[";
  const std::size_t n = std::min(pts.size(), kMaxPathPoints);
  for (std::size_t i = 0; i < n; ++i) {
    if (i) {out += ',';}
    out += "{\"x\":" + num(pts[i].x, 3) +
      ",\"y\":" + num(pts[i].y, 3) +
      ",\"v\":" + num(pts[i].v, 2) + "}";
  }
  return out + "]";
}

}  // namespace detail

// Serialize an Intent to the /drive_intent wire format. Throws
// std::invalid_argument if any value is non-finite -- catch it (rule 2 of
// the safety contract) and skip the message.
inline std::string encode(const Intent & intent)
{
  std::string out = "{\"v\":";
  out += std::to_string(kSchemaVersion);
  out += ",\"stamp\":" + detail::num(intent.stamp, 3);
  out += ",\"node\":" + detail::quote(intent.node);
  out += ",\"frame\":" + detail::quote(intent.frame);
  out += ",\"state\":" + detail::quote(intent.state);
  out += ",\"severity\":" + detail::quote(intent.severity);
  out += ",\"horizon_s\":" + detail::num(intent.horizon_s, 3);
  out += ",\"desired_steering\":" + detail::num(intent.desired_steering, 4);
  out += ",\"commanded_steering\":" + detail::num(intent.commanded_steering, 4);
  out += ",\"desired_speed\":" + detail::num(intent.desired_speed, 3);
  out += ",\"commanded_speed\":" + detail::num(intent.commanded_speed, 3);
  out += ",\"path\":" + detail::points(intent.path);
  out += ",\"commanded_path\":" + detail::points(intent.commanded_path);

  out += ",\"factors\":[";
  for (std::size_t i = 0; i < intent.factors.size(); ++i) {
    if (i) {out += ',';}
    const Factor & f = intent.factors[i];
    out += "{\"name\":" + detail::quote(f.name) +
      ",\"value\":" + detail::num(f.value, 3) +
      ",\"unit\":" + detail::quote(f.unit) +
      ",\"binding\":" + (f.binding ? "true" : "false") + "}";
  }
  out += "]";

  out += ",\"targets\":[";
  for (std::size_t i = 0; i < intent.targets.size(); ++i) {
    if (i) {out += ',';}
    const Target & t = intent.targets[i];
    out += "{\"kind\":" + detail::quote(t.kind) +
      ",\"x\":" + detail::num(t.x, 3) +
      ",\"y\":" + detail::num(t.y, 3) + "}";
  }
  out += "]";

  if (intent.has_reason) {
    out += ",\"reason\":" + detail::quote(intent.reason);
  }
  if (intent.has_wedge) {
    out += ",\"wedge\":{\"x\":" + detail::num(intent.wedge.x, 3) +
      ",\"y\":" + detail::num(intent.wedge.y, 3) +
      ",\"a0\":" + detail::num(intent.wedge.a0, 4) +
      ",\"a1\":" + detail::num(intent.wedge.a1, 4) +
      ",\"r\":" + detail::num(intent.wedge.r, 3) + "}";
  }
  return out + "}";
}

}  // namespace drive_intent

#endif  // DRIVE_INTENT__DRIVE_INTENT_HPP_
