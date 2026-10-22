#pragma once

#include "geometry_msgs/msg/twist.hpp"

namespace omni_mulinex_joystick {

struct FigureEightParams {
    double size = 1.0;        // half-length of the eight along its long axis [m]
    double max_lin_vel = 0.2; // peak linear speed along the path [m/s]
    double max_ang_vel = 0.5; // peak yaw rate (unicycle mode) [rad/s]
    int laps = 1;             // number of complete eights
    double ramp_time = 3.0;   // smooth speed ramp at start and end [s]
    bool holonomic = false;   // true: vx/vy with fixed heading; false: vx + wz (differential)
};

// Open-loop figure-eight (lemniscate of Gerono) expressed as a body twist.
//
//   x(φ) = A·sin(φ),   y(φ) = A/2·sin(2φ),   φ ∈ [0, 2π·laps]
//
// The eight's long axis is aligned with the robot's initial heading, centred on the
// start position. The phase φ follows a cosine ramp-up / cruise / cosine ramp-down
// profile, so the twist starts and ends at zero. The cruise phase rate is the largest
// one that keeps the linear speed (and, in unicycle mode, the yaw rate) within limits.
//
// Unicycle mode: the path tangent at the crossing point is not along the long axis, so
// the robot first turns in place onto it, runs the eight, then turns back to its
// initial heading.
class FigureEight {
public:
    explicit FigureEight(const FigureEightParams& params);

    /// Twist to command at time t [s] since start (zero once finished).
    geometry_msgs::msg::Twist twist_at(double t) const;
    bool finished(double t) const { return t >= duration_; }
    double duration() const { return duration_; }

private:
    double phase(double t) const;
    double phase_rate(double t) const;
    geometry_msgs::msg::Twist eight_twist_at(double t) const;

    FigureEightParams p_;
    double rate_ = 0.0;     // cruise phase rate [rad/s]
    double ramp_ = 0.0;     // ramp duration [s]
    double cruise_ = 0.0;   // cruise duration [s]
    double eight_duration_ = 0.0; // duration of the eight itself [s]
    double align_angle_ = 0.0;    // in-place turn before / after the eight [rad]
    double align_time_ = 0.0;     // duration of each in-place turn [s]
    double duration_ = 0.0;       // total duration [s]
};

} // namespace omni_mulinex_joystick
