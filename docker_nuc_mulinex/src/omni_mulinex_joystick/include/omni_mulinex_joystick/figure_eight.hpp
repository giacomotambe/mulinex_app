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
// The phase φ follows a cosine ramp-up / cruise / cosine ramp-down profile, so the
// twist starts and ends at zero. The cruise phase rate is the largest one that keeps
// the linear speed (and, in unicycle mode, the yaw rate) within the given limits.
// The robot starts at the crossing point of the eight, heading along the first lobe.
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

    FigureEightParams p_;
    double rate_ = 0.0;     // cruise phase rate [rad/s]
    double ramp_ = 0.0;     // ramp duration [s]
    double cruise_ = 0.0;   // cruise duration [s]
    double duration_ = 0.0; // total duration [s]
};

} // namespace omni_mulinex_joystick
