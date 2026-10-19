#include "omni_mulinex_joystick/figure_eight.hpp"

#include <algorithm>
#include <cmath>

namespace omni_mulinex_joystick {

namespace {

// Path derivatives w.r.t. φ for the lemniscate of Gerono
struct PathDerivatives {
    double dx, dy, ddx, ddy;
};

PathDerivatives derivatives(double a, double phi)
{
    return {
        a * std::cos(phi),
        a * std::cos(2.0 * phi),
        -a * std::sin(phi),
        -2.0 * a * std::sin(2.0 * phi),
    };
}

} // namespace

FigureEight::FigureEight(const FigureEightParams& params): p_(params)
{
    p_.laps = std::max(p_.laps, 1);
    ramp_ = std::max(p_.ramp_time, 0.0);

    // Peak speed and yaw rate per unit phase rate over one lap
    double max_speed = 0.0;
    double max_yaw = 0.0;
    constexpr int N = 2000;
    for (int i = 0; i < N; i++) {
        auto d = derivatives(p_.size, 2.0 * M_PI * i / N);
        double speed_sq = d.dx * d.dx + d.dy * d.dy;
        max_speed = std::max(max_speed, std::sqrt(speed_sq));
        max_yaw = std::max(max_yaw, std::abs(d.dx * d.ddy - d.dy * d.ddx) / speed_sq);
    }
    rate_ = p_.max_lin_vel / max_speed;
    if (!p_.holonomic)
        rate_ = std::min(rate_, p_.max_ang_vel / max_yaw);

    // Each cosine ramp covers rate·ramp/2 of phase; shrink the rate if the ramps alone
    // would overshoot the total phase.
    double total = 2.0 * M_PI * p_.laps;
    if (rate_ * ramp_ > total)
        rate_ = total / ramp_;
    cruise_ = total / rate_ - ramp_;
    duration_ = 2.0 * ramp_ + cruise_;
}

double FigureEight::phase_rate(double t) const
{
    if (t < 0.0 || t >= duration_)
        return 0.0;
    if (t < ramp_)
        return rate_ * 0.5 * (1.0 - std::cos(M_PI * t / ramp_));
    if (t < ramp_ + cruise_)
        return rate_;
    double tau = t - ramp_ - cruise_;
    return rate_ * 0.5 * (1.0 + std::cos(M_PI * tau / ramp_));
}

double FigureEight::phase(double t) const
{
    if (t <= 0.0)
        return 0.0;
    if (t >= duration_)
        return 2.0 * M_PI * p_.laps;
    if (t < ramp_)
        return rate_ * (0.5 * t - ramp_ / (2.0 * M_PI) * std::sin(M_PI * t / ramp_));
    if (t < ramp_ + cruise_)
        return rate_ * (0.5 * ramp_ + (t - ramp_));
    double tau = t - ramp_ - cruise_;
    return rate_ *
           (0.5 * ramp_ + cruise_ + 0.5 * tau + ramp_ / (2.0 * M_PI) * std::sin(M_PI * tau / ramp_));
}

geometry_msgs::msg::Twist FigureEight::twist_at(double t) const
{
    geometry_msgs::msg::Twist twist;
    double phi_dot = phase_rate(t);
    if (phi_dot <= 0.0)
        return twist;

    auto d = derivatives(p_.size, phase(t));
    if (p_.holonomic) {
        // Heading stays fixed: the path velocity is the body velocity
        twist.linear.x = d.dx * phi_dot;
        twist.linear.y = d.dy * phi_dot;
    } else {
        // Unicycle: forward speed along the tangent, yaw rate = curvature · speed
        double speed_sq = d.dx * d.dx + d.dy * d.dy;
        twist.linear.x = std::sqrt(speed_sq) * phi_dot;
        twist.angular.z = (d.dx * d.ddy - d.dy * d.ddx) / speed_sq * phi_dot;
    }
    return twist;
}

} // namespace omni_mulinex_joystick
