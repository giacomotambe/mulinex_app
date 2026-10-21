#include "omni_mulinex_joystick/identification.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace omni_mulinex_joystick {

IdentificationMission::IdentificationMission(
    const std::string& mission, const IdentificationParams& params
): p_(params), name_(mission)
{
    if (mission == "t1")
        build_t1();
    else if (mission == "t2")
        build_t2();
    else if (mission == "t3")
        build_t3();
    else
        throw std::invalid_argument(
            "unknown identification mission '" + mission + "' (t1, t2, t3)"
        );

    for (const auto& s : segments_) {
        starts_.push_back(duration_);
        duration_ += s.duration;
    }
    check_limits();
}

void IdentificationMission::check_limits() const
{
    // Sample the whole mission: shoulder range and track joint speed (belt speed / r_s + assist)
    for (double t = 0.0; t <= duration_; t += 0.005) {
        const auto sp = at(t);
        for (double a : {sp.alpha_front, sp.alpha_hind}) {
            if (a < p_.alpha_min || a > p_.alpha_max)
                throw std::invalid_argument(
                    name_ + ": shoulder angle " + std::to_string(a) + " rad at t " + std::to_string(t) +
                    " s outside [" + std::to_string(p_.alpha_min) + ", " + std::to_string(p_.alpha_max) + "]"
                );
        }
        const double w = std::max(std::abs(sp.v_left), std::abs(sp.v_right)) / p_.sprocket_radius +
                         p_.assist_gain * std::max(std::abs(sp.rate_front), std::abs(sp.rate_hind));
        if (w > p_.max_track_speed + 1e-9)
            throw std::invalid_argument(
                name_ + ": track joint speed " + std::to_string(w) + " rad/s at t " + std::to_string(t) +
                " s above " + std::to_string(p_.max_track_speed)
            );
    }
}

size_t IdentificationMission::segment_at(double t) const
{
    auto it = std::upper_bound(starts_.begin(), starts_.end(), t);
    return it == starts_.begin() ? 0 : static_cast<size_t>(it - starts_.begin()) - 1;
}

IdentSetpoint IdentificationMission::at(double t) const
{
    if (segments_.empty())
        return {};
    if (t >= duration_) {
        // after the end: last pose, tracks still
        auto sp = segments_.back().f(segments_.back().duration, duration_);
        sp.v_left = sp.v_right = 0.0;
        return sp;
    }
    const size_t i = segment_at(t);
    const auto& seg = segments_[i];
    const double tau = t - starts_[i];
    // shoulder rates by finite difference of the profile (feedforward and track assist)
    constexpr double dt = 1e-3;
    auto sp = seg.f(tau, t);
    auto next = seg.f(tau + dt, t + dt);
    sp.rate_front = (next.alpha_front - sp.alpha_front) / dt;
    sp.rate_hind = (next.alpha_hind - sp.alpha_hind) / dt;
    // track speeds: cosine ramp from the end speeds of the previous segment
    if (i > 0 && tau < p_.speed_ramp && p_.speed_ramp > 0.0) {
        const auto& prev = segments_[i - 1];
        const auto end = prev.f(prev.duration, starts_[i]);
        const double s = 0.5 * (1.0 - std::cos(M_PI * tau / p_.speed_ramp));
        sp.v_left = end.v_left + s * (sp.v_left - end.v_left);
        sp.v_right = end.v_right + s * (sp.v_right - end.v_right);
    }
    return sp;
}

void IdentificationMission::hold(
    const std::string& name, double dur, double a_front, double a_hind, double v_left,
    double v_right
)
{
    segments_.push_back({name, dur,
                         [=](double, double) {
                             IdentSetpoint sp;
                             sp.alpha_front = a_front;
                             sp.alpha_hind = a_hind;
                             sp.v_left = v_left;
                             sp.v_right = v_right;
                             return sp;
                         },
                         false});
}

void IdentificationMission::move(double dur, double f0, double h0, double f1, double h1)
{
    // cosine move of the shoulders, tracks only with the assist
    segments_.push_back({"move", dur,
                         [=](double tau, double) {
                             const double s = 0.5 * (1.0 - std::cos(M_PI * std::min(tau / dur, 1.0)));
                             IdentSetpoint sp;
                             sp.alpha_front = f0 + s * (f1 - f0);
                             sp.alpha_hind = h0 + s * (h1 - h0);
                             return sp;
                         },
                         true});
}

void IdentificationMission::sync(double a)
{
    const double v = p_.sync_speed;
    hold("stop", 2.0, a, a);
    hold("sync", 1.0, a, a, -v, v);
    hold("sync", 1.0, a, a, v, -v);
    hold("stop", 1.0, a, a);
    hold("sync", 2.0, a, a, -v, v);
    hold("stop", 2.0, a, a);
}

void IdentificationMission::build_t1()
{
    const double st = p_.stand;
    double a = st;
    sync(st);
    for (double al : p_.alphas) {
        move(4.0, a, a, al, al);
        hold("stop", 2.0, al, al);
        for (double w : {2.0, 4.0, 7.0}) {
            const double v = w * p_.sprocket_radius;
            hold("spin", 8.0, al, al, -v, v);
            hold("stop", 2.0, al, al);
            hold("spin", 8.0, al, al, v, -v);
            hold("stop", 2.0, al, al);
        }
        a = al;
    }
    move(4.0, a, a, st, st);
    sync(st);
}

void IdentificationMission::build_t2()
{
    constexpr double v0 = 0.12;
    const double st = p_.stand;
    double a = st;
    sync(st);
    for (double al : p_.alphas) {
        move(4.0, a, a, al, al);
        hold("stop", 2.0, al, al);
        for (double rho : {0.0, 0.3, -0.3, 0.6, -0.6}) {
            const double vl = v0 * (1.0 - rho), vr = v0 * (1.0 + rho);
            const char* name = rho == 0.0 ? "straight" : "arc";
            hold(name, 5.0, al, al, vl, vr);
            hold("stop", 1.5, al, al);
            hold(name, 5.0, al, al, -vl, -vr);
            hold("stop", 1.5, al, al);
        }
        a = al;
    }
    move(4.0, a, a, st, st);
    sync(st);
}

void IdentificationMission::build_t3()
{
    constexpr double mean = 1.45, amp = 0.3, period = 10.0, v0 = 0.12, rho = 0.4;
    auto front = [=](double t) { return mean + amp * std::sin(2.0 * M_PI * t / period); };
    auto hind = [=](double t) { return mean + amp * std::sin(2.0 * M_PI * t / period + M_PI / 2.0); };
    auto drive = [&](const std::string& name, double dur, double vl, double vr) {
        segments_.push_back({name, dur,
                             [=](double, double t) {
                                 IdentSetpoint sp;
                                 sp.alpha_front = front(t);
                                 sp.alpha_hind = hind(t);
                                 sp.v_left = vl;
                                 sp.v_right = vr;
                                 return sp;
                             },
                             true});
    };

    const double st = p_.stand;
    sync(st);
    double t_wave = 5.0;   // the wave starts after the sync and a 5 s move onto it
    for (const auto& s : segments_)
        t_wave += s.duration;
    move(5.0, st, st, front(t_wave), hind(t_wave));
    drive("wave_arc", 12.0, v0 * (1.0 - rho), v0 * (1.0 + rho));
    drive("wave_arc", 12.0, v0 * (1.0 + rho), v0 * (1.0 - rho));
    drive("wave_arc", 12.0, -v0 * (1.0 - rho), -v0 * (1.0 + rho));
    drive("wave_arc", 12.0, -v0 * (1.0 + rho), -v0 * (1.0 - rho));
    drive("wave_spin", 10.0, -v0, v0);
    drive("wave_hold", 5.0, 0.0, 0.0);
    const double t_end = t_wave + 63.0;
    move(5.0, front(t_end), hind(t_end), st, st);
    sync(st);
}

} // namespace omni_mulinex_joystick
