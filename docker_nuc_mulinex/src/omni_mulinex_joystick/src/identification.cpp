#include "omni_mulinex_joystick/identification.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace omni_mulinex_joystick {

IdentificationMission::IdentificationMission(
    const std::string& mission, const IdentificationParams& params
): p_(params), name_(mission)
{
    // one mission, or several in a row separated by commas ("t1,t2,t3"), with a pause at the stand
    // pose between them (each one starts and ends there with its sync turns)
    std::stringstream list(mission);
    std::string item;
    size_t n = 0;
    while (std::getline(list, item, ',')) {
        item.erase(0, item.find_first_not_of(" \t"));
        item.erase(item.find_last_not_of(" \t") + 1);
        if (n++ > 0)
            hold("pause", p_.pause, p_.stand, p_.stand);
        if (item == "t1")
            build_t1();
        else if (item == "t2")
            build_t2();
        else if (item == "t3")
            build_t3();
        else
            throw std::invalid_argument(
                "unknown identification mission '" + item + "' (t1, t2, t3, or a list as t1,t2,t3)"
            );
    }

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
    return creep_compensated(sp);
}

IdentSetpoint IdentificationMission::segment_target(size_t i) const
{
    const auto& seg = segments_[i];
    return creep_compensated(seg.f(0.5 * seg.duration, starts_[i] + 0.5 * seg.duration));
}

IdentSetpoint IdentificationMission::creep_compensated(IdentSetpoint sp) const
{
    // both belts slowed by the forward creep of the expected yaw rate (see IdentificationParams)
    const double sum = std::abs(sp.v_right + sp.v_left), diff = std::abs(sp.v_right - sp.v_left);
    const double load = sum + diff > 1e-9 ? diff / (sum + diff) : 0.0;
    const double half_diff = std::max(diff / 2.0 - p_.turn_deficit * load * p_.sprocket_radius, 0.0);
    const double xbar = p_.shoulder_x + p_.leg_length * 0.5 * (std::cos(sp.alpha_front) + std::cos(sp.alpha_hind));
    const double ratio = xbar / p_.half_track;
    const double y_eff = p_.half_track * (1.0 + p_.yaw_slip_gain * ratio * ratio);
    const double d = std::max(p_.creep_gain * (xbar - p_.creep_x0), 0.0);
    const double creep = d * half_diff / y_eff;
    sp.v_left -= creep;
    sp.v_right -= creep;
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
    // The robot creeps forward while turning, either way (6-25 cm/s per rad/s of yaw, more with the
    // tracks spread out, Qualisys 2026-10-22): turning one way then the other moves it sideways, about 0.6 m for each shoulder
    // angle. The first direction alternates at every speed, across the shoulder angles too, so these
    // moves cancel (simulated: largest distance from the start about 0.6 m instead of 2.3 m)
    const double st = p_.stand;
    double a = st;
    double first = 1.0;   // +1: counter-clockwise first
    sync(st);
    for (double al : p_.alphas) {
        move(4.0, a, a, al, al);
        hold("stop", 2.0, al, al);
        for (double w : {4.0, 6.0, 8.0}) {   // below about 3 rad/s the tracks stall while turning
            const double v = first * w * p_.sprocket_radius;
            hold("spin", 8.0, al, al, -v, v);
            hold("stop", 2.0, al, al);
            hold("spin", 8.0, al, al, v, -v);
            hold("stop", 2.0, al, al);
            first = -first;
        }
        a = al;
    }
    move(4.0, a, a, st, st);
    sync(st);
}

void IdentificationMission::build_t2()
{
    // 6 rad/s at the sprocket; with rho up to 0.33 the inner track keeps 4 rad/s and the outer 8
    const double v0 = 6.0 * p_.sprocket_radius;
    const double st = p_.stand;
    double a = st;
    sync(st);
    for (double al : p_.alphas) {
        move(4.0, a, a, al, al);
        hold("stop", 2.0, al, al);
        // 2 x 2 m area: legs of 2 s (about 0.4 m), each run there and back twice, forward first
        // then backward first, so the robot comes back to the start and its creep cancels
        for (double rho : {0.0, 0.15, -0.15, 0.33, -0.33}) {
            const double vl = v0 * (1.0 - rho), vr = v0 * (1.0 + rho);
            const char* name = rho == 0.0 ? "straight" : "arc";
            for (double dir : {1.0, -1.0, -1.0, 1.0}) {
                hold(name, 2.0, al, al, dir * vl, dir * vr);
                hold("stop", 1.0, al, al);
            }
        }
        a = al;
    }
    move(4.0, a, a, st, st);
    sync(st);
}

void IdentificationMission::build_t3()
{
    // shoulder wave of 15 s: the assist adds at most ±1.3 rad/s to the tracks, so with 6.5 rad/s and
    // rho 0.12 every track stays within about 4.5-8.5 rad/s
    constexpr double mean = 1.45, amp = 0.3, period = 15.0, rho = 0.12;
    const double v0 = 6.5 * p_.sprocket_radius;
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
    // 2 x 2 m area: legs of 2 s (about 0.45 m) there and back, left and right arcs in turn, over
    // more than two periods of the wave (40 s; the shoulders keep moving at the stops too)
    double t_drive = 0.0;
    for (int k = 0; k < 8; k++) {
        const double s = k % 2 == 0 ? 1.0 : -1.0;   // left / right arc
        for (double dir : {1.0, -1.0}) {
            drive("wave_arc", 2.0, dir * v0 * (1.0 - s * rho), dir * v0 * (1.0 + s * rho));
            drive("wave_hold", 0.5, 0.0, 0.0);
            t_drive += 2.5;
        }
    }
    drive("wave_spin", 10.0, -v0, v0);
    drive("wave_hold", 5.0, 0.0, 0.0);
    const double t_end = t_wave + t_drive + 15.0;
    move(5.0, front(t_end), hind(t_end), st, st);
    sync(st);
}

} // namespace omni_mulinex_joystick
