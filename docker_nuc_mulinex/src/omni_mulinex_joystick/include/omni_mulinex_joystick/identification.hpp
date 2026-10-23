#pragma once

#include <functional>
#include <string>
#include <vector>

namespace omni_mulinex_joystick {

struct IdentificationParams {
    double stand = 1.6;                              // shoulder angle of q_stand (SBC yaml) [rad]
    std::vector<double> alphas{1.05, 1.3, 1.6, 1.85}; // shoulder angles of T1 / T2 [rad]
    double sprocket_radius = 0.045;                  // wheel_rad of the SBC wheel IK [m]
    double assist_gain = 10.0;                       // transition_wheels.gain of the SBC yaml
    double sync_speed = 0.27;                        // belt speed of the sync turns [m/s] (6 rad/s)
    double speed_ramp = 0.5;                         // track speed changes as cosine ramps of this [s] (t1-t3)
    double max_track_speed = 9.0;                    // largest track joint command, assist included [rad/s]
                                                     // (the drives are limited to 10, max_velocity in the SBC xacro)
    double alpha_min = 1.0;                          // shoulder range: tracks flat, ankles off their stops [rad]
    double alpha_max = 1.9;
    double pause = 20.0;                             // stop at the stand pose between missions in a row [s]
    // The robot creeps forward by d·|ωz| while turning (Qualisys 2026-10-22): both tracks are slowed
    // by d·|ωz_expected| so that the missions stay in a 2 x 2 m area. ωz_expected is the yaw rate the
    // tracks give once they lose turn_deficit·load each (load 0 straight, 1 turning in place) through
    // yaw_half_track. 0 disables it (traquad8_dji.yaml traquad.spin_forward_drift)
    double spin_forward_drift = 0.066;               // [m/rad]
    double yaw_half_track = 0.556;                   // [m] traquad.yaw_half_track of the SBC
    double turn_deficit = 2.0;                       // [rad/s] track deficit turning in place
};

// Set point of an identification mission: shoulder angles alpha = sigma·q_HFE (sigma = +1 LF/RH,
// -1 LH/RF), front and hind, the same on both sides, with their rates, and the belt speeds of the
// left and right tracks (forward positive).
struct IdentSetpoint {
    double alpha_front = 0.0;
    double alpha_hind = 0.0;
    double rate_front = 0.0;
    double rate_hind = 0.0;
    double v_left = 0.0;
    double v_right = 0.0;
};

// Open-loop identification missions of the traquad, as fixed sequences of segments:
//
//   t1           turns in place at each shoulder angle, track joint speeds 4, 6, 8 rad/s, both
//                directions, 8 s each (the first direction alternates: the forward creep cancels)
//   t2           straight runs and arcs (track speeds 6·(1 ∓ rho) rad/s, rho = 0, ±0.15, ±0.33) at
//                each shoulder angle, legs of 2 s there and back twice (forward first, then backward)
//   t3           validation: arcs (legs of 2 s there and back, left and right in turn) and a turn while
//                the shoulders move all the time (15 s wave), front and hind a quarter period out of phase
//
// Below about 3 rad/s the tracks stall while turning (speed loop K_d only): every moving track is
// kept within about 4-8 rad/s, assist included.
//
// All of them stay in a 2 x 2 m area (robot started at its centre; simulated with the Qualisys model of
// 2026-10-22): short legs there and back, and both tracks slowed by the forward creep while turning.
// t1-t3 start and end at the stand angle with a sync pattern (turns in place of 1 s and 2 s): the
// Qualisys, recorded on another computer, is aligned to the robot bag by the yaw rate. Their track
// speed changes are cosine ramps of speed_ramp (no current spikes).
// The constructor checks the limits (shoulder range, track joint speed with the assist) and throws
// std::invalid_argument if a mission would exceed them.
class IdentificationMission {
public:
    /// mission: "t1", "t2", "t3", or several in a row as "t1,t2,t3" (std::invalid_argument otherwise).
    IdentificationMission(const std::string& mission, const IdentificationParams& params);

    IdentSetpoint at(double t) const;
    /// Set point of segment i once its speed ramp is over (shoulders and speeds it is meant to hold)
    IdentSetpoint segment_target(size_t i) const;
    size_t segment_at(double t) const;
    const std::string& segment_name(size_t i) const { return segments_[i].name; }
    bool hips_moving(size_t i) const { return segments_[i].hips_moving; }
    size_t num_segments() const { return segments_.size(); }
    double duration() const { return duration_; }
    bool finished(double t) const { return t >= duration_; }
    const std::string& name() const { return name_; }

private:
    IdentSetpoint creep_compensated(IdentSetpoint sp) const;
    using Profile = std::function<IdentSetpoint(double tau, double t)>;   // positions and speeds only
    struct Segment {
        std::string name;
        double duration;
        Profile f;
        bool hips_moving;
    };

    void hold(const std::string& name, double dur, double a_front, double a_hind, double v_left = 0.0,
              double v_right = 0.0);
    void move(double dur, double f0, double h0, double f1, double h1);
    void sync(double a);
    void build_t1();
    void build_t2();
    void build_t3();
    void check_limits() const;

    IdentificationParams p_;
    std::string name_;
    std::vector<Segment> segments_;
    std::vector<double> starts_;
    double duration_ = 0.0;
};

} // namespace omni_mulinex_joystick
