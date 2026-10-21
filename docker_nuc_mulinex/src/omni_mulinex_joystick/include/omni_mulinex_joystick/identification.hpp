#pragma once

#include <functional>
#include <string>
#include <vector>

namespace omni_mulinex_joystick {

struct IdentificationParams {
    double stand = 1.6;                              // shoulder angle of q_stand (SBC yaml) [rad]
    std::vector<double> alphas{1.05, 1.3, 1.6, 1.85}; // shoulder angles of T1 / T2 [rad]
    double sprocket_radius = 0.028;                  // wheel_rad of the SBC wheel IK [m]
    double assist_gain = 10.0;                       // transition_wheels.gain of the SBC yaml
    double sync_speed = 0.1;                         // belt speed of the sync turns [m/s]
    double speed_ramp = 0.5;                         // track speed changes as cosine ramps of this [s] (t1-t3)
    double max_track_speed = 9.0;                    // largest track joint command, assist included [rad/s]
                                                     // (the drives are limited to 10, max_velocity in the SBC xacro)
    double alpha_min = 1.0;                          // shoulder range: tracks flat, ankles off their stops [rad]
    double alpha_max = 1.9;
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
//   t1           turns in place at each shoulder angle, track joint speeds 2, 4, 7 rad/s, both
//                directions, 8 s each
//   t2           straight runs and arcs (belt speeds v0·(1 ∓ rho), rho = 0, ±0.3, ±0.6) at each
//                shoulder angle, 5 s forward then 5 s back over the same path
//   t3           validation: arcs and a turn while the shoulders move all the time, front and hind
//                a quarter period out of phase
//
// t1-t3 start and end at the stand angle with a sync pattern (turns in place of 1 s and 2 s): the
// Qualisys, recorded on another computer, is aligned to the robot bag by the yaw rate. Their track
// speed changes are cosine ramps of speed_ramp (no current spikes).
// The constructor checks the limits (shoulder range, track joint speed with the assist) and throws
// std::invalid_argument if a mission would exceed them.
class IdentificationMission {
public:
    /// mission: "t1", "t2" or "t3" (std::invalid_argument otherwise).
    IdentificationMission(const std::string& mission, const IdentificationParams& params);

    IdentSetpoint at(double t) const;
    size_t segment_at(double t) const;
    const std::string& segment_name(size_t i) const { return segments_[i].name; }
    bool hips_moving(size_t i) const { return segments_[i].hips_moving; }
    size_t num_segments() const { return segments_.size(); }
    double duration() const { return duration_; }
    bool finished(double t) const { return t >= duration_; }
    const std::string& name() const { return name_; }

private:
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
