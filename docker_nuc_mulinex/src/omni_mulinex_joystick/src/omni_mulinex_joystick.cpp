#include "omni_mulinex_joystick/omni_mulinex_joystick.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace omni_mulinex_joystick {

namespace {

// Traquad joints for the identification missions. sigma: alpha = sigma·q_HFE (q_stand = sigma·1.6);
// track sign: joint speed sign for forward belt motion (right side mirrored, as the SBC wheel IK).
struct Leg {
    const char* hfe;
    const char* wheel;
    double sigma;
    double track_sign;
    bool front;
    bool left;
};
constexpr std::array<Leg, 4> kLegs{{
    {"LF_HFE", "LF_WHEEL", 1.0, 1.0, true, true},
    {"LH_HFE", "LH_WHEEL", -1.0, 1.0, false, true},
    {"RF_HFE", "RF_WHEEL", -1.0, -1.0, true, false},
    {"RH_HFE", "RH_WHEEL", 1.0, -1.0, false, false},
}};

} // namespace

OmniMulinexJoystick::OmniMulinexJoystick(): Node("omni_mulinex_joystick")
{
    print_instructions();

    // Declare parameters
    this->declare_parameter("sup_vel_x", 1.0);
    this->declare_parameter("sup_vel_y", 1.0);
    this->declare_parameter("sup_omega", 2.0);
    this->declare_parameter("height_step", 0.005);
    this->declare_parameter("yaw_step", 0.02);
    this->declare_parameter("roll_step", 0.02);
    this->declare_parameter("pitch_step", 0.02);
    this->declare_parameter("pos_step", 0.005);
    this->declare_parameter("max_height", 0.05);
    this->declare_parameter("max_roll", 0.3);
    this->declare_parameter("max_pitch", 0.3);
    this->declare_parameter("max_yaw", 0.15);
    this->declare_parameter("max_pos_x", 0.05);
    this->declare_parameter("max_pos_y", 0.05);
    this->declare_parameter("deadzone", 0.1);
    this->declare_parameter("timer_period_ms", 100);
    this->declare_parameter("subscribe_state", true);
    // Demo parameters for pre-defined movement pattern (for testing/demo purposes)
    this->declare_parameter("demo.area_width", 0.5);   // metres, left-right
    this->declare_parameter("demo.area_depth", 0.5);   // metres, front-back
    this->declare_parameter("demo.travel_time", 1.0);  // seconds per leg
    this->declare_parameter("demo.pose_time",    1.0);   // seconds per pose leg
    this->declare_parameter("demo.yaw_angle",    1.5708); // radians (≈90°)
    this->declare_parameter("demo.pitch_angle",  0.2);    // radians
    this->declare_parameter("demo.roll_angle",   0.2);    // radians
    this->declare_parameter("demo.base_x", 0.2);
    this->declare_parameter("demo.base_z", 0.05);
    // Figure-eight trajectory (START)
    this->declare_parameter("figure_eight.size", 1.0);         // metres, half-length of the eight
    this->declare_parameter("figure_eight.max_lin_vel", 0.2);  // m/s
    this->declare_parameter("figure_eight.max_ang_vel", 0.5);  // rad/s
    this->declare_parameter("figure_eight.laps", 1);
    this->declare_parameter("figure_eight.ramp_time", 3.0);    // seconds
    this->declare_parameter("figure_eight.holonomic", false);  // false = differential (vx + wz)
    // Identification missions (○)
    this->declare_parameter("identification.mission", "t1");   // t1, t2, t3
    this->declare_parameter("identification.stand", 1.6);      // rad, shoulder angle of q_stand
    this->declare_parameter("identification.alphas", std::vector<double>{1.05, 1.3, 1.6, 1.85});
    this->declare_parameter("identification.sprocket_radius", 0.028);  // m, wheel_rad of the SBC
    this->declare_parameter("identification.assist_gain", 10.0);       // transition_wheels.gain
    this->declare_parameter("identification.sync_speed", 0.1);         // m/s
    this->declare_parameter("identification.stance_tolerance", 0.1);   // rad
    this->declare_parameter("identification.period_ms", 10);
    this->declare_parameter("identification.speed_ramp", 0.5);       // s, track speed ramps
    this->declare_parameter("identification.max_track_speed", 9.0);  // rad/s, drives limited to 10
    this->declare_parameter("identification.alpha_min", 1.0);        // rad, tracks flat / ankle stops
    this->declare_parameter("identification.alpha_max", 1.9);

    // Read parameters
    sup_vel_x_ = this->get_parameter("sup_vel_x").as_double();
    sup_vel_y_ = this->get_parameter("sup_vel_y").as_double();
    sup_omega_ = this->get_parameter("sup_omega").as_double();
    height_step_ = this->get_parameter("height_step").as_double();
    yaw_step_ = this->get_parameter("yaw_step").as_double();
    roll_step_ = this->get_parameter("roll_step").as_double();
    pitch_step_ = this->get_parameter("pitch_step").as_double();
    pos_step_ = this->get_parameter("pos_step").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    max_roll_ = this->get_parameter("max_roll").as_double();
    max_pitch_ = this->get_parameter("max_pitch").as_double();
    max_yaw_ = this->get_parameter("max_yaw").as_double();
    max_pos_x_ = this->get_parameter("max_pos_x").as_double();
    max_pos_y_ = this->get_parameter("max_pos_y").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();
    subscribe_state_ = this->get_parameter("subscribe_state").as_bool();
    demo_area_width_ = this->get_parameter("demo.area_width").as_double();
    demo_area_depth_ = this->get_parameter("demo.area_depth").as_double();
    demo_travel_time_ = this->get_parameter("demo.travel_time").as_double();
    demo_pose_time_   = this->get_parameter("demo.pose_time").as_double();
    demo_yaw_angle_   = this->get_parameter("demo.yaw_angle").as_double();
    demo_pitch_angle_  = this->get_parameter("demo.pitch_angle").as_double();
    demo_roll_angle_  = this->get_parameter("demo.roll_angle").as_double();
    demo_base_x_  = this->get_parameter("demo.base_x").as_double();
    demo_base_z_  = this->get_parameter("demo.base_z").as_double();

    FigureEightParams fig8;
    fig8.size = this->get_parameter("figure_eight.size").as_double();
    fig8.max_lin_vel = this->get_parameter("figure_eight.max_lin_vel").as_double();
    fig8.max_ang_vel = this->get_parameter("figure_eight.max_ang_vel").as_double();
    fig8.laps = this->get_parameter("figure_eight.laps").as_int();
    fig8.ramp_time = this->get_parameter("figure_eight.ramp_time").as_double();
    fig8.holonomic = this->get_parameter("figure_eight.holonomic").as_bool();
    figure_eight_ = std::make_unique<FigureEight>(fig8);

    RCLCPP_INFO(
        this->get_logger(), "Figure-eight: size=%.2f m, %d lap(s), %s, duration=%.1f s",
        fig8.size, fig8.laps, fig8.holonomic ? "holonomic" : "differential",
        figure_eight_->duration()
    );


    // Twist publisher with BestEffort QoS and deadline
    rclcpp::QoS twist_qos(10);
    twist_qos.best_effort();
    twist_qos.deadline(std::chrono::milliseconds(timer_period_ms_ + 5));
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/omni_controller/twist_cmd", twist_qos);

    // Pose publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ik_controller/base_pose", 1);

    // Identification missions: hips and tracks commanded directly (the controller subscribes Reliable)
    rclcpp::QoS reliable_qos(5);
    reliable_qos.reliable();
    hips_pub_ = this->create_publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>(
        "/omni_controller/joints_reference", reliable_qos
    );
    wheels_pub_ = this->create_publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>(
        "/omni_controller/direct_wheels_cmd", reliable_qos
    );
    segment_pub_ = this->create_publisher<std_msgs::msg::String>("~/identification/segment", 10);

    // Joy subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&OmniMulinexJoystick::joy_callback, this, std::placeholders::_1)
    );

    // State subscriber (optional)
    if (subscribe_state_) {
        state_sub_ = this->create_subscription<pi3hat_moteus_int_msgs::msg::JointsStates>(
            "/omni_controller/joints_state", 10,
            std::bind(&OmniMulinexJoystick::state_callback, this, std::placeholders::_1)
        );
    }

    // Service clients
    activate_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/activate_srv");
    emergency_client_ =
        this->create_client<std_srvs::srv::SetBool>("/omni_controller/emergency_srv");
    rest_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/rest_srv");
    stand_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/stand_srv");
    ik_reinit_client_ =
        this->create_client<std_srvs::srv::SetBool>("/ik_controller/reinitialize_srv");
    ik_activate_client_ =
        this->create_client<std_srvs::srv::SetBool>("/ik_controller/activate_srv");
    wheel_mode_client_ =
        this->create_client<std_srvs::srv::SetBool>("/omni_controller/set_wheel_mode");

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms_),
        std::bind(&OmniMulinexJoystick::timer_callback, this)
    );
}

void OmniMulinexJoystick::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    const auto& axes = msg->axes;
    const auto& buttons = msg->buttons;

    // Initialize previous button state on first message
    if (prev_buttons_.empty()) {
        prev_buttons_.resize(buttons.size(), false);
    }

    // Helper: apply deadzone
    auto dz = [this](double val) -> double { return (std::abs(val) < deadzone_) ? 0.0 : val; };

    // Helper: rising edge detection for buttons
    auto btn_rising = [&](size_t idx) -> bool {
        if (idx >= buttons.size())
            return false;
        return buttons[idx] && (idx >= prev_buttons_.size() || !prev_buttons_[idx]);
    };


    // SELECT → toggle demo mode
    if (btn_rising(8)) {
        if (!demo_active_) {
            if (figure_eight_active_)
                stop_figure_eight("demo started");
            stop_identification("demo started");
            demo_active_ = true;
            transition_demo(DemoPhase::RESET);
            RCLCPP_INFO(this->get_logger(), "Demo started");
        } else {
            // Route the interrupt through SINK so the robot ends in a safe state.
            transition_demo(DemoPhase::SINK);
            RCLCPP_INFO(this->get_logger(), "Demo interrupted");
        }
    }

    // ── Store analog stick values ───────────────────────────────────────
    if (axes.size() > 0)
        joy_left_x_ = dz(axes[0]);
    if (axes.size() > 1)
        joy_left_y_ = dz(axes[1]);
    if (axes.size() > 3)
        joy_right_x_ = dz(axes[3]);
    if (axes.size() > 4)
        joy_right_y_ = dz(axes[4]);

    // L2 modifier (button 6)
    l2_held_ = (buttons.size() > 6) && buttons[6];

    // ── D-pad: base x/y position (edge-triggered, only when IK active) ─
    // D-pad up/down (axes[7]) → x position
    if (axes.size() > 7) {
        bool active = std::abs(axes[7]) > 0.5;
        if (active && !prev_dpad_ud_active_ && ik_active_) {
            double sign = (axes[7] > 0.0) ? 1.0 : -1.0;
            body_x_ = std::clamp(body_x_ + sign * pos_step_, -max_pos_x_, max_pos_x_);
        }
        prev_dpad_ud_active_ = active;
    }

    // D-pad left/right (axes[6]) → y position
    if (axes.size() > 6) {
        bool active = std::abs(axes[6]) > 0.5;
        if (active && !prev_dpad_lr_active_ && ik_active_) {
            double sign = (axes[6] > 0.0) ? 1.0 : -1.0;
            body_y_ = std::clamp(body_y_ + sign * pos_step_, -max_pos_y_, max_pos_y_);
        }
        prev_dpad_lr_active_ = active;
    }

    // ── Figure-eight ────────────────────────────────────────────────────
    // START (btn 9) → start / stop the figure-eight trajectory
    if (btn_rising(9)) {
        if (figure_eight_active_)
            stop_figure_eight("START pressed");
        else if (demo_active_ || ident_active_)
            RCLCPP_WARN(this->get_logger(), "Figure-eight ignored: demo or identification running");
        else
            start_figure_eight();
    }
    // Any stick input or state-changing button takes control back
    if (figure_eight_active_) {
        if (joy_left_x_ != 0.0 || joy_left_y_ != 0.0 || joy_right_x_ != 0.0 || joy_right_y_ != 0.0)
            stop_figure_eight("stick moved");
        else if (btn_rising(0) || btn_rising(2) || btn_rising(4) || btn_rising(5))
            stop_figure_eight("service button pressed");
    }

    // ── Identification missions ───────────────────────────────────────────
    // ○ (btn 1) → start / stop the mission in identification.mission
    if (btn_rising(1)) {
        if (ident_active_)
            stop_identification("○ pressed");
        else if (demo_active_ || figure_eight_active_)
            RCLCPP_WARN(this->get_logger(), "Identification ignored: demo or figure-eight running");
        else
            start_identification();
    }
    // Any stick input or other button takes control back (R1 then still calls the emergency stop)
    if (ident_active_) {
        if (joy_left_x_ != 0.0 || joy_left_y_ != 0.0 || joy_right_x_ != 0.0 || joy_right_y_ != 0.0)
            stop_identification("stick moved");
        else if (btn_rising(0) || btn_rising(2) || btn_rising(3) || btn_rising(4) || btn_rising(5) ||
                 btn_rising(8) || btn_rising(9))
            stop_identification("button pressed");
    }

    // ── Services ────────────────────────────────────────────────────────
    // L1 (btn 4) → activate hardware
    if (btn_rising(4)) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (activate_client_->service_is_ready()) {
            activate_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Activate service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Activate service not available");
        }
    }

    // □ (btn 3) → activate IK controller (reinitialize + start)
    if (btn_rising(3)) {
        // Reset body pose to zero for clean start
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;

        if (ik_activate_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = true;
            ik_activate_client_->async_send_request(req);
            ik_active_ = true;
            RCLCPP_INFO(this->get_logger(), "IK controller activated");
        } else {
            RCLCPP_WARN(this->get_logger(), "IK activate service not available");
        }
    }

    // R1 (btn 5) → emergency stop (also deactivates IK)
    if (btn_rising(5)) {
        deactivate_ik();

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (emergency_client_->service_is_ready()) {
            emergency_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Emergency stop service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Emergency stop service not available");
        }
    }

    // X (btn 0) → rest (also deactivates IK)
    if (btn_rising(0)) {
        deactivate_ik();

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (rest_client_->service_is_ready()) {
            rest_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Rest service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Rest service not available");
        }
    }

    // △ (btn 2) → stand (also deactivates IK)
    if (btn_rising(2)) {
        deactivate_ik();

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (stand_client_->service_is_ready()) {
            stand_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Stand service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Stand service not available");
        }
    }

    // ── Resets ──────────────────────────────────────────────────────────
    // L3 (btn 11) → reset wheels
    if (btn_rising(11)) {
        joy_left_x_ = joy_left_y_ = joy_right_x_ = joy_right_y_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Wheels reset");
    }

    // R3 (btn 12) → reset body pose
    if (btn_rising(12)) {
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Body pose reset");
    }

    // PS (btn 10) → reset all
    if (btn_rising(10)) {
        joy_left_x_ = joy_left_y_ = joy_right_x_ = joy_right_y_ = 0.0;
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "All reset");
    }

    // Update previous button state
    prev_buttons_.resize(buttons.size());
    for (size_t i = 0; i < buttons.size(); i++) {
        prev_buttons_[i] = buttons[i];
    }
}

void OmniMulinexJoystick::timer_callback()
{
    // ── Twist (wheels) ──────────────────────────────────────────────────
    geometry_msgs::msg::Twist twist_msg;


    // ── Demo mode overrides normal twist ────────────────────────────────
    if (demo_active_) {
        const double dt = timer_period_ms_ * 1e-3;          // seconds per tick
        const double vel_y  = demo_area_width_ / demo_travel_time_;   // left-right
        const double vel_x  = demo_area_depth_ / demo_travel_time_;   // front-back
        const double vel_yaw    = demo_yaw_angle_    / demo_pose_time_;
        const double vel_pitch  = demo_pitch_angle_  / demo_pose_time_;
        const double vel_pitch_inverse  = -demo_pitch_angle_  / demo_pose_time_;
        const double vel_roll   = demo_roll_angle_   / demo_pose_time_;
        const double vel_roll_inverse   = -demo_roll_angle_   / demo_pose_time_;
        const double vel_base_x_   = demo_base_x_   / demo_pose_time_;
        const double vel_base_z_   = demo_base_z_   / demo_pose_time_;

        switch (demo_phase_) {
            case DemoPhase::RESET: {
                 // Request body lowering
                body_x_ = body_y_ = body_height_ = 0.0;
                body_roll_ = body_pitch_ = body_yaw_ = 0.0;
                deactivate_ik();

                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = true;
                if (rest_client_->service_is_ready()) {
                    rest_client_->async_send_request(req);
                    RCLCPP_INFO(this->get_logger(), "Rest service called");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Rest service not available");
                }               
                  
                break;
            }
            case DemoPhase::ACTIVATE: {

                auto req_ = std::make_shared<std_srvs::srv::SetBool::Request>();
                req_->data = true;
                if (activate_client_->service_is_ready()) {
                    activate_client_->async_send_request(req_);
                    RCLCPP_INFO(this->get_logger(), "Activate service called");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Activate service not available");
                }                   
                  
                break;
            }
            case DemoPhase::MOVE_RIGHT:         twist_msg.linear.y = -vel_y;        break;
            case DemoPhase::MOVE_LEFT:          twist_msg.linear.y = +vel_y;        break;
            case DemoPhase::MOVE_FORWARD:       twist_msg.linear.x = +vel_x;        break;
            case DemoPhase::MOVE_BACKWARD:      twist_msg.linear.x = -vel_x;        break;
            case DemoPhase::YAW_RIGHT:          twist_msg.angular.z = -vel_yaw;     break;
            case DemoPhase::YAW_LEFT:           twist_msg.angular.z = +vel_yaw;     break;
            // ── Rise: ramp body_height_ up to max_height_ ─────────────────
            case DemoPhase::RISE: {
                // Request body raising
                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = true;
                if (stand_client_->service_is_ready()) {
                    stand_client_->async_send_request(req);
                    RCLCPP_INFO(this->get_logger(), "Stand service called");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Stand service not available");
                }                         
                break;
            }
            // ── Elevated level ────────────────────────────────────────────
            case DemoPhase::MOVE_RIGHT_HIGH:    twist_msg.linear.y = -vel_y;        break;
            case DemoPhase::MOVE_LEFT_HIGH:     twist_msg.linear.y = +vel_y;        break;
            case DemoPhase::MOVE_FORWARD_HIGH:  twist_msg.linear.x = +vel_x;        break;
            case DemoPhase::MOVE_BACKWARD_HIGH: twist_msg.linear.x = -vel_x;        break;
            case DemoPhase::YAW_RIGHT_HIGH:     twist_msg.angular.z = -vel_yaw;     break;
            case DemoPhase::YAW_LEFT_HIGH:      twist_msg.angular.z = +vel_yaw;     break;
            case DemoPhase::ACTIVATE_IK: {
                // Reset body pose to zero for clean start
                body_x_ = body_y_ = body_height_ = 0.0;
                body_roll_ = body_pitch_ = body_yaw_ = 0.0;

                if (ik_activate_client_->service_is_ready()) {
                    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                    req->data = true;
                    ik_activate_client_->async_send_request(req);
                    ik_active_ = true;
                    RCLCPP_INFO(this->get_logger(), "IK controller activated");
                } else {
                    RCLCPP_WARN(this->get_logger(), "IK activate service not available");
                }
                break;
            }
            // ── Pitch ─────────────────────────────────────────────────────
            case DemoPhase::PITCH_FORWARD:
                body_pitch_ = std::clamp(body_pitch_ + vel_pitch * dt, -max_pitch_, max_pitch_);
                break;
            case DemoPhase::PITCH_BACKWARD:
                body_pitch_ = std::clamp(body_pitch_ - vel_pitch * dt, -max_pitch_, max_pitch_);
                break;
            case DemoPhase::PITCH_INVERSE:
                body_pitch_ = std::clamp(body_pitch_ + vel_pitch_inverse * dt, -max_pitch_, max_pitch_);
                break;
            case DemoPhase::PITCH_HOME:
                body_pitch_ = std::clamp(body_pitch_ - vel_pitch_inverse * dt, -max_pitch_, max_pitch_);
                break;
            // ── Roll ──────────────────────────────────────────────────────
            case DemoPhase::ROLL_RIGHT:
                body_roll_ = std::clamp(body_roll_ + vel_roll * dt, -max_roll_, max_roll_);
                break;
            case DemoPhase::ROLL_LEFT:
                body_roll_ = std::clamp(body_roll_ - vel_roll * dt, -max_roll_, max_roll_);
                break;
            case DemoPhase::ROLL_INVERSE:
                body_roll_ = std::clamp(body_roll_ + vel_roll_inverse * dt, -max_roll_, max_roll_);
                break;
            case DemoPhase::ROLL_HOME:
                body_roll_ = std::clamp(body_roll_ - vel_roll_inverse * dt, -max_roll_, max_roll_);
                break;

            case DemoPhase::BASE_FORWARD:
                body_x_ = std::clamp(body_x_ + vel_base_x_ * dt, -max_pos_x_, max_pos_x_);
                break;
            case DemoPhase::BASE_BACKWARD:
                body_x_ = std::clamp(body_x_ - vel_base_x_ * dt, -max_pos_x_, max_pos_x_);
                break;

            case DemoPhase::BASE_RISE:
                body_height_ = std::clamp(body_height_ + vel_base_z_ * dt, -max_height_, max_height_);
                break;
            case DemoPhase::BASE_LOWER:
                body_height_ = std::clamp(body_height_ - vel_base_z_ * dt, -max_height_, max_height_);
                break;

            case DemoPhase::SINK: {
                 // Request body lowering
                body_x_ = body_y_ = body_height_ = 0.0;
                body_roll_ = body_pitch_ = body_yaw_ = 0.0;
                deactivate_ik();
                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = true;
                if (rest_client_->service_is_ready()) {
                    rest_client_->async_send_request(req);
                    RCLCPP_INFO(this->get_logger(), "Rest service called");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Rest service not available");
                }                         
                break;
            }
            case DemoPhase::IDLE:
                break;
            case DemoPhase::DELAY:
                // do nothing — just wait out demo_current_phase_duration_
                break;
            default:
                demo_active_ = false;
                break;
        }

        twist_pub_->publish(twist_msg);
        if (ik_active_) {
            auto q = quat_exp_vec({body_roll_, body_pitch_, body_yaw_});

            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = body_x_;
            pose_msg.position.y = body_y_;
            pose_msg.position.z = body_height_;
            pose_msg.orientation.x = q[0];
            pose_msg.orientation.y = q[1];
            pose_msg.orientation.z = q[2];
            pose_msg.orientation.w = q[3];
            pose_pub_->publish(pose_msg);
        }

        demo_phase_elapsed_ += dt;

    if (demo_phase_elapsed_ >= demo_current_phase_duration_) {
        switch (demo_phase_) {
            case DemoPhase::RESET:              transition_demo(DemoPhase::ACTIVATE, 5.0);        break;
            case DemoPhase::ACTIVATE:           transition_demo(DemoPhase::MOVE_RIGHT, 2.0);       break;
            case DemoPhase::MOVE_RIGHT:         transition_demo(DemoPhase::MOVE_LEFT);             RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_RIGHT done");         break;
            case DemoPhase::MOVE_LEFT:          transition_demo(DemoPhase::MOVE_FORWARD);          RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_LEFT done");          break;
            case DemoPhase::MOVE_FORWARD:       transition_demo(DemoPhase::MOVE_BACKWARD);         RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_FORWARD done");       break;
            case DemoPhase::MOVE_BACKWARD:      transition_demo(DemoPhase::YAW_RIGHT);             RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_BACKWARD done");      break;
            case DemoPhase::YAW_RIGHT:          transition_demo(DemoPhase::YAW_LEFT);              RCLCPP_INFO(this->get_logger(), "[DEMO] YAW_RIGHT done");          break;
            case DemoPhase::YAW_LEFT:
                body_yaw_ = 0.0;  // guard: IK pose yaw should already be 0 here (only set in L2-held branch)
                transition_demo(DemoPhase::RISE);                                                   RCLCPP_INFO(this->get_logger(), "[DEMO] YAW_LEFT done");           break;
            case DemoPhase::RISE:               transition_demo(DemoPhase::MOVE_RIGHT_HIGH, 5.0);       RCLCPP_INFO(this->get_logger(), "[DEMO] RISE done");               break;
            case DemoPhase::MOVE_RIGHT_HIGH:    transition_demo(DemoPhase::MOVE_LEFT_HIGH);        RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_RIGHT_HIGH done");    break;
            case DemoPhase::MOVE_LEFT_HIGH:     transition_demo(DemoPhase::MOVE_FORWARD_HIGH);     RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_LEFT_HIGH done");     break;
            case DemoPhase::MOVE_FORWARD_HIGH:  transition_demo(DemoPhase::MOVE_BACKWARD_HIGH);    RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_FORWARD_HIGH done");  break;
            case DemoPhase::MOVE_BACKWARD_HIGH: transition_demo(DemoPhase::YAW_RIGHT_HIGH);        RCLCPP_INFO(this->get_logger(), "[DEMO] MOVE_BACKWARD_HIGH done"); break;
            case DemoPhase::YAW_RIGHT_HIGH:     transition_demo(DemoPhase::YAW_LEFT_HIGH);         RCLCPP_INFO(this->get_logger(), "[DEMO] YAW_RIGHT_HIGH done");     break;
            case DemoPhase::YAW_LEFT_HIGH:
                body_yaw_ = 0.0;  // guard: IK pose yaw should already be 0 here (only set in L2-held branch)
                transition_demo(DemoPhase::ACTIVATE_IK);                                            RCLCPP_INFO(this->get_logger(), "[DEMO] YAW_LEFT_HIGH done");      break;
            case DemoPhase::ACTIVATE_IK:        transition_demo(DemoPhase::PITCH_FORWARD, 3.0);         RCLCPP_INFO(this->get_logger(), "[DEMO] ACTIVATE_IK done");        break;
            case DemoPhase::PITCH_FORWARD:      transition_demo(DemoPhase::PITCH_BACKWARD);        RCLCPP_INFO(this->get_logger(), "[DEMO] PITCH_FORWARD done");      break;
            case DemoPhase::PITCH_BACKWARD:     transition_demo(DemoPhase::PITCH_INVERSE);         RCLCPP_INFO(this->get_logger(), "[DEMO] PITCH_BACKWARD done");     break;
            case DemoPhase::PITCH_INVERSE:      transition_demo(DemoPhase::PITCH_HOME);            RCLCPP_INFO(this->get_logger(), "[DEMO] PITCH_INVERSE done");     break;
            case DemoPhase::PITCH_HOME:
                body_pitch_ = 0.0;
                transition_demo(DemoPhase::ROLL_RIGHT);                                             RCLCPP_INFO(this->get_logger(), "[DEMO] PITCH_HOME done");         break;
            case DemoPhase::ROLL_RIGHT:         transition_demo(DemoPhase::ROLL_LEFT);             RCLCPP_INFO(this->get_logger(), "[DEMO] ROLL_RIGHT done");         break;
            case DemoPhase::ROLL_LEFT:          transition_demo(DemoPhase::ROLL_INVERSE);          RCLCPP_INFO(this->get_logger(), "[DEMO] ROLL_LEFT done");          break;
            case DemoPhase::ROLL_INVERSE:       transition_demo(DemoPhase::ROLL_HOME);          RCLCPP_INFO(this->get_logger(), "[DEMO] ROLL_INVERSE done");         break;
            case DemoPhase::ROLL_HOME:
                body_roll_ = 0.0;
                transition_demo(DemoPhase::BASE_FORWARD);                                           RCLCPP_INFO(this->get_logger(), "[DEMO] ROLL_HOME done");          break;
            case DemoPhase::BASE_FORWARD:       transition_demo(DemoPhase::BASE_BACKWARD);         RCLCPP_INFO(this->get_logger(), "[DEMO] BASE_FORWARD done");       break;
            case DemoPhase::BASE_BACKWARD:
                body_x_ = 0.0;
                transition_demo(DemoPhase::BASE_RISE);                                              RCLCPP_INFO(this->get_logger(), "[DEMO] BASE_BACKWARD done");      break;
            case DemoPhase::BASE_RISE:          transition_demo(DemoPhase::BASE_LOWER);            RCLCPP_INFO(this->get_logger(), "[DEMO] BASE_RISE done");          break;
            case DemoPhase::BASE_LOWER:
                body_height_ = 0.0;
                transition_demo(DemoPhase::SINK, 1.0);                                                   RCLCPP_INFO(this->get_logger(), "[DEMO] BASE_LOWER done");         break;
            case DemoPhase::SINK:
                demo_active_ = false;
                demo_phase_  = DemoPhase::IDLE;
                RCLCPP_INFO(this->get_logger(), "[DEMO] Complete");
                break;
            case DemoPhase::DELAY: {
                DemoPhase resume = demo_next_phase_;
                demo_next_phase_ = DemoPhase::IDLE;   // clear before use
                if (resume == DemoPhase::DELAY) {
                    RCLCPP_ERROR(this->get_logger(), "[DEMO] DELAY next phase is DELAY — aborting demo");
                    demo_active_ = false;
                    demo_phase_  = DemoPhase::IDLE;
                } else {
                    set_demo_phase(resume);           // NOT transition_demo — never go through delay again
                }
                break;
            }
            default: break;
        }
    }
        return;   // skip normal twist/pose publishing below
    }



    if (ident_active_) {
        // Tracks and hips are commanded by identification_step(); keep the twist stream alive
        // (deadline QoS of the controller) with zero velocity
        twist_pub_->publish(twist_msg);
        return;
    }

    if (figure_eight_active_) {
        double t = (this->now() - figure_eight_start_time_).seconds();
        twist_msg = figure_eight_->twist_at(t);
        if (figure_eight_->finished(t)) {
            figure_eight_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Figure-eight completed");
        }
    } else if (!l2_held_) {
        // Normal: sticks → wheels
        twist_msg.linear.x = joy_left_y_ * sup_vel_x_;
        twist_msg.linear.y = joy_left_x_ * sup_vel_y_;
        twist_msg.angular.z = joy_right_x_ * sup_omega_;

        // Right stick Y → height rate (only when IK active)
        if (ik_active_) {
            body_height_ =
                std::clamp(body_height_ + joy_right_y_ * height_step_, -max_height_, max_height_);
        }
    } else {
        // L2 held: wheels zero
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        // Sticks → body orientation rate (only when IK active)
        if (ik_active_) {
            body_roll_ = std::clamp(body_roll_ + joy_left_x_ * roll_step_, -max_roll_, max_roll_);
            body_pitch_ =
                std::clamp(body_pitch_ + joy_left_y_ * pitch_step_, -max_pitch_, max_pitch_);
            body_yaw_ = std::clamp(body_yaw_ + joy_right_x_ * yaw_step_, -max_yaw_, max_yaw_);
        }
    }
    twist_pub_->publish(twist_msg);

    // ── Pose (body) – only publish when IK is active ────────────────────
    if (ik_active_) {
        auto q = quat_exp_vec({body_roll_, body_pitch_, body_yaw_});

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = body_x_;
        pose_msg.position.y = body_y_;
        pose_msg.position.z = body_height_;
        pose_msg.orientation.x = q[0];
        pose_msg.orientation.y = q[1];
        pose_msg.orientation.z = q[2];
        pose_msg.orientation.w = q[3];
        pose_pub_->publish(pose_msg);
    }
}

void OmniMulinexJoystick::
    state_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg)
{
    // HFE positions: the identification missions check the stance before starting
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
        for (const auto& leg : kLegs) {
            if (msg->name[i] == leg.hfe)
                hfe_pos_[leg.hfe] = msg->position[i];
        }
    }
}

void OmniMulinexJoystick::start_figure_eight()
{
    figure_eight_active_ = true;
    figure_eight_start_time_ = this->now();
    RCLCPP_INFO(
        this->get_logger(), "Figure-eight started (%.1f s)", figure_eight_->duration()
    );
}

void OmniMulinexJoystick::stop_figure_eight(const char* reason)
{
    figure_eight_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Figure-eight stopped (%s)", reason);
}

void OmniMulinexJoystick::start_identification()
{
    // parameters read at every start: ros2 param set changes the next mission
    const auto mission = this->get_parameter("identification.mission").as_string();
    ident_params_.stand = this->get_parameter("identification.stand").as_double();
    ident_params_.alphas = this->get_parameter("identification.alphas").as_double_array();
    ident_params_.sprocket_radius = this->get_parameter("identification.sprocket_radius").as_double();
    ident_params_.assist_gain = this->get_parameter("identification.assist_gain").as_double();
    ident_params_.sync_speed = this->get_parameter("identification.sync_speed").as_double();
    ident_stance_tolerance_ = this->get_parameter("identification.stance_tolerance").as_double();
    ident_period_ms_ = this->get_parameter("identification.period_ms").as_int();
    ident_params_.speed_ramp = this->get_parameter("identification.speed_ramp").as_double();
    ident_params_.max_track_speed = this->get_parameter("identification.max_track_speed").as_double();
    ident_params_.alpha_min = this->get_parameter("identification.alpha_min").as_double();
    ident_params_.alpha_max = this->get_parameter("identification.alpha_max").as_double();
    try {
        ident_ = std::make_unique<IdentificationMission>(mission, ident_params_);
    } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(this->get_logger(), "Identification: %s", e.what());
        return;
    }
    // The missions start from the stand pose: refuse otherwise (they would jump the hips there)
    if (hfe_pos_.size() < kLegs.size()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Identification '%s' refused: no HFE positions from /omni_controller/joints_state",
            mission.c_str()
        );
        return;
    }
    for (const auto& leg : kLegs) {
        const double alpha = leg.sigma * hfe_pos_[leg.hfe];
        if (std::abs(alpha - ident_params_.stand) > ident_stance_tolerance_) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Identification '%s' refused: %s at %.2f rad, not standing at %.2f (press △)",
                mission.c_str(), leg.hfe, alpha, ident_params_.stand
            );
            return;
        }
    }
    if (!wheel_mode_client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Identification refused: set_wheel_mode not available");
        return;
    }
    deactivate_ik();   // the IK controller would also publish joints_reference
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    wheel_mode_client_->async_send_request(req);

    ident_active_ = true;
    ident_start_time_ = this->now();
    ident_segment_ = 0;
    ident_last_ = ident_->at(0.0);
    publish_identification_segment(0, 0.0);
    ident_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(ident_period_ms_),
        std::bind(&OmniMulinexJoystick::identification_step, this)
    );
    RCLCPP_INFO(
        this->get_logger(), "Identification '%s' started (%.0f s, %zu segments)", mission.c_str(),
        ident_->duration(), ident_->num_segments()
    );
}

void OmniMulinexJoystick::stop_identification(const char* reason)
{
    if (!ident_active_)
        return;
    ident_active_ = false;
    if (ident_timer_) {
        ident_timer_->cancel();
        ident_timer_.reset();
    }
    // Tracks to zero at once, hips held where they are (the controller then keeps them by its
    // timeouts); back to the twist (IK) mode so the sticks drive again
    IdentSetpoint hold = ident_last_;
    hold.rate_front = hold.rate_hind = 0.0;
    hold.v_left = hold.v_right = 0.0;
    publish_identification(hold);
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = false;
    if (wheel_mode_client_->service_is_ready())
        wheel_mode_client_->async_send_request(req);
    std_msgs::msg::String msg;
    msg.data = "{\"mission\": \"" + ident_->name() + "\", \"i\": " +
               std::to_string(ident_->num_segments()) + ", \"segment\": \"end\", \"reason\": \"" +
               reason + "\"}";
    segment_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Identification '%s' stopped (%s)", ident_->name().c_str(), reason);
}

void OmniMulinexJoystick::identification_step()
{
    if (!ident_active_)
        return;
    const double t = (this->now() - ident_start_time_).seconds();
    if (ident_->finished(t)) {
        stop_identification("completed");
        return;
    }
    const size_t i = ident_->segment_at(t);
    if (i != ident_segment_) {
        ident_segment_ = i;
        publish_identification_segment(i, t);
    }
    ident_last_ = ident_->at(t);
    publish_identification(ident_last_);
}

void OmniMulinexJoystick::publish_identification(const IdentSetpoint& sp)
{
    pi3hat_moteus_int_msgs::msg::JointsCommand wheels;
    pi3hat_moteus_int_msgs::msg::JointsCommand legs;
    for (const auto& leg : kLegs) {
        const double alpha = leg.front ? sp.alpha_front : sp.alpha_hind;
        const double rate = leg.front ? sp.rate_front : sp.rate_hind;
        const double v = leg.left ? sp.v_left : sp.v_right;
        // belt speed at the sprocket, plus the stand-up assist of the SBC: -k·q̇_HFE of the same leg
        wheels.name.push_back(leg.wheel);
        wheels.velocity.push_back(
            leg.track_sign * v / ident_params_.sprocket_radius -
            ident_params_.assist_gain * leg.sigma * rate
        );
        wheels.kp_scale.push_back(0.0);
        wheels.kd_scale.push_back(1.0);
        legs.name.push_back(leg.hfe);
        legs.position.push_back(leg.sigma * alpha);
        legs.velocity.push_back(leg.sigma * rate);
        legs.effort.push_back(0.0);
        legs.kp_scale.push_back(1.0);
        legs.kd_scale.push_back(1.0);
    }
    wheels.header.stamp = legs.header.stamp = this->now();
    wheels_pub_->publish(wheels);
    hips_pub_->publish(legs);
}

void OmniMulinexJoystick::publish_identification_segment(size_t i, double t)
{
    const auto sp = ident_->at(t);
    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        "{\"mission\": \"%s\", \"i\": %zu, \"segment\": \"%s\", \"t\": %.3f, \"alpha_F\": %.4f, "
        "\"alpha_H\": %.4f, \"v_left\": %.4f, \"v_right\": %.4f, \"hips_moving\": %s}",
        ident_->name().c_str(), i, ident_->segment_name(i).c_str(), t, sp.alpha_front, sp.alpha_hind,
        sp.v_left, sp.v_right, ident_->hips_moving(i) ? "true" : "false"
    );
    std_msgs::msg::String msg;
    msg.data = buf;
    segment_pub_->publish(msg);
    RCLCPP_INFO(
        this->get_logger(), "[ID %s] t %6.1f s  %s  alpha %.2f/%.2f  v %+.3f/%+.3f", ident_->name().c_str(),
        t, ident_->segment_name(i).c_str(), sp.alpha_front, sp.alpha_hind, sp.v_left, sp.v_right
    );
}

void OmniMulinexJoystick::deactivate_ik()
{
    if (ik_active_) {
        ik_active_ = false;
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;

        if (ik_activate_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = false;
            ik_activate_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "IK controller deactivated");
        }
    }
}

void OmniMulinexJoystick::print_instructions()
{
    RCLCPP_INFO(
        this->get_logger(), "\n\033[32m"
                            "╔════════════════════════════════════════════════════════════╗\n"
                            "║              Omni Mulinex Joystick (PS4)                   ║\n"
                            "╠════════════════════════════════════════════════════════════╣\n"
                            "║  NORMAL MODE                                               ║\n"
                            "║  ───────────                                               ║\n"
                            "║  Left Stick X/Y   → wheel vy / vx                          ║\n"
                            "║  Right Stick X    → wheel omega                            ║\n"
                            "║  Right Stick Y    → base height (rate)                     ║\n"
                            "║                                                            ║\n"
                            "║  L2 + STICKS (body orientation)                            ║\n"
                            "║  ──────────────────────────────                            ║\n"
                            "║  L2 + Left Stick  → roll / pitch (rate)                    ║\n"
                            "║  L2 + Right Stick → yaw (rate)                             ║\n"
                            "║                                                            ║\n"
                            "║  D-PAD (base position)                                     ║\n"
                            "║  ────────────────────                                      ║\n"
                            "║  D-Pad U/D → x position (step)                             ║\n"
                            "║  D-Pad L/R → y position (step)                             ║\n"
                            "║                                                            ║\n"
                            "║  BUTTONS                                                   ║\n"
                            "║  ───────                                                   ║\n"
                            "║  L1 → ACTIVATE HW     R1 → EMERGENCY STOP                  ║\n"
                            "║  □  → ACTIVATE IK     ✕  → REST                            ║\n"
                            "║  △  → STAND           ○  → start/stop identification       ║\n"
                            "║  SELECT → toggle demo cycle   START → toggle figure-eight  ║\n"
                            "║  L3 → reset wheels    R3 → reset body pose                 ║\n"
                            "║  PS → reset ALL                                            ║\n"
                            "╚════════════════════════════════════════════════════════════╝"
                            "\033[0m"
    );
}




void OmniMulinexJoystick::set_demo_phase(DemoPhase phase)
{
    demo_phase_          = phase;
    demo_phase_elapsed_  = 0.0;

    switch (phase) {
        case DemoPhase::RESET:
        case DemoPhase::ACTIVATE_IK:            
        case DemoPhase::RISE:
        case DemoPhase::SINK:
            demo_current_phase_duration_ = timer_period_ms_ * 1e-3;
                break;
        case DemoPhase::BASE_RISE:   case DemoPhase::BASE_LOWER:
        case DemoPhase::YAW_RIGHT:   case DemoPhase::YAW_LEFT:
        case DemoPhase::YAW_RIGHT_HIGH: case DemoPhase::YAW_LEFT_HIGH:
        case DemoPhase::PITCH_FORWARD:  case DemoPhase::PITCH_BACKWARD:
        case DemoPhase::PITCH_INVERSE:  case DemoPhase::PITCH_HOME:
        case DemoPhase::ROLL_RIGHT:     case DemoPhase::ROLL_LEFT:
        case DemoPhase::ROLL_INVERSE:   case DemoPhase::ROLL_HOME:
        case DemoPhase::BASE_FORWARD:   case DemoPhase::BASE_BACKWARD:
            demo_current_phase_duration_ = demo_pose_time_;
            break;
        case DemoPhase::DELAY:
            demo_current_phase_duration_ = demo_delay_time_;
            break;
        default:
            demo_current_phase_duration_ = demo_travel_time_;
            break;
    }
}

// Transition through an optional pause before the next phase.
// Pass delay_s = 0.0 to transition immediately (same behaviour as before).
void OmniMulinexJoystick::transition_demo(DemoPhase next, double delay_s)
{
    if (delay_s > 0.0) {
        demo_next_phase_ = next;
        demo_delay_time_ = delay_s;
        set_demo_phase(DemoPhase::DELAY);  
    } else {
        set_demo_phase(next);             
    }
}

} // namespace omni_mulinex_joystick
