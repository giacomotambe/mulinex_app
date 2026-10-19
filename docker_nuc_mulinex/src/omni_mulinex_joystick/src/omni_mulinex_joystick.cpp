#include "omni_mulinex_joystick/omni_mulinex_joystick.hpp"

#include <algorithm>
#include <cmath>

namespace omni_mulinex_joystick {

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
    // Figure-eight trajectory (○)
    this->declare_parameter("figure_eight.size", 1.0);         // metres, half-length of the eight
    this->declare_parameter("figure_eight.max_lin_vel", 0.2);  // m/s
    this->declare_parameter("figure_eight.max_ang_vel", 0.5);  // rad/s
    this->declare_parameter("figure_eight.laps", 1);
    this->declare_parameter("figure_eight.ramp_time", 3.0);    // seconds
    this->declare_parameter("figure_eight.holonomic", false);  // false = differential (vx + wz)

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
    // ○ (btn 1) → start / stop the figure-eight trajectory
    if (btn_rising(1)) {
        if (figure_eight_active_)
            stop_figure_eight("○ pressed");
        else if (demo_active_)
            RCLCPP_WARN(this->get_logger(), "Figure-eight ignored: demo is running");
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
    state_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr /*msg*/)
{
    // State available for future logging/monitoring
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
                            "║  △  → STAND           ○  → toggle figure-eight             ║\n"
                            "║  SELECT → toggle demo cycle                                ║\n"
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
