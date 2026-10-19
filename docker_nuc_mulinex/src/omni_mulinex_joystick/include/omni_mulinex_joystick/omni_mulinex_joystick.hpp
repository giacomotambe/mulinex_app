#pragma once

#include "omni_mulinex_joystick/figure_eight.hpp"
#include "omni_utils/quat_math.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <memory>

namespace omni_mulinex_joystick {

// Demo state
enum class DemoPhase { 
    IDLE, 
    RESET,                  ACTIVATE,
    MOVE_RIGHT,             MOVE_LEFT, 
    MOVE_FORWARD,           MOVE_BACKWARD, 
    YAW_RIGHT,              YAW_LEFT,
    RISE,     
    MOVE_RIGHT_HIGH,        MOVE_LEFT_HIGH, 
    MOVE_FORWARD_HIGH,      MOVE_BACKWARD_HIGH,
    YAW_RIGHT_HIGH,         YAW_LEFT_HIGH,
    ACTIVATE_IK,
    PITCH_FORWARD,          PITCH_BACKWARD,
    PITCH_INVERSE,          PITCH_HOME,
    ROLL_RIGHT,             ROLL_LEFT,
    ROLL_INVERSE,           ROLL_HOME,
    BASE_FORWARD,           BASE_BACKWARD,
    BASE_RISE,              BASE_LOWER,
    SINK, 
    DELAY,
};


class OmniMulinexJoystick: public rclcpp::Node {
public:
    OmniMulinexJoystick();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
    void state_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg);
    void deactivate_ik();
    void print_instructions();
    void set_demo_phase(DemoPhase phase);
    void transition_demo(DemoPhase next, double delay_s = 0.7);
    void start_figure_eight();
    void stop_figure_eight(const char* reason);

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<pi3hat_moteus_int_msgs::msg::JointsStates>::SharedPtr state_sub_;

    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr activate_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr rest_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stand_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ik_reinit_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ik_activate_client_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double sup_vel_x_;
    double sup_vel_y_;
    double sup_omega_;
    double height_step_;
    double yaw_step_;
    double roll_step_;
    double pitch_step_;
    double pos_step_;
    double max_height_;
    double max_roll_;
    double max_pitch_;
    double max_yaw_;
    double max_pos_x_;
    double max_pos_y_;
    double deadzone_;
    int timer_period_ms_;
    bool subscribe_state_;

    // Demo parameters
    double demo_area_width_;
    double demo_area_depth_;
    double demo_travel_time_;
    double demo_yaw_angle_;
    double demo_pitch_angle_;
    double demo_roll_angle_;
    double demo_pose_time_;
    double demo_base_x_;
    double demo_base_z_;
    double demo_current_phase_duration_{0.0};  // set when entering each phase

    // Demo runtime state
    bool       demo_active_{false};
    DemoPhase  demo_phase_{DemoPhase::IDLE};
    double     demo_phase_elapsed_{0.0};
    DemoPhase demo_next_phase_{DemoPhase::IDLE};
    double    demo_delay_time_{0.0};

    // Figure-eight trajectory (○ toggles)
    std::unique_ptr<FigureEight> figure_eight_;
    bool figure_eight_active_{false};
    rclcpp::Time figure_eight_start_time_;

    // Stored joystick analog inputs
    double joy_left_x_ = 0.0;
    double joy_left_y_ = 0.0;
    double joy_right_x_ = 0.0;
    double joy_right_y_ = 0.0;
    bool l2_held_ = false;
    bool ik_active_ = false;

    // State: body pose (accumulated via rate control)
    double body_x_ = 0.0;
    double body_y_ = 0.0;
    double body_height_ = 0.0;
    double body_roll_ = 0.0;
    double body_pitch_ = 0.0;
    double body_yaw_ = 0.0;

    // Edge-detection for buttons
    std::vector<bool> prev_buttons_;

    // Edge-detection for D-pad axes (track previous non-zero state)
    bool prev_dpad_lr_active_ = false;
    bool prev_dpad_ud_active_ = false;
};

} // namespace omni_mulinex_joystick
