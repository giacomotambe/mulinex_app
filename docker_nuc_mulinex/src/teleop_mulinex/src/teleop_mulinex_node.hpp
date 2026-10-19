#pragma once

#include "omni_utils/quat_math.h"
#include "teleop_mulinex/keyboard_reader.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#define KEYCODE_w 0x77
#define KEYCODE_s 0x73
#define KEYCODE_a 0x61
#define KEYCODE_d 0x64
#define KEYCODE_q 0x71
#define KEYCODE_e 0x65
#define KEYCODE_i 0x69
#define KEYCODE_k 0x6b
#define KEYCODE_j 0x6a
#define KEYCODE_l 0x6c
#define KEYCODE_u 0x75
#define KEYCODE_o 0x6f
#define KEYCODE_r 0x72
#define KEYCODE_f 0x66
#define KEYCODE_t 0x74
#define KEYCODE_g 0x67
#define KEYCODE_n 0x6e
#define KEYCODE_m 0x6d
#define KEYCODE_v 0x76
#define KEYCODE_b 0x62
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_SPACE 0x20

namespace teleop_mulinex {

class TeleopMulinex: public rclcpp::Node {
public:
    TeleopMulinex();
    void shutdown() { keyboard_reader_.shutdown(); }

private:
    void teleop_callback();
    bool process_key(char c);
    void update_messages();
    void publish_messages();
    void print_instructions();
    void call_service(
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client, const std::string& name,
        bool value
    );
    void deactivate_ik();

    KeyboardReader keyboard_reader_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_pub_;

    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr activate_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr homing_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr rest_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stand_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ik_reinit_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ik_activate_client_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double linear_step_;
    double angular_step_;
    double body_pos_step_;
    double body_ang_step_;

    // Limits
    double max_height_;
    double max_roll_;
    double max_pitch_;
    double max_yaw_;
    double max_pos_x_;
    double max_pos_y_;

    // State: body pose (Euler angles, converted to quaternion at publish time)
    double body_x_ = 0.0;
    double body_y_ = 0.0;
    double body_height_ = 0.0;
    double body_roll_ = 0.0;
    double body_pitch_ = 0.0;
    double body_yaw_ = 0.0;

    // State: wheel velocities
    double wheel_vx_ = 0.0;
    double wheel_vy_ = 0.0;
    double wheel_omega_ = 0.0;

    // IK active tracking
    bool ik_active_ = false;

    // Messages
    geometry_msgs::msg::Pose pose_msg_;
    geometry_msgs::msg::Twist wheel_msg_;
};

} // namespace teleop_mulinex
