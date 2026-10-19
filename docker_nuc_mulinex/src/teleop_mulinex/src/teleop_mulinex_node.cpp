#include "teleop_mulinex_node.hpp"

#include <algorithm>

namespace teleop_mulinex {

TeleopMulinex::TeleopMulinex(): Node("teleop_mulinex")
{
    this->declare_parameter("linear_step", 0.1);
    this->declare_parameter("angular_step", 0.1);
    this->declare_parameter("body_pos_step", 0.005);
    this->declare_parameter("body_ang_step", 0.02);
    this->declare_parameter("max_height", 0.05);
    this->declare_parameter("max_roll", 0.3);
    this->declare_parameter("max_pitch", 0.3);
    this->declare_parameter("max_yaw", 0.15);
    this->declare_parameter("max_pos_x", 0.05);
    this->declare_parameter("max_pos_y", 0.05);

    this->get_parameter("linear_step", linear_step_);
    this->get_parameter("angular_step", angular_step_);
    this->get_parameter("body_pos_step", body_pos_step_);
    this->get_parameter("body_ang_step", body_ang_step_);
    this->get_parameter("max_height", max_height_);
    this->get_parameter("max_roll", max_roll_);
    this->get_parameter("max_pitch", max_pitch_);
    this->get_parameter("max_yaw", max_yaw_);
    this->get_parameter("max_pos_x", max_pos_x_);
    this->get_parameter("max_pos_y", max_pos_y_);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ik_controller/base_pose", 1);

    // Twist publisher with BestEffort QoS and deadline (matching joystick)
    rclcpp::QoS twist_qos(10);
    twist_qos.best_effort();
    twist_qos.deadline(std::chrono::milliseconds(45)); // timer period (40ms) + 5ms
    wheel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/omni_controller/twist_cmd", twist_qos);

    // Service clients
    activate_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/activate_srv");
    emergency_client_ =
        this->create_client<std_srvs::srv::SetBool>("/omni_controller/emergency_srv");
    homing_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/homing_srv");
    rest_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/rest_srv");
    stand_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/stand_srv");
    ik_reinit_client_ =
        this->create_client<std_srvs::srv::SetBool>("/ik_controller/reinitialize_srv");
    ik_activate_client_ =
        this->create_client<std_srvs::srv::SetBool>("/ik_controller/activate_srv");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(40), // 25 Hz
        std::bind(&TeleopMulinex::teleop_callback, this)
    );

    update_messages();
    print_instructions();
}

void TeleopMulinex::teleop_callback()
{
    char c;
    bool dirty = false;

    try {
        if (keyboard_reader_.read_one(&c)) {
            dirty = process_key(c);
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
        return;
    }

    if (dirty) {
        update_messages();
        print_instructions();
    }

    publish_messages();
}

bool TeleopMulinex::process_key(char c)
{
    switch (c) {
    // Wheel velocities
    case KEYCODE_w:
        wheel_vx_ += linear_step_;
        return true;
    case KEYCODE_s:
        wheel_vx_ -= linear_step_;
        return true;
    case KEYCODE_a:
        wheel_vy_ += linear_step_;
        return true;
    case KEYCODE_d:
        wheel_vy_ -= linear_step_;
        return true;
    case KEYCODE_u:
        wheel_omega_ += angular_step_;
        return true;
    case KEYCODE_o:
        wheel_omega_ -= angular_step_;
        return true;

    // Body pose - height
    case KEYCODE_q:
        body_height_ = std::clamp(body_height_ + body_pos_step_, -max_height_, max_height_);
        return true;
    case KEYCODE_e:
        body_height_ = std::clamp(body_height_ - body_pos_step_, -max_height_, max_height_);
        return true;

    // Body pose - pitch
    case KEYCODE_i:
        body_pitch_ = std::clamp(body_pitch_ - body_ang_step_, -max_pitch_, max_pitch_);
        return true;
    case KEYCODE_k:
        body_pitch_ = std::clamp(body_pitch_ + body_ang_step_, -max_pitch_, max_pitch_);
        return true;

    // Body pose - roll
    case KEYCODE_j:
        body_roll_ = std::clamp(body_roll_ - body_ang_step_, -max_roll_, max_roll_);
        return true;
    case KEYCODE_l:
        body_roll_ = std::clamp(body_roll_ + body_ang_step_, -max_roll_, max_roll_);
        return true;

    // Body pose - yaw
    case KEYCODE_n:
        body_yaw_ = std::clamp(body_yaw_ + body_ang_step_, -max_yaw_, max_yaw_);
        return true;
    case KEYCODE_m:
        body_yaw_ = std::clamp(body_yaw_ - body_ang_step_, -max_yaw_, max_yaw_);
        return true;

    // Body pose - x position
    case KEYCODE_r:
        body_x_ = std::clamp(body_x_ + body_pos_step_, -max_pos_x_, max_pos_x_);
        return true;
    case KEYCODE_f:
        body_x_ = std::clamp(body_x_ - body_pos_step_, -max_pos_x_, max_pos_x_);
        return true;

    // Body pose - y position
    case KEYCODE_t:
        body_y_ = std::clamp(body_y_ + body_pos_step_, -max_pos_y_, max_pos_y_);
        return true;
    case KEYCODE_g:
        body_y_ = std::clamp(body_y_ - body_pos_step_, -max_pos_y_, max_pos_y_);
        return true;

    // Services
    case KEYCODE_1:
        // Activate hardware
        call_service(activate_client_, "Activate", true);
        return true;
    case KEYCODE_2:
        // Emergency stop (also deactivates IK)
        deactivate_ik();
        call_service(emergency_client_, "Emergency stop", true);
        return true;
    case KEYCODE_3:
        call_service(homing_client_, "Homing", true);
        return true;
    case KEYCODE_4:
        // Rest (also deactivates IK)
        deactivate_ik();
        call_service(rest_client_, "Rest", true);
        return true;
    case KEYCODE_5:
        // Stand (also deactivates IK)
        deactivate_ik();
        call_service(stand_client_, "Stand", true);
        return true;
    case KEYCODE_6:
        // Activate IK controller (reset body pose, reinit, then activate)
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        call_service(ik_reinit_client_, "IK reinitialize", true);
        call_service(ik_activate_client_, "IK activate", true);
        ik_active_ = true;
        return true;

    // Reset wheels only
    case KEYCODE_v:
        wheel_vx_ = wheel_vy_ = wheel_omega_ = 0.0;
        return true;

    // Reset body only
    case KEYCODE_b:
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        return true;

    // Reset all
    case KEYCODE_SPACE:
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        wheel_vx_ = wheel_vy_ = wheel_omega_ = 0.0;
        return true;
    }

    return false;
}

void TeleopMulinex::update_messages()
{
    auto q = quat_exp_vec({body_roll_, body_pitch_, body_yaw_});

    pose_msg_.position.x = body_x_;
    pose_msg_.position.y = body_y_;
    pose_msg_.position.z = body_height_;
    pose_msg_.orientation.x = q[0];
    pose_msg_.orientation.y = q[1];
    pose_msg_.orientation.z = q[2];
    pose_msg_.orientation.w = q[3];

    wheel_msg_.linear.x = wheel_vx_;
    wheel_msg_.linear.y = wheel_vy_;
    wheel_msg_.angular.z = wheel_omega_;
}

void TeleopMulinex::publish_messages()
{
    wheel_pub_->publish(wheel_msg_);
    if (ik_active_) {
        pose_pub_->publish(pose_msg_);
    }
}

void TeleopMulinex::deactivate_ik()
{
    if (ik_active_) {
        ik_active_ = false;
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        call_service(ik_activate_client_, "IK deactivate", false);
    }
}

void TeleopMulinex::call_service(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr& client, const std::string& name, bool value
)
{
    if (client->service_is_ready()) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = value;
        client->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "%s service called", name.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "%s service not available", name.c_str());
    }
}

void TeleopMulinex::print_instructions()
{
    puts("\033[2J\033[1;1H"); // clear terminal
    puts("Mulinex Teleop - Reading from keyboard");
    puts("---------------------------");
    puts("Wheel velocity commands:");
    puts("  w/s: +/- forward velocity (v_x)");
    puts("  a/d: +/- lateral velocity (v_y)");
    puts("  u/o: +/- angular velocity (omega)");
    puts("---------------------------");
    puts("Body pose commands:");
    puts("  q/e: +/- height (z)");
    puts("  i/k: +/- pitch");
    puts("  j/l: +/- roll");
    puts("  n/m: +/- yaw");
    puts("  r/f: +/- body x position");
    puts("  t/g: +/- body y position");
    puts("---------------------------");
    puts("Services:");
    puts("  1: activate HW");
    puts("  2: emergency stop");
    puts("  3: homing");
    puts("  4: rest");
    puts("  5: stand");
    puts("  6: activate IK (reset body + reinit)");
    puts("---------------------------");
    puts("Resets:");
    puts("  v: reset wheels only");
    puts("  b: reset body only");
    puts("  SPACE: reset all to zero");
    puts("---------------------------");
    printf("Wheel velocity: vx=%.2f, vy=%.2f, omega=%.2f\n", wheel_vx_, wheel_vy_, wheel_omega_);
    printf("Body position: x=%.3f, y=%.3f, z=%.3f\n", body_x_, body_y_, body_height_);
    printf(
        "Body orientation: roll=%.3f, pitch=%.3f, yaw=%.3f\n", body_roll_, body_pitch_, body_yaw_
    );
}

} // namespace teleop_mulinex
