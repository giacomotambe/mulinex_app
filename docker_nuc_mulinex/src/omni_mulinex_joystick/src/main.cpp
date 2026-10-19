#include "omni_mulinex_joystick/omni_mulinex_joystick.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<omni_mulinex_joystick::OmniMulinexJoystick>());
    rclcpp::shutdown();
    return 0;
}
