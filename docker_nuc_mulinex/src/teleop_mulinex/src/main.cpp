#include "teleop_mulinex_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<teleop_mulinex::TeleopMulinex>();
    rclcpp::spin(node);
    node->shutdown();
    rclcpp::shutdown();
    return 0;
}
