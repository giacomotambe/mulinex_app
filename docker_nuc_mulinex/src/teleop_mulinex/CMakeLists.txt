cmake_minimum_required(VERSION 3.8)
project(teleop_mulinex)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omni_utils REQUIRED)
find_package(std_srvs REQUIRED)
add_executable(teleop_mulinex_node src/teleop_mulinex_node.cpp src/main.cpp)

target_include_directories(
    teleop_mulinex_node
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

ament_target_dependencies(teleop_mulinex_node rclcpp geometry_msgs omni_utils
                          std_srvs)

install(TARGETS teleop_mulinex_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
