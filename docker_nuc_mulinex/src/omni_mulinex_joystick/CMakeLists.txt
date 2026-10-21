cmake_minimum_required(VERSION 3.8)
project(omni_mulinex_joystick)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pi3hat_moteus_int_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omni_utils REQUIRED)

add_executable(omni_mulinex_joystick_node src/omni_mulinex_joystick.cpp
                                          src/figure_eight.cpp
                                          src/identification.cpp
                                          src/main.cpp)

target_include_directories(
    omni_mulinex_joystick_node
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

ament_target_dependencies(
    omni_mulinex_joystick_node
    rclcpp
    sensor_msgs
    geometry_msgs
    pi3hat_moteus_int_msgs
    std_srvs
    std_msgs
    omni_utils)

install(TARGETS omni_mulinex_joystick_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
