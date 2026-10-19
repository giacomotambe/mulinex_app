cmake_minimum_required(VERSION 3.8)
project(robot_model)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

set(LIBRARY_NAME ${PROJECT_NAME})

set(LIBRARY_DEPENDENCIES ament_index_cpp Eigen3 pinocchio)

add_library(${LIBRARY_NAME} SHARED src/robot_wrapper.cpp)

target_include_directories(
    ${LIBRARY_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}> ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${LIBRARY_NAME} PUBLIC yaml-cpp)
ament_target_dependencies(${LIBRARY_NAME} PUBLIC ${LIBRARY_DEPENDENCIES})

ament_export_targets(${LIBRARY_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${LIBRARY_DEPENDENCIES} yaml-cpp)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY robots DESTINATION share/${PROJECT_NAME})

install(
    TARGETS ${LIBRARY_NAME}
    EXPORT ${LIBRARY_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
