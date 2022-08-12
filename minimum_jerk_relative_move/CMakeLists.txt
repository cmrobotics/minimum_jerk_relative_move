cmake_minimum_required(VERSION 3.5)
project(minimum_jerk_relative_move)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(minimum_jerk_trajectory_planner REQUIRED)
find_package(tf2 REQUIRED)

set(library_name "${PROJECT_NAME}_lib")

add_library(${library_name} 
src/minimum_jerk_relative_move.cpp
    )

target_include_directories(
    ${library_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
    )

add_executable(${PROJECT_NAME}
src/minimum_jerk_relative_move_node.cpp
)

target_link_libraries(minimum_jerk_relative_move
  minimum_jerk_relative_move_lib
)

set(dependencies
    rclcpp
    std_msgs
    cmr_msgs
    geometry_msgs
    minimum_jerk_trajectory_planner
    tf2
)

ament_target_dependencies(${library_name}
    ${dependencies})

install(TARGETS minimum_jerk_relative_move DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    # set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    # set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()