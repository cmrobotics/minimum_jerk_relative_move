cmake_minimum_required(VERSION 3.5)
project(minimum_jerk_trajectory_planner)

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

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmr_msgs REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
    src/pose.cpp
    src/acceleration.cpp
    src/robot.cpp
    src/trajectory.cpp
    src/trajectory_planners.cpp
    src/velocity.cpp
    )

add_executable(test_main
    src/main.cpp
)

target_link_libraries(minimum_jerk_trajectory_planner
  minimum_jerk_trajectory_planner_lib
)

set(dependencies
    rclcpp
    std_msgs
    cmr_msgs
)

ament_target_dependencies(${library_name}
    ${dependencies})

install(TARGETS ${library_name}
    EXPORT ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION include/)

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

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
)

ament_package()