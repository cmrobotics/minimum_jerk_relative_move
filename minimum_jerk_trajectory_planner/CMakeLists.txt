cmake_minimum_required(VERSION 3.5)
project(minimum_jerk_trajectory_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(include
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/acceleration.cpp
  src/pose.cpp
  src/robot.cpp
  src/trajectory_planners.cpp
  src/trajectory.cpp
  src/velocity.cpp
)

target_link_libraries(${PROJECT_NAME})

set(dependencies
  "ament_cmake"
  "rclcpp"
  "rclcpp_lifecycle"
  "tf2_geometry_msgs"
  "nav_msgs"
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${dependencies}
  )

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
