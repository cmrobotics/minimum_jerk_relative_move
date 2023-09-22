cmake_minimum_required(VERSION 3.5)
project(minimum_jerk_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmr_msgs REQUIRED)
find_package(minimum_jerk_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(minimum_jerk_trajectory_planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(cmr_geometry_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cmr_clients_utils REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  cmr_msgs
  nav_msgs
  minimum_jerk_msgs
  geometry_msgs
  minimum_jerk_trajectory_planner
  tf2
  nav2_util
  cmr_geometry_utils
  visualization_msgs
  cmr_clients_utils
)

add_executable(${PROJECT_NAME} 
               src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  cmr_msgs
  nav_msgs
  minimum_jerk_msgs
  geometry_msgs
  minimum_jerk_trajectory_planner
  tf2
  nav2_util
  cmr_geometry_utils
  rclcpp_lifecycle
  lifecycle_msgs
  visualization_msgs
  cmr_clients_utils
)

ament_package()
