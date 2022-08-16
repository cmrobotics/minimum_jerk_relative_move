cmake_minimum_required(VERSION 3.5)
project(minimum_jerk_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(minimum_jerk_trajectory_planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  cmr_msgs
  geometry_msgs
  minimum_jerk_trajectory_planner
  tf2
  nav2_util
)

add_executable(${PROJECT_NAME} 
               src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  cmr_msgs
  geometry_msgs
  minimum_jerk_trajectory_planner
  tf2
  nav2_util
)

ament_package()
